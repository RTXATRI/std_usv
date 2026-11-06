// Attach our narrowly scoped VisualPlugin to existing VRX debug-box visuals.
// Scene discovery/retries use wall time, including while Gazebo is paused.
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <ros/ros.h>
#include <xmlrpcpp/XmlRpcValue.h>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <cstdio>
#include <fcntl.h>
#include <memory>
#include <mutex>
#include <netdb.h>
#include <poll.h>
#include <set>
#include <stdexcept>
#include <string>
#include <sys/socket.h>
#include <unistd.h>
#include <vector>

namespace {
struct Configuration {
  std::string world;
  std::string gazebo_world = "robotx_example_course";
  std::string plugin_filename = "libstd_usv_navigation_sensor_visibility.so";
  std::set<std::string> known_docks;
};

Configuration readConfiguration(ros::NodeHandle& nh) {
  Configuration config;
  nh.param("world", config.world, std::string());
  nh.param("plugin_filename", config.plugin_filename, config.plugin_filename);
  const bool explicit_namespace = nh.getParam("gazebo_world", config.gazebo_world);
  XmlRpc::XmlRpcValue docks;
  const bool explicit_docks = nh.getParam("known_docks", docks);
  if (explicit_docks) {
    if (docks.getType() != XmlRpc::XmlRpcValue::TypeArray)
      throw std::invalid_argument("~known_docks must be an array of model instance names");
    for (int i = 0; i < docks.size(); ++i) {
      if (docks[i].getType() != XmlRpc::XmlRpcValue::TypeString)
        throw std::invalid_argument("~known_docks entries must be strings");
      config.known_docks.insert(static_cast<std::string>(docks[i]));
    }
  }
  // This reads only XML metadata: no include/model is spawned or modified.
  if (!config.world.empty()) {
    boost::property_tree::ptree tree;
    boost::property_tree::read_xml(config.world, tree);
    const auto& world = tree.get_child("sdf.world");
    if (!explicit_namespace)
      config.gazebo_world = world.get<std::string>("<xmlattr>.name");
    if (!explicit_docks) {
      for (const auto& element : world) {
        if (element.first != "include") continue;
        const auto name = element.second.get<std::string>("name", "");
        const auto uri = element.second.get<std::string>("uri", "");
        if (name.compare(0, 12, "robotx_dock_") == 0 && uri == "model://dock_2018_dynamic")
          config.known_docks.insert(name);
      }
    }
  }
  if (config.gazebo_world.empty() || config.plugin_filename.empty())
    throw std::invalid_argument("Gazebo world namespace and plugin filename must be nonempty");
  for (const auto& dock : config.known_docks) {
    if (dock.compare(0, 12, "robotx_dock_") != 0 || dock.find("::") != std::string::npos)
      throw std::invalid_argument("Only root robotx_dock_* instance names may be filtered");
  }
  return config;
}

std::string escapeXml(const std::string& text) {
  std::string result;
  for (char character : text) {
    switch (character) {
      case '&': result += "&amp;"; break;
      case '<': result += "&lt;"; break;
      case '>': result += "&gt;"; break;
      case '\"': result += "&quot;"; break;
      case '\'': result += "&apos;"; break;
      default: result += character;
    }
  }
  return result;
}

bool masterPortAvailable(const std::string& host, unsigned int port) {
  addrinfo hints{};
  hints.ai_family = AF_UNSPEC;
  hints.ai_socktype = SOCK_STREAM;
  addrinfo* addresses = nullptr;
  if (getaddrinfo(host.c_str(), std::to_string(port).c_str(), &hints, &addresses) != 0) return false;
  bool connected = false;
  for (auto* address = addresses; address && ros::ok() && !connected; address = address->ai_next) {
    const int connection = socket(address->ai_family, address->ai_socktype, address->ai_protocol);
    if (connection < 0) continue;
    const int flags = fcntl(connection, F_GETFL, 0);
    if (flags >= 0 && fcntl(connection, F_SETFL, flags | O_NONBLOCK) == 0) {
      if (connect(connection, address->ai_addr, address->ai_addrlen) == 0) connected = true;
      else if (errno == EINPROGRESS) {
        pollfd descriptor{connection, POLLOUT, 0};
        if (poll(&descriptor, 1, 100) > 0) {
          int error = 0;
          socklen_t size = sizeof(error);
          connected = getsockopt(connection, SOL_SOCKET, SO_ERROR, &error, &size) == 0 && error == 0;
        }
      }
    }
    close(connection);
  }
  freeaddrinfo(addresses);
  return connected;
}

bool startTransport() {
  std::string host;
  unsigned int port = 0;
  // Gazebo returns false when the environment is unset, but still fills its
  // valid localhost:11345 defaults. Validate the resolved endpoint itself.
  gazebo::transport::get_master_uri(host, port);
  if (host.empty() || port == 0)
    throw std::runtime_error("Invalid GAZEBO_MASTER_URI");
  ROS_INFO("Dock sensor visibility filter: waiting for Gazebo master %s:%u", host.c_str(), port);
  // Gazebo's full client setup also starts FileLogger/ModelDatabase and enters
  // its own master connection retry loop. This transport-only process needs
  // none of those facilities. Keep the ordinary startup wait in the ROS loop
  // so shutdown works even if Gazebo has not been started at all.
  ros::WallRate poll_master(10);
  while (ros::ok() && !masterPortAvailable(host, port)) {
    ros::spinOnce();
    poll_master.sleep();
  }
  if (!ros::ok()) return false;
  ROS_INFO("Dock sensor visibility filter: initializing Gazebo transport at %s:%u", host.c_str(), port);
  if (!gazebo::transport::init(host, port, 1))
    throw std::runtime_error("Cannot initialize Gazebo transport after master became available");
  gazebo::transport::run();
  ROS_INFO("Dock sensor visibility filter: Gazebo transport started");
  return true;
}

bool waitForWorld(const std::string& world) {
  const std::string response_topic = "/gazebo/" + world + "/response";
  ROS_INFO("Dock sensor visibility filter: waiting for world namespace and %s", response_topic.c_str());
  ros::WallRate poll_world(10);
  while (ros::ok()) {
    std::list<std::string> namespaces;
    gazebo::transport::get_topic_namespaces(namespaces);
    if (std::find(namespaces.begin(), namespaces.end(), world) != namespaces.end()) {
      const auto responses = gazebo::transport::getAdvertisedTopics("gazebo.msgs.Response");
      if (std::find(responses.begin(), responses.end(), response_topic) != responses.end()) {
        ROS_INFO("Dock sensor visibility filter: world response publisher is ready");
        return true;
      }
    }
    ros::spinOnce();
    poll_world.sleep();
  }
  return false;
}

class VisualFilterAttacher {
 public:
  explicit VisualFilterAttacher(const Configuration& config) : config_(config) {
    node_.reset(new gazebo::transport::Node());
    ROS_INFO("Dock sensor visibility filter: initializing world namespace %s", config_.gazebo_world.c_str());
    node_->Init(config_.gazebo_world);
    visuals_ = node_->Advertise<gazebo::msgs::Visual>("~/visual", 100);
    requests_ = node_->Advertise<gazebo::msgs::Request>("~/request", 10);
    responses_ = node_->Subscribe("~/response", &VisualFilterAttacher::onResponse, this);
    ROS_INFO("Dock sensor visibility filter: world=%s, %zu allowed dock models",
             config_.gazebo_world.c_str(), config_.known_docks.size());
    ROS_INFO("Dock visibility transport topics: namespace=%s request=%s visual=%s",
             node_->GetTopicNamespace().c_str(), requests_->GetTopic().c_str(), visuals_->GetTopic().c_str());
  }

  void requestScene() {
    const auto now = ros::WallTime::now();
    if (last_diagnostic_.isZero() || (now - last_diagnostic_).toSec() >= 5) {
      ROS_INFO("Dock visibility transport: request_peer=%d visual_peer=%d sent=%u responses=%u matched=%u scenes=%u attachments=%u",
               requests_->HasConnections(), visuals_->HasConnections(), sent_.load(),
               responses_received_.load(), matched_.load(), scenes_.load(), attachments_.load());
      last_diagnostic_ = now;
    }
    if (!requests_->HasConnections()) return;
    std::unique_ptr<gazebo::msgs::Request> request(gazebo::msgs::CreateRequest("scene_info"));
    {
      std::lock_guard<std::mutex> lock(mutex_);
      pending_.insert(request->id());
      // Responses are normally immediate. Bound memory if Gazebo is restarting.
      while (pending_.size() > 32) pending_.erase(pending_.begin());
    }
    requests_->Publish(*request);
    ++sent_;
  }

 private:
  std::string allowedDock(const std::string& name) const {
    std::string scoped = name;
    const auto prefix = config_.gazebo_world + "::";
    if (scoped.compare(0, prefix.size(), prefix) == 0) scoped.erase(0, prefix.size());
    const auto separator = scoped.rfind("::");
    if (separator == std::string::npos) return "";
    const auto leaf = scoped.substr(separator + 2);
    if (leaf != "bay_box" && leaf != "bay_box_exterior") return "";
    const auto parent = scoped.substr(0, separator);
    const std::string bay_suffix = "::bay_visualization";
    if (parent.size() < bay_suffix.size() ||
        parent.compare(parent.size() - bay_suffix.size(), bay_suffix.size(), bay_suffix) != 0)
      return "";
    const auto root = scoped.substr(0, scoped.find("::"));
    return config_.known_docks.count(root) ? root : "";
  }

  void collect(const gazebo::msgs::Model& model, std::vector<gazebo::msgs::Visual>& visuals) {
    for (const auto& visual : model.visual()) visuals.push_back(visual);
    for (const auto& link : model.link())
      for (const auto& visual : link.visual()) visuals.push_back(visual);
    for (const auto& child : model.model()) collect(child, visuals);
  }

  void onResponse(ConstResponsePtr& response) {
    ++responses_received_;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!pending_.erase(response->id())) return;
    }
    ++matched_;
    if (response->request() != "scene_info" || response->response() != "success" ||
        response->type() != "gazebo.msgs.Scene" || !visuals_->HasConnections()) return;
    gazebo::msgs::Scene scene;
    if (!scene.ParseFromString(response->serialized_data())) {
      ROS_WARN("Could not parse Gazebo scene_info for dock visibility filter");
      return;
    }
    ++scenes_;
    std::vector<gazebo::msgs::Visual> candidates;
    for (const auto& model : scene.model()) collect(model, candidates);
    std::set<std::string> visited;
    for (const auto& visual : candidates) {
      const auto dock = allowedDock(visual.name());
      if (dock.empty() || visual.parent_name().empty() || visual.delete_me() ||
          !visited.insert(visual.name()).second) continue;
      // UpdateFromMsg attaches a plugin only if this visual has no existing
      // plugins. Never replace an unrelated plugin or alter physical visuals.
      if (visual.plugin_size() != 0) continue;
      gazebo::msgs::Visual update;
      update.set_name(visual.name());
      update.set_parent_name(visual.parent_name());
      if (visual.has_id()) update.set_id(visual.id());
      if (visual.has_parent_id()) update.set_parent_id(visual.parent_id());
      auto* plugin = update.add_plugin();
      plugin->set_name("std_usv_navigation_sensor_visibility");
      plugin->set_filename(config_.plugin_filename);
      plugin->set_innerxml("<dock_model>" + escapeXml(dock) + "</dock_model>");
      visuals_->Publish(update);
      ++attachments_;
      // Repeat updates on later scene responses: physics scene_info does not
      // acknowledge rendering-only plugins. The rendering loader ignores an
      // already attached plugin, while recreated visuals can attach afresh.
      std::lock_guard<std::mutex> lock(mutex_);
      const std::string identity = visual.name() + "#" + std::to_string(visual.id());
      if (reported_.insert(identity).second)
        ROS_INFO("Requested GUI-only sensor filter attachment: %s", visual.name().c_str());
    }
  }

  Configuration config_;
  gazebo::transport::NodePtr node_;
  gazebo::transport::PublisherPtr visuals_, requests_;
  gazebo::transport::SubscriberPtr responses_;
  std::mutex mutex_;
  std::set<int> pending_;
  std::set<std::string> reported_;
  ros::WallTime last_diagnostic_;
  std::atomic<unsigned int> sent_{0}, responses_received_{0}, matched_{0}, scenes_{0}, attachments_{0};
};
}  // namespace

int main(int argc, char** argv) {
  // roslaunch commonly redirects stdout to a file. Keep startup-stage messages
  // visible immediately instead of waiting for stdio's multi-kilobyte buffer.
  setvbuf(stdout, nullptr, _IOLBF, 0);
  ros::init(argc, argv, "navigation_visual_filter");
  ros::NodeHandle private_nh("~");
  bool gazebo_started = false;
  try {
    const auto config = readConfiguration(private_nh);
    if (!ros::ok()) return 0;
    if (config.known_docks.empty()) {
      ROS_INFO("No known docks require a sensor visibility filter");
      return 0;
    }
    if (!startTransport()) return 0;
    gazebo_started = true;
    if (!waitForWorld(config.gazebo_world)) {
      gazebo::transport::stop(); gazebo::transport::fini();
      return 0;
    }
    {
      VisualFilterAttacher attacher(config);
      ros::WallRate rate(1);
      while (ros::ok()) {
        attacher.requestScene();
        ros::spinOnce();
        rate.sleep();
      }
      // Join transport callbacks while their owning attacher still exists.
      gazebo::transport::stop();
    }
  } catch (const std::exception& error) {
    ROS_FATAL("Dock sensor visibility filter failed: %s", error.what());
    if (gazebo_started) {
      gazebo::transport::stop();
      gazebo::transport::fini();
    }
    return 1;
  }
  gazebo::transport::fini();
  return 0;
}

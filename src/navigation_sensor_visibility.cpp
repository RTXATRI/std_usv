// Exclude VRX docking-volume debug boxes from rendered sensor measurements.
// Loaded at runtime through a Visual message; no world/SDF source is changed.
#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/rendering/Scene.hh>
#include <gazebo/rendering/Visual.hh>

#include <memory>
#include <string>

namespace gazebo {
class NavigationSensorVisibility : public VisualPlugin {
 public:
  void Load(rendering::VisualPtr visual, sdf::ElementPtr parameters) override {
    if (!visual || !parameters || !parameters->HasElement("dock_model")) {
      gzerr << "NavigationSensorVisibility requires an explicit dock_model\n";
      return;
    }
    const std::string dock = parameters->Get<std::string>("dock_model");
    const auto scene = visual->GetScene();
    std::string name = visual->Name();
    if (scene && name.compare(0, scene->Name().size() + 2, scene->Name() + "::") == 0)
      name.erase(0, scene->Name().size() + 2);
    const auto separator = name.rfind("::");
    const std::string leaf = separator == std::string::npos ? name : name.substr(separator + 2);
    const bool debug_box = leaf == "bay_box" || leaf == "bay_box_exterior";
    const bool bay_link = separator != std::string::npos &&
        name.substr(0, separator).size() >= 19 &&
        name.substr(0, separator).compare(name.substr(0, separator).size() - 19,
                                         19, "::bay_visualization") == 0;
    if (dock.compare(0, 12, "robotx_dock_") != 0 ||
        name.compare(0, dock.size() + 2, dock + "::") != 0 || !debug_box || !bay_link) {
      gzerr << "NavigationSensorVisibility rejected non-docking debug visual: "
            << visual->Name() << '\n';
      return;
    }
    visual_ = visual;
    Apply();
    // A visual update can arrive before its geometry. Reapply only if a later
    // geometry load/reset changes the flags, in the rendering thread itself.
    pre_render_ = event::Events::ConnectPreRender([this]() { Apply(); });
    gzmsg << "NavigationSensorVisibility applied GUI-only flags to " << visual->Name()
          << " (server=" << (scene && scene->IsServer() ? "true" : "false") << ")\n";
  }

  void Reset() override { Apply(); }

 private:
  void Apply() {
    const auto visual = visual_.lock();
    if (visual && visual->GetVisibilityFlags() != GZ_VISIBILITY_GUI)
      visual->SetVisibilityFlags(GZ_VISIBILITY_GUI);
  }

  std::weak_ptr<rendering::Visual> visual_;
  event::ConnectionPtr pre_render_;
};
GZ_REGISTER_VISUAL_PLUGIN(NavigationSensorVisibility)
}  // namespace gazebo

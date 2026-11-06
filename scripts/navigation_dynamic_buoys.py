#!/usr/bin/env python3
"""Opt-in dynamic obstacles for the two supported navigation worlds.

Routes reproduce the original three standalone buoy scripts.  Motion is
evaluated from ROS simulation time along a reflected polyline, so a paused
world stays paused and endpoint crossings do not lose travel distance.  The
current routes are managed together by this node; the old standalone scripts
have been removed.
"""

import math
import os


ROUTES = {
    "dynamic_obstacle_1": ((-529., 215.5, .3), (-562., 215.5, .3)),
    "dynamic_obstacle_2": ((-547.5, 234.5, .3), (-547.5, 246.5, .3), (-516., 246.5, .3)),
    "dynamic_obstacle_3": ((-498.5, 196., .3), (-498.5, 234.5, .3)),
}


def routes_for_world(world):
    """Return only models actually present in a supported map variant."""
    name = os.path.splitext(os.path.basename(str(world)))[0]
    names = {"map1-3": ("dynamic_obstacle_1", "dynamic_obstacle_2"),
             "map2-1": ("dynamic_obstacle_3",)}.get(name, ())
    return {name: ROUTES[name] for name in names}


def route_length(waypoints):
    if len(waypoints) < 2 or any(len(point) != 3 or not all(math.isfinite(v) for v in point) for point in waypoints):
        raise ValueError("a route requires at least two finite 3D waypoints")
    lengths = [math.dist(a, b) for a, b in zip(waypoints[:-1], waypoints[1:])]
    if any(length <= 0. for length in lengths):
        raise ValueError("route segments must have positive length")
    return sum(lengths)


def route_position(waypoints, distance):
    """Advance A-B-C-B-A without overshoot, including across several turns."""
    if not math.isfinite(distance) or distance < 0.:
        raise ValueError("travel distance must be nonnegative and finite")
    length = route_length(waypoints)
    position = distance % (2. * length)
    if position > length:
        position = 2. * length - position
    for start, end in zip(waypoints[:-1], waypoints[1:]):
        segment = math.dist(start, end)
        if position <= segment:
            fraction = position / segment
            return tuple(a + fraction * (b - a) for a, b in zip(start, end))
        position -= segment
    return tuple(waypoints[-1])


class NavigationDynamicBuoys:
    def __init__(self, routes):
        import rospy
        from gazebo_msgs.msg import ModelState, ModelStates
        from gazebo_msgs.srv import GetModelState, SetModelState

        self.rospy, self.ModelState = rospy, ModelState
        self.routes = routes
        self.speed = float(rospy.get_param("~speed", .7))
        self.rate = float(rospy.get_param("~rate", 20.))
        if not math.isfinite(self.speed) or self.speed < 0. or not math.isfinite(self.rate) or self.rate <= 0.:
            raise ValueError("speed must be finite/nonnegative and rate finite/positive")
        self.present_models = set()
        self.ready = False
        self.last_time = None
        self.distance = 0.
        self.get_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
        self.set_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
        self.subscriber = rospy.Subscriber("/gazebo/model_states", ModelStates, self.models_callback, queue_size=1)
        # Event-based service availability, with no assumed startup delay.
        rospy.wait_for_service("/gazebo/get_model_state")
        rospy.wait_for_service("/gazebo/set_model_state")
        self.timer = rospy.Timer(rospy.Duration(1. / self.rate), self.update, reset=True)

    def models_callback(self, message):
        self.present_models = set(message.name)

    def update(self, _event):
        now = self.rospy.Time.now().to_sec()
        if not math.isfinite(now) or now <= 0.:
            return
        if self.last_time is not None and now < self.last_time:
            self.ready = False
            self.distance = 0.
            self.last_time = None
        if not self.ready:
            if not all(name in self.present_models for name in self.routes):
                self.last_time = now
                return
            try:
                for name in self.routes:
                    response = self.get_state(name, "world")
                    if not response.success:
                        self.rospy.logwarn_throttle(5., "Waiting for dynamic buoy %s: %s", name, response.status_message)
                        self.last_time = now
                        return
            except self.rospy.ServiceException as error:
                self.rospy.logwarn_throttle(5., "Waiting for dynamic buoy state service: %s", str(error))
                self.last_time = now
                return
            self.ready = True
            self.last_time = now
            return
        elapsed = now - self.last_time
        self.last_time = now
        if elapsed <= 0.:
            return
        distance = self.distance + self.speed * elapsed
        try:
            for name, route in self.routes.items():
                position = route_position(route, distance)
                state = self.ModelState()
                state.model_name = name
                state.reference_frame = "world"
                state.pose.position.x, state.pose.position.y, state.pose.position.z = position
                state.pose.orientation.w = 1.
                response = self.set_state(state)
                if not response.success:
                    self.ready = False
                    self.rospy.logwarn_throttle(5., "Dynamic buoy update failed for %s: %s", name, response.status_message)
                    return
            self.distance = distance
        except self.rospy.ServiceException as error:
            self.ready = False
            self.rospy.logwarn_throttle(5., "Dynamic buoy update service failed: %s", str(error))


def main():
    import rospy
    rospy.init_node("navigation_dynamic_buoys")
    routes = routes_for_world(rospy.get_param("~world", ""))
    if not routes:
        rospy.loginfo("This navigation world has no supported dynamic buoy routes; exiting.")
        return
    try:
        NavigationDynamicBuoys(routes)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()

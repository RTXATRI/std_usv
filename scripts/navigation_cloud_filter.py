#!/usr/bin/env python3
"""Crop self returns in body coordinates, preserving the raw lidar interface."""
import numpy as np


def filter_points(points, transform, footprint, minimum_height=0.05,
                  maximum_height=2.0, self_margin=0.05):
    """Return finite XYZ points outside the convex physical hull."""
    points = np.asarray(points, dtype=float).reshape((-1, 3))
    transform = np.asarray(transform, dtype=float)
    hull = np.asarray(footprint, dtype=float)
    if (transform.shape != (4, 4) or not np.isfinite(transform).all()
            or hull.ndim != 2 or hull.shape[1] != 2 or len(hull) < 3
            or not np.isfinite(hull).all()):
        raise ValueError("Invalid rigid transform or physical footprint")
    points = points[np.isfinite(points).all(axis=1)]
    body = points.dot(transform[:3, :3].T) + transform[:3, 3]
    body = body[(body[:, 2] >= minimum_height)
                & (body[:, 2] <= maximum_height)]
    orientation = np.sign(np.sum(hull[:, 0] * np.roll(hull[:, 1], -1)
                                 - hull[:, 1] * np.roll(hull[:, 0], -1)))
    if orientation == 0:
        raise ValueError("Degenerate physical footprint")
    inside = np.ones(len(body), dtype=bool)
    for a, b in zip(hull, np.roll(hull, -1, axis=0)):
        edge = b - a
        cross = edge[0] * (body[:, 1] - a[1]) - edge[1] * (body[:, 0] - a[0])
        inside &= orientation * cross >= -self_margin * np.linalg.norm(edge)
    return body[~inside]


def main():
    import rospy
    import tf2_ros
    from tf.transformations import quaternion_matrix
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs import point_cloud2
    from std_msgs.msg import Header

    rospy.init_node("navigation_cloud_filter")
    frame = rospy.get_param("~target_frame", "wamv/base_link")
    footprint = rospy.get_param("/wamv/navigation/model/footprint")
    minimum = rospy.get_param("~min_height", 0.05)
    maximum = rospy.get_param("~max_height", 2.0)
    margin = rospy.get_param("~self_margin", 0.05)
    buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10))
    listener = tf2_ros.TransformListener(buffer)
    publisher = rospy.Publisher("/wamv/navigation/points_filtered", PointCloud2,
                                queue_size=1)

    def receive(message):
        try:
            mounted = buffer.lookup_transform(frame, message.header.frame_id,
                                              message.header.stamp,
                                              rospy.Duration(0.1)).transform
            q = mounted.rotation
            transform = quaternion_matrix([q.x, q.y, q.z, q.w])
            transform[:3, 3] = [mounted.translation.x, mounted.translation.y,
                                mounted.translation.z]
            xyz = list(point_cloud2.read_points(message, field_names=("x", "y", "z"),
                                               skip_nans=True))
            filtered = filter_points(xyz, transform, footprint, minimum, maximum, margin)
            header = Header(stamp=message.header.stamp, frame_id=frame)
            publisher.publish(point_cloud2.create_cloud_xyz32(header, filtered))
        except (tf2_ros.TransformException, ValueError, KeyError) as error:
            rospy.logwarn_throttle(2, "Navigation cloud unavailable: %s", error)

    rospy.Subscriber("/wamv/sensors/lidars/lidar_wamv/points", PointCloud2,
                     receive, queue_size=1, buff_size=4194304)
    rospy.spin()


if __name__ == "__main__":
    main()

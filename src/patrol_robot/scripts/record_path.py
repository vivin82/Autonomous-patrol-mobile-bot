#!/usr/bin/env python3
import rospy
import tf
import yaml
import math
import os

WAYPOINT_FILE = os.path.expanduser("~/patrol_ws/src/patrol_robot/maps/waypoints.yaml")
MIN_DIST = 0.1  # Record a waypoint every 0.5 metres

waypoints = []

def save_and_exit():
    os.makedirs(os.path.dirname(WAYPOINT_FILE), exist_ok=True)
    with open(WAYPOINT_FILE, "w") as f:
        yaml.dump({"waypoints": waypoints}, f)
    rospy.loginfo(f"Saved {len(waypoints)} waypoints to {WAYPOINT_FILE}")

def record_path():
    rospy.init_node("record_path")
    rospy.on_shutdown(save_and_exit)
    
    # Listen directly to the robot's spatial geometry (TF)
    listener = tf.TransformListener()
    rospy.loginfo("Waiting for TF data... Drive the robot. Press Ctrl+C when done.")
    
    last_x, last_y = None, None
    rate = rospy.Rate(10.0) # Check 10 times a second
    
    while not rospy.is_shutdown():
        try:
            # Look up the exact position of the robot relative to the starting point
            (trans, rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            # If TF isn't ready yet, just wait and try again
            rate.sleep()
            continue
            
        x, y = trans[0], trans[1]
        
        # Convert quaternion to yaw angle
        yaw = tf.transformations.euler_from_quaternion(rot)[2]
        
        if last_x is None:
            last_x, last_y = x, y
            waypoints.append({"x": round(x, 3), "y": round(y, 3), "yaw": round(yaw, 3)})
            rospy.loginfo(f"First waypoint recorded: ({x:.2f}, {y:.2f})")
        else:
            dist = math.sqrt((x - last_x)**2 + (y - last_y)**2)
            if dist >= MIN_DIST:
                last_x, last_y = x, y
                waypoints.append({"x": round(x, 3), "y": round(y, 3), "yaw": round(yaw, 3)})
                rospy.loginfo(f"Waypoint {len(waypoints)} recorded: ({x:.2f}, {y:.2f})")
        
        rate.sleep()

if __name__ == '__main__':
    try:
        record_path()
    except rospy.ROSInterruptException:
        pass

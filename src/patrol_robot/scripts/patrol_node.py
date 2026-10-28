#!/usr/bin/env python3
import rospy
import yaml
import actionlib
import tf
import os
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Quaternion

# Pointing to the exact same file in your patrol_ws
WAYPOINT_FILE = os.path.expanduser("~/patrol_ws/src/patrol_robot/maps/waypoints.yaml")
RETRY_WAIT  = 3.0 
MAX_RETRIES = 3 

def load_waypoints():
    if not os.path.exists(WAYPOINT_FILE):
        rospy.logerr(f"Waypoint file not found at {WAYPOINT_FILE}")
        return []
    with open(WAYPOINT_FILE, "r") as f:
        data = yaml.safe_load(f)
    return data["waypoints"]

def make_goal(wp):
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp    = rospy.Time.now()
    goal.target_pose.pose.position.x = wp["x"]
    goal.target_pose.pose.position.y = wp["y"]
    q = tf.transformations.quaternion_from_euler(0, 0, wp.get("yaw", 0.0))
    goal.target_pose.pose.orientation = Quaternion(*q)
    return goal

def patrol():
    rospy.init_node("patrol_node")
    client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    
    rospy.loginfo("Waiting for move_base...")
    if not client.wait_for_server(rospy.Duration(10)):
        rospy.logerr("move_base server not found! Exiting.")
        return

    waypoints = load_waypoints()
    if not waypoints:
        return
        
    rospy.loginfo(f"Loaded {len(waypoints)} waypoints. Starting patrol.")

    idx = 0
    while not rospy.is_shutdown():
        wp = waypoints[idx]
        rospy.loginfo(f"Going to waypoint {idx}: ({wp['x']:.2f}, {wp['y']:.2f})")

        success = False
        for retries in range(MAX_RETRIES):
            if rospy.is_shutdown(): break
            
            goal = make_goal(wp)
            client.send_goal(goal)
            
            # Wait 45s for result
            finished = client.wait_for_result(rospy.Duration(45.0))

            if finished:
                state = client.get_state()
                if state == 3: # SUCCEEDED
                    rospy.loginfo(f"SUCCESS: Reached waypoint {idx}")
                    success = True
                    break
                else:
                    rospy.logwarn(f"FAILED/BLOCKED: Retry {retries+1}/{MAX_RETRIES}")
                    rospy.sleep(RETRY_WAIT)
            else:
                client.cancel_goal()
                rospy.logwarn(f"TIMEOUT: Robot was likely blocked at waypoint {idx}")
                break 

        idx = (idx + 1) % len(waypoints)

if __name__ == "__main__":
    try:
        patrol()
    except rospy.ROSInterruptException:
        pass

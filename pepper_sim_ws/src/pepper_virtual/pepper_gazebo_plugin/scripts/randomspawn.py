#!/usr/bin/env python3
import rospy
import random
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState

def random_spawn():
    rospy.init_node("pepper_random_spawn")

    rospy.wait_for_service("/gazebo/set_model_state")
    set_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)

    spawn_positions = [
        {"x": 0.04, "y": -5.17},
        {"x": -1.3, "y": -4.597},
        {"x": -3.0, "y": 0.4},
        {"x": -2.0, "y": 2.0},
    ]

    pos = random.choice(spawn_positions)

    state = ModelState()
    state.model_name = "pepper_MP"
    state.pose.position.x = pos["x"]
    state.pose.position.y = pos["y"]
    state.pose.position.z = 0.0
    state.pose.orientation.w = 1.0

    resp = set_state(state)
    rospy.loginfo("Spawned Pepper at (%.2f, %.2f), success=%s",
                  pos["x"], pos["y"], str(resp.success))

if __name__ == "__main__":
    try:
        random_spawn()
    except rospy.ROSInterruptException:
        pass

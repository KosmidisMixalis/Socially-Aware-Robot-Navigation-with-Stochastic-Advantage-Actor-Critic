#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SpawnModel, DeleteModel,SetModelState, GetModelState
from geometry_msgs.msg import Pose, Quaternion
import math
from tf.transformations import quaternion_from_euler
import random

def spawn_model(model_name, path):
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    
    try:
        # Create the service client
        spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        
        # Read the SDF file
        sdf_path = path
        with open(sdf_path, "r") as f:
            model_xml = f.read()
        
        # Set the model name and pose
        model_name = model_name
        pose = Pose()
        pose.position.x = -8.0
        pose.position.y = 0.0
        pose.position.z = 0.0
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0
        pose.orientation.w = 0.0

        
        # Call the service to spawn the model
        spawn_model(model_name, model_xml, "", pose, "world")
        rospy.loginfo("Actor spawned successfully!")
    
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)

def get_model_pose(model_name):
    rospy.wait_for_service('/gazebo/get_model_state')
    get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    resp = get_model_state(model_name, '')
    return resp.pose

def delete_model(model_name):
    rospy.wait_for_service('/gazebo/delete_model')
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    try:
        
        delete_service = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        delete_service(model_name)  
        
        rospy.loginfo("Actor deleted. Re-spawning...")

    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")


def reset_model(model_name, x, y, angle):

    rospy.wait_for_service('/gazebo/set_model_state')
    try:
        reset_service = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        state_msg = ModelState()
        state_msg.model_name = model_name  
        state_msg.pose.position.x = x
        state_msg.pose.position.y = y
        state_msg.pose.position.z = 1.0  

        yaw = math.radians(angle) 
        q = quaternion_from_euler(0, 0, yaw)  

        orientation = Quaternion()
        orientation.x = q[0]
        orientation.y = q[1]
        orientation.z = q[2]
        orientation.w = q[3]

        state_msg.pose.orientation = Quaternion(*q)

        reset_service(state_msg)

        

        rospy.loginfo("Rover position reset to initial state.")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")




def reset():
    scenarioRobot = {1: [2, 0, 180], 2: [5, -6, 90], 3: [6, -8, 180]} # robot starting pose per scenario

    scenarioHuman = {1: [-6, 0.5, -90], 2: [4.5, 4, 180], 3: [-2.5, -9, -90]} # goal pose per scenario
    
    common_keys = list(set(scenarioRobot.keys()) & set(scenarioHuman.keys()))

    random_key = random.choice(common_keys)

    robot_position = scenarioRobot[random_key]
    human_position = scenarioHuman[random_key]

    reset_model("ridgeback", robot_position[0], robot_position[1], robot_position[2])
    reset_model("pedestrian", human_position[0], human_position[1], human_position[2])


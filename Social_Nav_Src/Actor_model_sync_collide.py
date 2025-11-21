#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SpawnModel, SetModelState, GetModelState
from geometry_msgs.msg import Pose

def spawn_model_cylinder(model_name, path, spawn_pose):
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
        pose.position.x = spawn_pose.position.x
        pose.position.y = spawn_pose.position.y
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

def set_pose(model_name, x, y):
    rospy.wait_for_service('/gazebo/set_model_state')
    try:
        reset_service = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        
        state_msg = ModelState()
        state_msg.model_name = model_name
        state_msg.pose.position.x = x
        state_msg.pose.position.y = y
        state_msg.pose.position.z = 0.5  # Ground level
        
        state_msg.reference_frame = "world"
        reset_service(state_msg)
        
        
    except rospy.ServiceException as e:
        rospy.logerr(f"Failed to reset model {model_name}: {e}")




def actor_collision():
    

    pose2 = get_model_pose("actor1")
    set_pose("cylinder1", pose2.position.x, pose2.position.y)



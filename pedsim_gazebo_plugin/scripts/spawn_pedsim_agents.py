#!/usr/bin/env python
"""
Created on Mon Dec  2 17:03:34 2019

@author: mahmoud
"""

import rospy
from gazebo_msgs.srv import SpawnModel, SpawnModelRequest, DeleteModel, SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import *
from rospkg import RosPack
from pedsim_msgs.msg import AgentStates

# xml file containing a gazebo model to represent agent, currently is represented by a cubic but can be changed
xml_string = ''
agents_list = []


def actor_poses_callback(actors):
    global agents_list
    global xml_string
    current_agents = []

    for actor in actors.agent_states:
        current_agents.append(actor.id)

        actor_id = str(actor.id)
        if actor.id not in agents_list:  # spawn unless spawned before
            req = SpawnModelRequest()
            req.model_name = 'actor_' + actor_id
            req.model_xml = xml_string
            req.initial_pose = actor.pose
            req.reference_frame = "world"
            req.robot_namespace = 'actor_' + actor_id

            # rospy.logerr("SPAWNING: {}".format(actor_id))
            spawn_model(req)
            
        else:  # just update the pose if spawned before
            state = ModelState()
            state.model_name = 'actor_' + actor_id
            state.pose = actor.pose
            state.reference_frame = "world"
            try:
                # rospy.logwarn("{}: set_state service call".format(actor_id))
                set_state(state)
            except rospy.ServiceException, e:
                rospy.logerr("set_state failed: %s" % e)

    for agent in agents_list:
        if agent not in current_agents:  # agent left the scene
            try:
                # rospy.logwarn("{} delete_model service call".format(str(agent)))
                agents_list.remove(agent)
                delete_model('actor_' + str(agent))
            except rospy.ServiceException, e:
                rospy.logerr("delete_model failed: %s" % e)
            
    # update agents_list
    agents_list = current_agents


if __name__ == '__main__':
    rospy.init_node("manipulate_pedsim_agents")
    
    rospack1 = RosPack()
    pkg_path = rospack1.get_path('pedsim_gazebo_plugin')
#    file_xml = open(pkg_path + "/models/humanbody/model.sdf")
    file_xml = open(pkg_path + "/models/model.sdf")
    xml_string = file_xml.read()

    agents_list = []

    print("Waiting for gazebo services...")
    rospy.wait_for_service('gazebo/spawn_sdf_model')
    rospy.wait_for_service('gazebo/delete_model')
    rospy.wait_for_service('/gazebo/set_model_state')
    
    spawn_model = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
    delete_model = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)
    set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    print("Services are available!")
    
    rospy.Subscriber("/pedsim_simulator/simulated_agents", AgentStates, actor_poses_callback, queue_size=1)

    rospy.spin()

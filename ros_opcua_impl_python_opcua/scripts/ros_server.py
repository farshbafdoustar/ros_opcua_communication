#!/usr/bin/env python
from multiprocessing.connection import Client
from pydoc import cli
from pydoc_data.topics import topics
import sys
import this
import time
from webbrowser import get

import rosgraph
import rosnode
import rospy
from opcua import Server, ua


import ros_services
import ros_topics


from std_msgs.msg import *
from control_msgs.msg import *
from trajectory_msgs import *

#client = ros_client.ROSClient("opc.tcp://0.0.0.0:21554/")

# Returns the hierachy as one string from the first remaining part on.


def nextname(hierachy, index_of_last_processed):
    try:
        output = ""
        counter = index_of_last_processed + 1
        while counter < len(hierachy):
            output += hierachy[counter]
            counter += 1
        return output
    except Exception as e:
        rospy.logerr("Error encountered ", e)


# def own_rosnode_cleanup():
#     pinged, unpinged = rosnode.rosnode_ping_all()
#     if unpinged:
#         master = rosgraph.Master(rosnode.ID)
#         # noinspection PyTypeChecker
#         rosnode.cleanup_master_blacklist(master, unpinged)


class ROSServer:
    
    def __init__(self):
        self.namespace_ros = rospy.get_param("/rosopcua/namespace")
        self.topicsDict = {}
        self.servicesDict = {}
        self.actionsDict = {}
        rospy.init_node("rosopcua")
        self.server = Server()
        self.server.set_endpoint("opc.tcp://10.33.1.124:21554/")
        self.server.set_server_name("ROS ua Server")
        self.server.start()

        self.method_map = None
        self.INPUT_TOPIC="INPUT"
        self.OUTPUT_TOPIC="OUTPUT"

        # setup our own namespaces, this is expected
        uri_topics = "http://ros.org/topics"
        # two different namespaces to make getting the correct node easier for get_node (otherwise had object for service and topic with same name
        uri_services = "http://ros.org/services"
        uri_actions = "http://ros.org/actions"
        idx_topics = self.server.register_namespace(uri_topics)
        idx_services = self.server.register_namespace(uri_services)
        idx_actions = self.server.register_namespace(uri_actions)
        # get Objects node, this is where we should put our custom stuff
        objects = self.server.get_objects_node()

        # one object per type we are watching
        topics_object = objects.add_object(idx_topics, "ROS-Topics")
        services_object = objects.add_object(idx_services, "ROS-Services")
        actions_object = objects.add_object(idx_actions, "ROS_Actions")

        # hh
        # self.client = Client("opc.tcp://0.0.0.0:21554/",timeout=10)
        # self.client.connect()
        # print("client conected")

        # while not rospy.is_shutdown():

        all_topics_lst = self.get_all_topics()

        # ros_topics starts a lot of publisher/subscribers, might slow everything down quite a bit.
        ros_services.refresh_services(self.namespace_ros, self, self.servicesDict, idx_services, services_object)
        self.method_map = ros_topics.refresh_topics_and_actions(self.namespace_ros, self, self.topicsDict, self.actionsDict,
                                                                idx_topics, idx_actions, topics_object, actions_object, all_topics_lst)
        # hh

        sub_lst = []
        for key in self.method_map:
            for topic_name, topic_type,io_type in all_topics_lst:
                if io_type==self.OUTPUT_TOPIC and topic_name in key:
                    leaf_node = self.server.get_node(key)
                    sub_lst.append(leaf_node)            
        try:
            
            sub = self.server.create_subscription(period=1, handler=self)
            handle = sub.subscribe_data_change(sub_lst)
        except:
            print("Can not create_subscription for this object:",key)
            
            #print(sub_lst)

        #print(self.method_map)
        print("ROS OPCUA Server initialized.")
        while not rospy.is_shutdown():
            rospy.spin()
       
        #time.sleep(0)  # used to be 60

        self.server.stop()
        print("ROS OPCUA Server stopped.")
        quit()
        
        

    def get_all_topics(self):
        # function to provide topics
       
        all_ros_topics = []
        all_ros_topics.append(['/teststation/controller/digital_output_activate_head_thread_cutter/commands', 'io_controllers_msgs/DigitalStateCommand',self.OUTPUT_TOPIC])
        all_ros_topics.append(['/teststation/controller/digital_output_activate_head_thread_tension/commands', 'io_controllers_msgs/DigitalStateCommand',self.OUTPUT_TOPIC])
        all_ros_topics.append(['/teststation/controller/digital_output_close_sewing_head/commands', 'io_controllers_msgs/DigitalStateCommand',self.OUTPUT_TOPIC])
        all_ros_topics.append(['/teststation/controller/digital_output_open_sewing_head/commands', 'io_controllers_msgs/DigitalStateCommand',self.OUTPUT_TOPIC])
        all_ros_topics.append(['/teststation/controller/digital_output_unlock_tool_changer/commands', 'io_controllers_msgs/DigitalStateCommand',self.OUTPUT_TOPIC])
        #all_ros_topics.append(['/joint_states', 'sensor_msgs/JointState',self.INPUT_TOPIC])
        all_ros_topics.append(['/teststation/controller/digital_input_head_thread_cutter/states', 'io_controllers_msgs/DigitalStateCommand',self.INPUT_TOPIC])
        all_ros_topics.append(['/teststation/controller/digital_input_head_opening/states', 'io_controllers_msgs/DigitalStateCommand',self.INPUT_TOPIC])
        all_ros_topics.append(['/teststation/controller/digital_input_head_needle_in_cutting_position/states', 'io_controllers_msgs/DigitalStateCommand',self.INPUT_TOPIC])
        all_ros_topics.append(['/teststation/controller/digital_input_head_needle_on_zero/states', 'io_controllers_msgs/DigitalStateCommand',self.INPUT_TOPIC])
        all_ros_topics.append(['/teststation/controller/digital_input_tool_changer_lock/states', 'io_controllers_msgs/DigitalStateCommand',self.INPUT_TOPIC])

        # all_ros_topics.append(['/workcell_smp_irb2600/controller/position_trajectory_controller/state', 'control_msgs/JointTrajectoryControllerState',self.INPUT_TOPIC])
        # all_ros_topics.append(['/workcell_smp_irb2600/controller/table_controller/state', 'control_msgs/JointTrajectoryControllerState',self.INPUT_TOPIC])

        # all_ros_topics.append(['/workcell_smp_irb2600/controller/position_trajectory_controller/command', 'trajectory_msgs/JointTrajectory',self.INPUT_TOPIC])
        # all_ros_topics.append(['/workcell_smp_irb2600/controller/table_controller/command', 'trajectory_msgs/JointTrajectory',self.INPUT_TOPIC])

        # all_ros_topics.append(['/workcell_smp_irb2600/controller/camera_hw_trigger_state/world_to_tcp_transform', 'geometry_msgs/TransformStamped',self.INPUT_TOPIC])
        # all_ros_topics.append(['/workcell_smp_irb2600/controller/joint_based_transform/world_to_tcp_transform', 'geometry_msgs/TransformStamped',self.INPUT_TOPIC])

        # all_ros_topics.append(['/workcell_smp_irb2600/controller/analog_input_vacuum_sensors/states', 'io_controllers_msgs/DigitalStateCommand',self.INPUT_TOPIC])
        # all_ros_topics.append(['/workcell_smp_irb2600/controller/camera_hw_trigger_state/states', 'io_controllers_msgs/DigitalStateCommandy',self.INPUT_TOPIC])
        # all_ros_topics.append(['/workcell_smp_irb2600/controller/digital_input_head_opening/states', 'io_controllers_msgs/DigitalStateCommand',self.INPUT_TOPIC])
        # all_ros_topics.append(['/workcell_smp_irb2600/controller/digital_input_head_thread_cutter/states', 'io_controllers_msgs/DigitalStateCommand',self.INPUT_TOPIC])
        # all_ros_topics.append(['/workcell_smp_irb2600/controller/digital_input_tool_changer_lock/states', 'io_controllers_msgs/DigitalStateCommand',self.INPUT_TOPIC])
        # all_ros_topics.append(['/workcell_smp_irb2600/controller/digital_output_activate_head_air_blowing/states', 'io_controllers_msgs/DigitalStateCommand',self.INPUT_TOPIC])
        # all_ros_topics.append(['/workcell_smp_irb2600/controller/digital_output_activate_head_thread_cutter/states', 'io_controllers_msgs/DigitalStateCommand',self.INPUT_TOPIC])
        # all_ros_topics.append(['/workcell_smp_irb2600/controller/digital_output_activate_head_thread_tension/states', 'io_controllers_msgs/DigitalStateCommand',self.INPUT_TOPIC])
        # all_ros_topics.append(['/workcell_smp_irb2600/controller/digital_output_activate_hmi_right_green_button/states', 'io_controllers_msgs/DigitalStateCommand',self.INPUT_TOPIC])
        # all_ros_topics.append(['/workcell_smp_irb2600/controller/digital_output_activate_laser_left/states', 'io_controllers_msgs/DigitalStateCommand',self.INPUT_TOPIC])
        # all_ros_topics.append(['/workcell_smp_irb2600/controller/digital_output_activate_laser_middle/states', 'io_controllers_msgs/DigitalStateCommand',self.INPUT_TOPIC])
        # all_ros_topics.append(['/workcell_smp_irb2600/controller/digital_output_activate_laser_middle_left/states', 'io_controllers_msgs/DigitalStateCommand',self.INPUT_TOPIC])
        # all_ros_topics.append(['/workcell_smp_irb2600/controller/digital_output_activate_laser_middle_right/states', 'io_controllers_msgs/DigitalStateCommand',self.INPUT_TOPIC])
        # all_ros_topics.append(['/workcell_smp_irb2600/controller/digital_output_activate_laser_right/states', 'io_controllers_msgs/DigitalStateCommand',self.INPUT_TOPIC])
        # all_ros_topics.append(['/workcell_smp_irb2600/controller/digital_output_activate_sewing_light/states', 'io_controllers_msgs/DigitalStateCommand',self.INPUT_TOPIC])
        # all_ros_topics.append(['/workcell_smp_irb2600/controller/digital_output_compressor_fast/states', 'io_controllers_msgs/DigitalStateCommand',self.INPUT_TOPIC])
        # all_ros_topics.append(['/workcell_smp_irb2600/controller/digital_output_compressor_medium/states', 'io_controllers_msgs/DigitalStateCommand',self.INPUT_TOPIC])
        # all_ros_topics.append(['/workcell_smp_irb2600/controller/digital_output_compressor_slow/states', 'io_controllers_msgs/DigitalStateCommand',self.INPUT_TOPIC])
        # all_ros_topics.append(['/workcell_smp_irb2600/controller/digital_output_open_sewing_head/states', 'io_controllers_msgs/DigitalStateCommand',self.INPUT_TOPIC])
        # all_ros_topics.append(['/workcell_smp_irb2600/controller/digital_output_vacuum_valve_zone_1/states', 'io_controllers_msgs/DigitalStateCommand',self.INPUT_TOPIC])
        # all_ros_topics.append(['/workcell_smp_irb2600/controller/digital_output_vacuum_valve_zone_2/states', 'io_controllers_msgs/DigitalStateCommand',self.INPUT_TOPIC])
        # all_ros_topics.append(['/workcell_smp_irb2600/controller/digital_output_vacuum_valve_zone_3/states', 'io_controllers_msgs/DigitalStateCommand',self.INPUT_TOPIC])
        # all_ros_topics.append(['/workcell_smp_irb2600/controller/digital_output_vacuum_valve_zone_4/states', 'io_controllers_msgs/DigitalStateCommand',self.INPUT_TOPIC])
        # all_ros_topics.append(['/workcell_smp_irb2600/controller/digital_output_vacuum_valve_zone_5/states', 'io_controllers_msgs/DigitalStateCommand',self.INPUT_TOPIC])
        # all_ros_topics.append(['/workcell_smp_irb2600/controller/digital_output_vacuum_valve_zone_6/states', 'io_controllers_msgs/DigitalStateCommand',self.INPUT_TOPIC])
        # all_ros_topics.append(['/workcell_smp_irb2600/controller/digital_output_vacuum_valve_zone_7/states', 'io_controllers_msgs/DigitalStateCommand',self.INPUT_TOPIC])
        # all_ros_topics.append(['/workcell_smp_irb2600/controller/digital_output_vacuum_valve_zone_8/states', 'io_controllers_msgs/DigitalStateCommand',self.INPUT_TOPIC])
        # all_ros_topics.append(['/workcell_smp_irb2600/controller/digital_output_vacuum_valve_zone_9/states', 'io_controllers_msgs/DigitalStateCommand',self.INPUT_TOPIC])
        # all_ros_topics.append(['/workcell_smp_irb2600/controller/digital_output_vacuum_valve_zone_10/states', 'io_controllers_msgs/DigitalStateCommand',self.INPUT_TOPIC])

        # all_ros_topics.append(['/workcell_smp_irb2600/controller/camera_hw_trigger_state/commands', 'io_controllers_msgs/DigitalStateCommand',self.INPUT_TOPIC])
        # all_ros_topics.append(['/workcell_smp_irb2600/controller/digital_output_activate_head_air_blowing/commands', 'io_controllers_msgs/DigitalStateCommand',self.INPUT_TOPIC])
        # all_ros_topics.append(['/workcell_smp_irb2600/controller/digital_output_activate_head_thread_cutter/commands', 'io_controllers_msgs/DigitalStateCommand',self.INPUT_TOPIC])
        # all_ros_topics.append(['/workcell_smp_irb2600/controller/digital_output_activate_head_thread_tension/commands', 'io_controllers_msgs/DigitalStateCommand',self.INPUT_TOPIC])
        # all_ros_topics.append(['/workcell_smp_irb2600/controller/digital_output_activate_hmi_right_green_button/commands', 'io_controllers_msgs/DigitalStateCommand',self.INPUT_TOPIC])
        # all_ros_topics.append(['/workcell_smp_irb2600/controller/digital_output_activate_laser_left/commands', 'io_controllers_msgs/DigitalStateCommand',self.INPUT_TOPIC])
        # all_ros_topics.append(['/workcell_smp_irb2600/controller/digital_output_activate_laser_middle/commands', 'io_controllers_msgs/DigitalStateCommand',self.INPUT_TOPIC])
        # all_ros_topics.append(['/workcell_smp_irb2600/controller/digital_output_activate_laser_middle_left/commands', 'io_controllers_msgs/DigitalStateCommand',self.INPUT_TOPIC])
        # all_ros_topics.append(['/workcell_smp_irb2600/controller/digital_output_activate_laser_middle_right/commands', 'io_controllers_msgs/DigitalStateCommand',self.INPUT_TOPIC])
        # all_ros_topics.append(['/workcell_smp_irb2600/controller/digital_output_activate_laser_right/commands', 'io_controllers_msgs/DigitalStateCommand',self.INPUT_TOPIC])
        # all_ros_topics.append(['/workcell_smp_irb2600/controller/digital_output_activate_sewing_light/commands', 'io_controllers_msgs/DigitalStateCommand',self.INPUT_TOPIC])
        # all_ros_topics.append(['/workcell_smp_irb2600/controller/digital_output_compressor_fast/commands', 'io_controllers_msgs/DigitalStateCommand',self.INPUT_TOPIC])
        # all_ros_topics.append(['/workcell_smp_irb2600/controller/digital_output_compressor_medium/commands', 'io_controllers_msgs/DigitalStateCommand',self.INPUT_TOPIC])
        # all_ros_topics.append(['/workcell_smp_irb2600/controller/digital_output_compressor_slow/commands', 'io_controllers_msgs/DigitalStateCommand',self.INPUT_TOPIC])
        # all_ros_topics.append(['/workcell_smp_irb2600/controller/digital_output_open_sewing_head/commands', 'io_controllers_msgs/DigitalStateCommand',self.INPUT_TOPIC])
        # all_ros_topics.append(['/workcell_smp_irb2600/controller/digital_output_vacuum_valve_zone_1/commands', 'io_controllers_msgs/DigitalStateCommand',self.INPUT_TOPIC])
        # all_ros_topics.append(['/workcell_smp_irb2600/controller/digital_output_vacuum_valve_zone_2/commands', 'io_controllers_msgs/DigitalStateCommand',self.INPUT_TOPIC])
        # all_ros_topics.append(['/workcell_smp_irb2600/controller/digital_output_vacuum_valve_zone_3/commands', 'io_controllers_msgs/DigitalStateCommand',self.INPUT_TOPIC])
        # all_ros_topics.append(['/workcell_smp_irb2600/controller/digital_output_vacuum_valve_zone_4/commands', 'io_controllers_msgs/DigitalStateCommand',self.INPUT_TOPIC])
        # all_ros_topics.append(['/workcell_smp_irb2600/controller/digital_output_vacuum_valve_zone_5/commands', 'io_controllers_msgs/DigitalStateCommand',self.INPUT_TOPIC])
        # all_ros_topics.append(['/workcell_smp_irb2600/controller/digital_output_vacuum_valve_zone_6/commands', 'io_controllers_msgs/DigitalStateCommand',self.INPUT_TOPIC])
        # all_ros_topics.append(['/workcell_smp_irb2600/controller/digital_output_vacuum_valve_zone_7/commands', 'io_controllers_msgs/DigitalStateCommand',self.INPUT_TOPIC])
        # all_ros_topics.append(['/workcell_smp_irb2600/controller/digital_output_vacuum_valve_zone_8/commands', 'io_controllers_msgs/DigitalStateCommand',self.INPUT_TOPIC])
        # all_ros_topics.append(['/workcell_smp_irb2600/controller/digital_output_vacuum_valve_zone_9/commands', 'io_controllers_msgs/DigitalStateCommand',self.INPUT_TOPIC])
        # all_ros_topics.append(['/workcell_smp_irb2600/controller/digital_output_vacuum_valve_zone_10/commands', 'io_controllers_msgs/DigitalStateCommand',self.INPUT_TOPIC])
        
        
        
        return all_ros_topics

    # cb

    def datachange_notification(self, node, val, data):
        nodeid_str = node.nodeid.to_string()
        method_str = self.method_map[nodeid_str]
        method = self.server.get_node(method_str)
        node.call_method(method)

    def find_service_node_with_same_name(self, name, idx):
        print("Reached ServiceCheck for name " + name)
        for service in self.servicesDict:
            print(
                "Found name: " + str(self.servicesDict[service].parent.nodeid.Identifier))
            if self.servicesDict[service].parent.nodeid.Identifier == name:
                print("Found match for name: " + name)
                return self.servicesDict[service].parent
        return None

    def find_topics_node_with_same_name(self, name, idx):
        
        print("Reached TopicCheck for name " + name)
        for topic in self.topicsDict:
            print(
                "Found name: " + str(self.topicsDict[topic].parent.nodeid.Identifier))
            if self.topicsDict[topic].parent.nodeid.Identifier == name:
                print("Found match for name: " + name)
                return self.topicsDict[topic].parent
        return None

    def find_action_node_with_same_name(self, name, idx):
        print("Reached ActionCheck for name " + name)
        for topic in self.actionsDict:
            print(
                "Found name: " + str(self.actionsDict[topic].parent.nodeid.Identifier))
            if self.actionsDict[topic].parent.nodeid.Identifier == name:
                print("Found match for name: " + name)
                return self.actionsDict[topic].parent
        return None


def main(args):
    rosserver = ROSServer()


if __name__ == "__main__":
    main(sys.argv)

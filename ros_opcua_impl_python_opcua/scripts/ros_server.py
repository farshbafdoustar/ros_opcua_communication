#!/usr/bin/env python
import sys
import time
import ast

import rosgraph
import rosnode
import rospy
from opcua import Server, ua

import ros_services
import ros_topics

from io_controllers_msgs.msg import *
from control_msgs.msg import *
from trajectory_msgs import *
from std_msgs.msg import *

# Returns the hierachy as one string from the first remaining part on.
sub_lst = []

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


def own_rosnode_cleanup():
    pinged, unpinged = rosnode.rosnode_ping_all()
    if unpinged:
        master = rosgraph.Master(rosnode.ID)
        # noinspection PyTypeChecker
        rosnode.cleanup_master_blacklist(master, unpinged)


class ROSServer:
    def __init__(self):
        self.namespace_ros = rospy.get_param("/rosopcua/namespace")
        self.topicsDict = {}
        self.servicesDict = {}
        self.actionsDict = {}
        rospy.init_node("rosopcua")
        self.server = Server()
        # self.server.set_endpoint("opc.tcp://192.168.1.100:21554/")
        with open('/data/workcell_smp_irb2600/config/vetron_opcua_server.txt', 'r') as txt:
            txtfile = txt.read()
        self.server.set_endpoint(txtfile)
        self.server.set_server_name("ROS OPCUA Server")
        self.server.start()
        self.method_map = None
        # self.INPUT_TOPIC="INPUT"
        # self.OUTPUT_TOPIC="OUTPUT"
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

        # while not rospy.is_shutdown():

        all_topics_lst = self.get_all_topics()

        # ros_topics starts a lot of publisher/subscribers, might slow everything down quite a bit.
        ros_services.refresh_services(self.namespace_ros, self, self.servicesDict, idx_services, services_object)
        self.method_map = ros_topics.refresh_topics_and_actions(self.namespace_ros, self, self.topicsDict, self.actionsDict,
                                                                idx_topics, idx_actions, topics_object, actions_object, all_topics_lst)
        # hh
        for key in self.method_map:
            for topic_name, topic_type in all_topics_lst:
                if topic_name in key:
                    leaf_node = self.server.get_node(key)
                    sub_lst.append(leaf_node)                 
        try:
            sub = self.server.create_subscription(period=1, handler=self)
            handle = sub.subscribe_data_change(sub_lst)

        except:
            print("Can not create_subscription for this object:", key)

        print("ROS OPCUA Server initialized.")
        while not rospy.is_shutdown():
            rospy.spin()
       
        #time.sleep(0)  # used to be 60

        self.server.stop()
        print("ROS OPCUA Server stopped.")
        quit()
 
    def get_all_topics(self):
        # function to provide topics
        ros_topics = rospy.get_param("/rosopcua/topics_glob")
        all_ros_topics = ast.literal_eval(ros_topics)

        return all_ros_topics

    # cb

    def datachange_notification(self, node, val, data):
        str = node.nodeid.to_string()
        method_str = self.method_map[str]
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

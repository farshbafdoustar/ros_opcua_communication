#!/usr/bin/env python

# Thanks to:
# https://github.com/ros-visualization/rqt_common_plugins/blob/groovy-devel/rqt_topic/src/rqt_topic/topic_widget.py
import numpy
import random

import roslib
import roslib.message
import rospy
from opcua import ua, uamethod

import ros_actions
import ros_server
import rostopic

from io_controllers_msgs.msg import DigitalStateCommand
from geometry_msgs.msg import TransformStamped
from io_controllers_msgs.msg import *
from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import *
from sensor_msgs.msg import *
from trajectory_msgs.msg import *
from control_msgs.msg import *


class OpcUaROSTopic:
    def __init__(self, server, parent, idx, topic_name, topic_type):
        # INPUT_TOPIC="INPUT"
        # OUTPUT_TOPIC="OUTPUT"
        self.server = server
        self.parent = self.recursive_create_objects(topic_name, idx, parent)
        self.type_name = topic_type
        self.name = topic_name
        self._nodes = {}
        self.idx = idx
        self.message_class = None

        
        # hh
        self.update_method_str = None
        self.child_to_update_method_map = {}



        try:
            self.message_class = roslib.message.get_message_class(topic_type)
            self.message_instance = self.message_class()
        except rospy.ROSException:
            self.message_class = None
            rospy.logfatal("Couldn't find message class for type " + topic_type)
        

        self._recursive_create_items(self.parent, idx, topic_name, topic_type, self.message_instance, True)
        self._subscriber = rospy.Subscriber(self.name, self.message_class, self.message_callback)
        self._publisher = rospy.Publisher(self.name, self.message_class, latch = True, queue_size=1)
        rospy.loginfo("Created ROS Topic: " + str(self.name))
       
        
        #self.opcua_update_callback(self.parent)

        # return self.child_to_update_method_map



    def _recursive_create_items(self, parent, idx, topic_name, type_name, message, top_level=False):
        topic_text = topic_name.split('/')[-1]
        if '[' in topic_text:
            topic_text = topic_text[topic_text.index('['):]
        # This here are 'complex datatypes'
        if hasattr(message, '__slots__') and hasattr(message, '_slot_types'):
            complex_type = True
            new_node = parent.add_object(ua.NodeId(topic_name, parent.nodeid.NamespaceIndex, ua.NodeIdType.String),
                                         ua.QualifiedName(topic_name, parent.nodeid.NamespaceIndex))
            new_node.add_property(ua.NodeId(topic_name + ".Type", idx),
                                  ua.QualifiedName("Type", parent.nodeid.NamespaceIndex), type_name)
            if top_level:
                new_node.add_method(ua.NodeId(topic_name + ".Update", parent.nodeid.NamespaceIndex),
                                    ua.QualifiedName("Update", parent.nodeid.NamespaceIndex),
                                    self.opcua_update_callback, [], [])
                #hh
                self.update_method_str = ua.NodeId(topic_name + ".Update", parent.nodeid.NamespaceIndex).to_string()  
            for slot_name, type_name_child in zip(message.__slots__, message._slot_types): 
                self._recursive_create_items(new_node, idx, topic_name + '/' + slot_name, type_name_child,
                                             getattr(message, slot_name))
            self._nodes[topic_name] = new_node

        else:
            
            base_type_str, array_size = _extract_array_info(type_name)
            try:
                base_instance = roslib.message.get_message_class(base_type_str)()
            except (ValueError, TypeError):
                base_instance = None

            if array_size is not None and hasattr(base_instance, '__slots__'):
                for index in range(array_size):
                    self._recursive_create_items(parent, idx, topic_name + '[%d]' % index, base_type_str, base_instance)
            
            elif array_size is not None:
                array_size = 1
                new_node = _create_nodearray_with_type(parent, idx, topic_name, topic_text, type_name, array_size)
                self._nodes[topic_name] = new_node
                child_node_str = new_node.nodeid.to_string()
                self.child_to_update_method_map[child_node_str] = self.update_method_str                

            else:
                new_node = _create_node_with_type(parent, idx, topic_name, topic_text, type_name, array_size)
                self._nodes[topic_name] = new_node

                # hh
                child_node_str = new_node.nodeid.to_string()
                self.child_to_update_method_map[child_node_str] = self.update_method_str

        if topic_name in self._nodes and self._nodes[topic_name].get_node_class() == ua.NodeClass.Variable:
            self._nodes[topic_name].set_writable(True)

        return

    def message_callback(self, message):
        self.update_value(self.name, message)


    @uamethod
    def opcua_update_callback(self, parent):
        # print("!!!OPC-UA Object Change Callback :: ROS Publisher is Called!!!")
        if self._publisher is not None:
            try:
                for nodeName in self._nodes:
                    child = self._nodes[nodeName]
                    name = child.get_display_name().Text
                    if hasattr(self.message_instance, name):
                        if child.get_node_class() == ua.NodeClass.Variable:
                            setattr(self.message_instance, name, correct_type(child, type(getattr(self.message_instance, name))))
                        if child.get_node_class == ua.NodeClass.Object:
                            setattr(self.message_instance, name, self.create_message_instance(child))
                print("OPC-UA object change callback:", self.message_instance)
                self._publisher.publish(self.message_instance)

            except:
                rospy.logerr("Error when updating node" + self.name, e)
                self.server.server.delete_nodes([self.parent])
        else:
            print('OPC-UA object changed but no publisher available for:', self.name, self.message_instance)
               
        # print("OPC-UA object change callback finished.")


# TODO: Update_Value cant extract topic info of the following topic: 
# all_ros_topics.append(['/teststation/controller/position_trajectory_controller/command', 'trajectory_msgs/JointTrajectory',self.INPUT_TOPIC])

    def update_value(self, topic_name, message):#it does enter this func
        if hasattr(message, '__slots__') and hasattr(message, '_slot_types'):
            for slot_name in message.__slots__:
                self.update_value(topic_name + '/' + slot_name, getattr(message, slot_name))
        
        elif type(message) in (list, tuple):
            if (len(message) > 0) and hasattr(message[0], '__slots__'):
                for index, slot in enumerate(message):
                    if topic_name + '[%d]' % index in self._nodes:
                        self.update_value(topic_name + '[%d]' % index, slot)
                    else:
                        if topic_name in self._nodes:
                            base_type_str, _ = _extract_array_info(
                                self._nodes[topic_name].text(self.type_name))
                            self._recursive_create_items(self._nodes[topic_name], topic_name + '[%d]' % index,
                                                         base_type_str,
                                                         slot, None)
            if (len(message) > 0):
                for index, slot in enumerate(message):
                    if topic_name + '[%d]' % index in self._nodes:
                        self.update_value(topic_name + '[%d]' % index, slot)
                    else:
                        if topic_name in self._nodes:
                            base_type_str = _extract_array_info_python(type(slot))
                            self._recursive_create_items(self._nodes[topic_name], self.idx, topic_name + '[%d]' % index,
                                                         base_type_str,
                                                         slot, None)
            
            #remove obsolete children
            if topic_name in self._nodes:
                if len(message) < len(self._nodes[topic_name].get_children()):
                    for i in range(len(message), self._nodes[topic_name].childCount()):
                        item_topic_name = topic_name + '[%d]' % i
                        self.recursive_delete_items(self._nodes[item_topic_name])
                        del self._nodes[item_topic_name]
        else:
            if topic_name in self._nodes and self._nodes[topic_name] is not None:
                self._nodes[topic_name].set_value(message,self._nodes[topic_name].get_data_type_as_variant_type())

    def recursive_delete_items(self, item):
        self._publisher.unregister()
        self._subscriber.unregister()
        for child in item.get_children():
            self.recursive_delete_items(child)
            if child in self._nodes:
                del self._nodes[child]
            self.server.server.delete_nodes([child])
        self.server.server.delete_nodes([item])
        if len(self.parent.get_children()) == 0:
            self.server.server.delete_nodes([self.parent])

    def create_message_instance(self, node):
        for child in node.get_children():
            name = child.get_display_name().Text
            if hasattr(self.message_instance, name):
                if child.get_node_class() == ua.NodeClass.Variable:
                    setattr(self.message_instance, name,
                            correct_type(child, type(getattr(self.message_instance, name))))
                elif child.get_node_class == ua.NodeClass.Object:
                    setattr(self.message_instance, name, self.create_message_instance(child))
        return self.message_instance  # Converts the value of the node to that specified in the ros message we are trying to fill. Casts python ints

    def recursive_create_objects(self, topic_name, idx, parent):
        hierachy = topic_name.split('/')
        if len(hierachy) == 0 or len(hierachy) == 1:
            return parent
        for name in hierachy:
            if name != '':
                try:
                    nodewithsamename = self.server.find_topics_node_with_same_name(name, idx)
                    if nodewithsamename is not None:
                        return self.recursive_create_objects(ros_server.nextname(hierachy, hierachy.index(name)), idx,
                                                             nodewithsamename)
                    else:
                        # if for some reason 2 services with exactly same name are created use hack>: add random int, prob to hit two
                        # same ints 1/10000, should be sufficient
                        newparent = parent.add_object(
                            ua.NodeId(name, parent.nodeid.NamespaceIndex, ua.NodeIdType.String),
                            ua.QualifiedName(name, parent.nodeid.NamespaceIndex))
                        return self.recursive_create_objects(ros_server.nextname(hierachy, hierachy.index(name)), idx,
                                                             newparent)
                # thrown when node with parent name is not existent in server
                except (IndexError, common.UaError) as e:
                    newparent = parent.add_object(
                        ua.NodeId(name + str(random.randint(0, 10000)), parent.nodeid.NamespaceIndex,
                                  ua.NodeIdType.String),
                        ua.QualifiedName(name, parent.nodeid.NamespaceIndex))
                    return self.recursive_create_objects(ros_server.nextname(hierachy, hierachy.index(name)), idx,
                                                         newparent)

        return parent

#End of Class

#hh
def merge_two_dicts(x, y):
    z = x.copy()   # start with keys and values of x
    z.update(y)    # modifies z with keys and values of y
    return z


# to unsigned integers as to fulfill ros specification. Currently only uses a few different types,
# no other types encountered so far.
def correct_type(node, typemessage):
    data_value = node.get_data_value()
    result = node.get_value()
    if isinstance(data_value, ua.DataValue):
        if typemessage.__name__ == "float":
            result = numpy.float(result)
        if typemessage.__name__ == "double":
            result = numpy.double(result)
        if typemessage.__name__ == "byte":
            result = numpy.byte(result)
    else:
        rospy.logerr("Can't Convert: " + str(node.get_data_value.Value))
        return None
    
    # newnode = node.nodeid.to_string()
    # resultstr = str(result)
    # if newnode.find('name') != -1:
    #     if resultstr.find(',') != -1:
    #         result = result.split(',')
    #     else:
    #         result = [result]
    # if newnode.find('value') != -1:
    #     if resultstr.find(',') != -1:
    #         result = result
    #     else:
    #         result = [result]
    return result

def _extract_array_info_python(type_python):
    type_str = None
    if type_python == str:
        type_str = "string[]"
    elif type_python == float:
        type_str = "float[]"
    elif type_python == bytearray:
        type_str = "byte[]"
    elif type_python == int:
        type_str = "int[]"
    elif type_python == bool:
        type_str = "bool[]"
    else:
        print("NOT FOUND TYPE:", type_python)

    return type_str

def _extract_array_info(type_str):
    array_size = None
    if '[' in type_str and type_str[-1] == ']':
        type_str, array_size_str = type_str.split('[', 1)
        array_size_str = array_size_str[:-1]
        if len(array_size_str) > 0:
            array_size = int(array_size_str)
        else:
            array_size = 0
    return type_str, array_size


def _create_node_with_type(parent, idx, topic_name, topic_text, type_name, array_size):

    if '[' in type_name:
        type_name = type_name[:type_name.index('[')]
        is_array = True
    if type_name == 'bool':
        dv = ua.Variant(False, ua.VariantType.Boolean)
    elif type_name == 'byte':
        dv = ua.Variant(0, ua.VariantType.Byte)
    elif type_name == 'int':
        dv = ua.Variant(0, ua.VariantType.Int32)
    elif type_name == 'int8':
        dv = ua.Variant(0, ua.VariantType.SByte)
    elif type_name == 'uint8':
        dv = ua.Variant(0, ua.VariantType.Byte)
    elif type_name == 'int16':
        dv = ua.Variant(0, ua.VariantType.Int16)
    elif type_name == 'uint16':
        dv = ua.Variant(0, ua.VariantType.UInt16)
    elif type_name == 'int32':
        dv = ua.Variant(0, ua.VariantType.Int32)
    elif type_name == 'uint32':
        dv = ua.Variant(0, ua.VariantType.UInt32)
    elif type_name == 'int64':
        dv = ua.Variant(0, ua.VariantType.Int64)
    elif type_name == 'uint64':
        dv = ua.Variant(0, ua.VariantType.UInt64)
    elif type_name == 'float' or type_name == 'float32' or type_name == 'float64':
        dv = ua.Variant(0.0, ua.VariantType.Float)
    elif type_name == 'double':
        dv = ua.Variant(0.0, ua.VariantType.Double)
    elif type_name == 'string':
        dv = ua.Variant('', ua.VariantType.String)

    else:
        rospy.logerr("Can't create node with type" + str(type_name))
        return None
    if array_size is not None:
        value = []
        for i in range(array_size):
            value.append(i)
    return parent.add_variable(ua.NodeId(topic_name, parent.nodeid.NamespaceIndex),
                               ua.QualifiedName(topic_text, parent.nodeid.NamespaceIndex), dv.Value, dv.VariantType)

def _create_nodearray_with_type(parent, idx, topic_name, topic_text, type_name, array_size):
    if '[' in type_name:
        type_name = type_name[:type_name.index('[')]
        is_array = True
    if type_name == 'bool':
        dv = ua.Variant([False], ua.VariantType.Boolean)
    elif type_name == 'byte':
        dv = ua.Variant([0], ua.VariantType.Byte)
    elif type_name == 'int':
        dv = ua.Variant([0], ua.VariantType.Int32)
    elif type_name == 'int8':
        dv = ua.Variant([0], ua.VariantType.SByte)
    elif type_name == 'uint8':
        dv = ua.Variant([0], ua.VariantType.Byte)
    elif type_name == 'int16':
        dv = ua.Variant([0], ua.VariantType.Int16)
    elif type_name == 'uint16':
        dv = ua.Variant([0], ua.VariantType.UInt16)
    elif type_name == 'int32':
        dv = ua.Variant([0], ua.VariantType.Int32)
    elif type_name == 'uint32':
        dv = ua.Variant([0], ua.VariantType.UInt32)
    elif type_name == 'int64':
        dv = ua.Variant([0], ua.VariantType.Int64)
    elif type_name == 'uint64':
        dv = ua.Variant([0], ua.VariantType.UInt64)
    elif type_name == 'float' or type_name == 'float32' or type_name == 'float64':
        dv = ua.Variant([0.0], ua.VariantType.Float)
    elif type_name == 'double':
        dv = ua.Variant([0.0], ua.VariantType.Double)
    elif type_name == 'string':
        dv = ua.Variant([''], ua.VariantType.String)

    else:
        rospy.logerr("Can't create node with type" + str(type_name))
        return None
    if array_size is not None:
        value = []
        for i in range(array_size):
            value.append(i)
    return parent.add_variable(ua.NodeId(topic_name, parent.nodeid.NamespaceIndex),
                               ua.QualifiedName(topic_text, parent.nodeid.NamespaceIndex), dv.Value, dv.VariantType)


# Used to delete obsolete topics
def numberofsubscribers(nametolookfor, topicsDict):
    # rosout only has one subscriber/publisher at all times, so ignore.
    # if nametolookfor != "/rosout":
    #    ret = topicsDict[nametolookfor]._subscriber.get_subscription_count()
    # else:
    ret = 2
    return ret


def refresh_topics_and_actions(namespace_ros, server, topicsdict, actionsdict, idx_topics, idx_actions, topics, actions, all_topics_lst):
    
    #hh
    all_child_to_method_maps = {}

    ros_topics = rospy.get_published_topics(namespace_ros)

    ros_topics = all_topics_lst

    rospy.logdebug(str(ros_topics))
    rospy.logdebug(str(rospy.get_published_topics('/move_base_simple')))
    for topic_name, topic_type in ros_topics:
        if topic_name not in topicsdict or topicsdict[topic_name] is None:
            splits = topic_name.split('/')
            if "cancel" in splits[-1] and "result" in splits[-1] and "feedback" in splits[-1] and "goal" in splits[-1] and "status" in splits[-1]:
                rospy.logdebug("Found an action: " + str(topic_name))
                correct_name = ros_actions.get_correct_name(topic_name)

                if correct_name not in actionsdict:
                    try:
                        rospy.loginfo("Creating Action with name: " + correct_name)
                        actionsdict[correct_name] = ros_actions.OpcUaROSAction(server,
                                                                               actions,
                                                                               idx_actions,
                                                                               correct_name,
                                                                               get_goal_type(correct_name),
                                                                               get_feedback_type(correct_name))
                    except (ValueError, TypeError, AttributeError) as e:
                        print(e)
                        rospy.logerr("Error while creating Action Objects for Action " + topic_name)
                pass       
            else:
                # rospy.loginfo("Ignoring normal topics for debugging...")
                topic = OpcUaROSTopic(server, topics, idx_topics, topic_name, topic_type)
                topicsdict[topic_name] = topic
                
                all_child_to_method_maps = merge_two_dicts(all_child_to_method_maps, topic.child_to_update_method_map)

        elif numberofsubscribers(topic_name, topicsdict) <= 1 and "rosout" not in topic_name:
            topicsdict[topic_name].recursive_delete_items(server.server.get_node(ua.NodeId(topic_name, idx_topics)))
            del topicsdict[topic_name]
            ros_server.own_rosnode_cleanup()

    ros_topics = rospy.get_published_topics(namespace_ros)

    # use to not get dict changed during iteration errors
    tobedeleted = []
    for topic_nameOPC in topicsdict:
        found = False
        for topicROS, topic_type in ros_topics:
            if topic_nameOPC == topicROS:
                found = True
        if not found:
            print(topic_nameOPC, 'to be deleted not found')
            #HERE the idea is deleting the topics from opcua that has not publisher
            #but actually the code has problem with server?? @Hamoon
            # topicsdict[topic_nameOPC].recursive_delete_items(server.get_node(ua.NodeId(topic_nameOPC, idx_topics)))
            # tobedeleted.append(topic_nameOPC)
    for name in tobedeleted:
        del topicsdict[name]
    ros_actions.refresh_dict(namespace_ros, actionsdict, topicsdict, server, idx_actions)
    return all_child_to_method_maps


def get_feedback_type(action_name):
    try:
        type, name, fn = rostopic.get_topic_type(action_name + "/feedback")
        return type
    except rospy.ROSException as e:
        try:
            type, name, fn = rostopic.get_topic_type(action_name + "/Feedback", e)
            return type
        except rospy.ROSException as e2:
            rospy.logerr("Couldn't find feedback type for action " + action_name, e2)
            return None


def get_goal_type(action_name):
    try:
        type, name, fn = rostopic.get_topic_type(action_name + "/goal")
        return type
    except rospy.ROSException as e:
        try:
            type, name, fn = rostopic.get_topic_type(action_name + "/Goal", e)
            return type
        except rospy.ROSException as e2:
            rospy.logerr("Couldn't find goal type for action " + action_name, e2)
            return None

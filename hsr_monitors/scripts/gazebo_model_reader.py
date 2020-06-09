#! /usr/bin/env python

from gazebo_msgs.srv import GetModelState, GetWorldProperties
import rospy

class Block:
    def __init__(self, name, relative_entity_name):
        self._name = name
        self._relative_entity_name = relative_entity_name

class Tutorial:

    _blockListDict = {
        'block_a': Block('mobile_base', 'wheel_left_link'),
        'block_b': Block('brick_box_3x1x3', 'chassis'),

    }

    def show_gazebo_models(self):
        try:
            model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            for block in self._blockListDict.itervalues():
                blockName = str(block._name)
                resp_coordinates = model_coordinates(blockName, block._relative_entity_name)
                print '\n'
                print 'Status.success = ', resp_coordinates.success
                print(blockName)
                print("Cube " + str(block._name))
                print("Valeur de X : " + str(resp_coordinates.pose.position.x))
                print("Quaternion X : " + str(resp_coordinates.pose.orientation.x))

        except rospy.ServiceException as e:
            rospy.loginfo("Get Model State service call failed:  {0}".format(e))

    def show_gazebo_model_list(self):
        get_world_properties = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
        try:
            resp = get_world_properties()
            for model in resp.model_names:
                print(model)
                
        except rospy.ServiceException as e:
            rospy.loginfo("Get Model State service call failed:  {0}".format(e))

if __name__ == '__main__':
    tuto = Tutorial()
    #tuto.show_gazebo_models()
    tuto.show_gazebo_model_list()

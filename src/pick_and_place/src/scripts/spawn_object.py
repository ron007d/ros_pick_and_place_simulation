'''
Create by - Bishwajit Kumar Poddar
'''
import rospy
from gazebo_msgs.srv import SpawnModel, SpawnModelRequest, SpawnModelResponse
from gazebo_msgs.srv import ApplyBodyWrench, ApplyBodyWrenchRequest, ApplyBodyWrenchResponse
from gazebo_msgs.srv import SetModelState, SetModelStateRequest, SetModelStateResponse
from gazebo_msgs.srv import DeleteModel, DeleteModelRequest, DeleteLightResponse
import time

class Block:
    
    '''
    This class will create different types of object with different dimentations.
    more types will be added
    - cylibder
    - box
    etc
    
    Currently only BOX is here
    '''
    
    def __init__(self, 
                 box_x: float = 0.07,
                 box_y: float = 0.07, 
                 box_z: float = 0.07,
                 object_name : str = 'simple_box'):
        
        self.box_x = box_x
        self.box_y = box_y
        self.box_z = box_z
        self.object_name = object_name
        self.create_xml()
    
    def create_xml(self):
        self.model_xml = '''<?xml version="1.0"?>
            <robot name="red_box">

            <link name="base_link">
            <collision>
                <origin xyz="0 0 0" rpy="0 0 0"/>
                <geometry>
                <box size="{box_x} {box_y} {box_z}" />
                </geometry>
            </collision>

            <visual>
                <origin xyz="0 0 0" rpy="0 0 0"/>
                <geometry>
                <box size="{box_x} {box_y} {box_z}" />
                </geometry>
            </visual>

            <inertial>
                <origin xyz="0 0 0" rpy="0 0 0" />
                <mass value="0.05" />
                <inertia
                ixx="0.001" ixy="0.0"  ixz="0.0"
                iyy="0.001" iyz="0.0"
                izz="0.001" />
            </inertial>
            </link>

            <gazebo reference="base_link">
            <material>Gazebo/Red</material>
                <mu1>5</mu1>
                <mu2>5</mu2>
            </gazebo>

            </robot>
        '''.format(box_x = self.box_x,
                   box_y = self.box_y,
                   box_z = self.box_z)
        
class Spawnner:
    
    def __init__(self,
                 object : Block):
        '''
        locations data have locations to spawn objects.
        spawned object store the uniqie id of spawnned object,
        so that same unique name is not generated
        
        locaion will be like this
        
        [[<x>, <y>, <z>], ...]
        Currently the orientation is made is fixed.
        '''
        
        
        
        
        self.spawnned_objects = []
        
        self.object = object
        
        self.__spawn_client = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        self.__spawn_model_req = SpawnModelRequest()
        self.__spawn_model_resp = SpawnModelResponse()

        self.__set_state_client = rospy.ServiceProxy('/gazebo/set_model_state',SetModelState)
        self.__set_state_req = SetModelStateRequest()
        self.__set_state_resp = SetModelStateResponse()
        self.object_count = 0
    
    
    def spawn_in_locations(self,location :list):
        
        self.__spawn_model_req.initial_pose.orientation.x=0.0
        self.__spawn_model_req.initial_pose.orientation.y=0.0
        self.__spawn_model_req.initial_pose.orientation.z=0.0
        self.__spawn_model_req.initial_pose.orientation.w=1.0
        self.__spawn_model_req.reference_frame = "world"

                
        self.__spawn_model_req.initial_pose.position.x = location[0]
        self.__spawn_model_req.initial_pose.position.y = location[1]
        self.__spawn_model_req.initial_pose.position.z = location[2]
        
        rospy.loginfo(f'Y position of the box {self.__spawn_model_req.initial_pose.position}')
        
        
        model_name = self.object.object_name + f'_loc_{self.object_count}'
        self.object_count += 1
        self.spawnned_objects.append(model_name)
        self.__spawn_model_req.model_name = model_name
        self.__spawn_model_req.robot_namespace = model_name
        self.__spawn_model_req.model_xml = self.object.model_xml
        try:
            call_service = self.__spawn_client(self.__spawn_model_req.model_name,
                                                self.__spawn_model_req.model_xml, 
                                                self.__spawn_model_req.robot_namespace, 
                                                self.__spawn_model_req.initial_pose, 
                                                self.__spawn_model_req.reference_frame)
            print(call_service)
        except:
            print('error happening in spawn call')
            pass
            
    
    def delete_spawnned_model(self):
        delete_client = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        for i in self.spawnned_objects:
            
            call_service = delete_client.call(model_name= i)
            if call_service:
                rospy.loginfo(f'Deleted models {i}')
                
        self.spawnned_objects = []
            
            

if __name__ == '__main__':
    '''
    How the test will perform
    
    designing of the code
    - 1. Create the object [ basically the model is referred here ]
    - 2. Pass the object to the Spawnner
    - 3. Pass the locations [ Testing done from the URDF now ]
    
    - Frame reference
    world { Can be changed }
    '''
    rospy.init_node('block_spawnner_task2')
    
    box = Block()
    spawnning_locations = [ 
                 [-0.1,0.7,1],
                 [-0.4,0.9,1],
                 [0.4,0.9,1],
                 [0.0,0.6,1],
                 [-0.4,0.6,1],
                 [0.4,0.6,1],
                 [-0.4,0.15,1]
                 ]
    spawning = Spawnner(object= box)
    # Spawn the objects in locations
    for i in spawnning_locations:
        spawning.spawn_in_locations(i)
    
        time.sleep(5)
    
        # Delete Spawnned objects
        spawning.delete_spawnned_model()
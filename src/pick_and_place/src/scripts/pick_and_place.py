#!/usr/bin/env python
'''
Create by - Bishwajit Kumar Poddar
'''
import sys
import os
sys.path.append(os.path.dirname(__file__))
import rospy
from std_msgs.msg import String
from robot_mover import MoveRobot
import json
import time
from spawn_object import Block, Spawnner

class PickPlace:
    def __init__(self) -> None:
        rospy.init_node('picking_and_placing',anonymous=True)
        self.robot = MoveRobot()
        
        box = Block(box_x=0.05,
            box_y= 0.05,
            box_z=0.05)
        self.spawning = Spawnner(object=box)
    
    
    def pick(self,angle,x,y):  
        self.robot.take_home()
        time.sleep(2)
        self.robot.pick_at_position(angle,x,y)
    
    def place_home(self):
        self.robot.place_at_home()
    
    def spawn_object(self,spawning_location: list ):
        self.spawning.spawn_in_locations(spawning_location)
        
    def checking_message(self):
        position = rospy.wait_for_message('/object_location',String,timeout=None) 
        print('------------- {text} ----------------'.format(text = "FOUND OBJECT"))
        print('------------- {text} ----------------'.format(text = "LET THE OBJECT TAKE SOME TIME"))
        time.sleep(2)
        position = rospy.wait_for_message('/object_location',String,timeout=None) 
        position = json.loads(position.data)
        angle = position['angel']
        x = position['position']['x']
        y = position['position']['y']
        return angle, x, y
    
    def example(self,locations : list):
        for i in locations:
            self.spawn_object(i)
            angle, x, y = self.checking_message()
            self.pick(angle,x,y)
            self.place_home()
            time.sleep(1)
        time.sleep(4)
        self.spawning.delete_spawnned_model()
        
        
        
        
if __name__ == '__main__':
    x = PickPlace()
    locations_of_objects = [
        [-0.2 , 0.7 , 1],
        [-0.4 , 0.6 , 1],
        [ 0.1 , 0.7 , 1]
    ]
    x.example(locations_of_objects)
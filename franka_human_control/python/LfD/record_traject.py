#!/usr/bin/env python
import time
import numpy as np

### UNCOMMENT THIS STUFF EVENTUALLY!!
from Learning_from_demonstration import LfD
from geometry_msgs.msg import PoseStamped
import rospy
# import actionlib
# import roslib


"""
This scripts records trajectories and saves them in a specific folder
"""

class Pipeline:
    def __init__(self):
        self.tasks = np.array(['PUSH_BLUE_BUTTON','GRAB_KEY','GRAB_ETHER','OPEN_BATTERY_BOX','EJECT_BATTERIES',
                            'PICK_BATTERY_1','PICK_BATTERY_2','PUSH_BATTERIES','PUSH_RED_BUTTON'])

        self.methods = np.array(['RECORD', 'PLAYBACK'])

        self.start_locations = np.array(['CURRENT', 'PREVIOUS_TASK'])

        self.method = None
        self.task = None
        self.start = None

        self.task_int = -1
        self.method_int = -1
        self.load_int = -1
        self.start_loc_int = -1
        self.save = -1

        self.LfD = LfD()


    def select_task(self):
        while not (self.task_int in [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]):
            print('\n')
            print('PLEASE SELECT AN ACTION:')
            print('0. STOP_PROGRAM')
            print('1. PUSH_BLUE_BUTTON')  
            print('2. GRAB_KEY')
            print('3. GRAB_ETHER')
            print('4. OPEN_BATTERY_BOX')
            print('5. EJECT_BATTERIES')
            print('6. PICK_BATTERY_1')
            print('7. PICK_BATTERY_2')
            print('8. PUSH_BATTERIES')
            print('9. PUSH_RED_BUTTON')

            try:
                self.task_int = int(input('\n'))
            except:
                print("INVALID INPUT")

        if self.task_int == 0:
            print('GOOD BYE!')
            exit()
        else:
            self.task = self.tasks[self.task_int-1]
            print(self.task)

    def select_method(self):
        while not (self.method_int in [0, 1, 2]):
            print('\n')
            print('PLEASE SELECT AN ACTION:')
            print('0. STOP_PROGRAM')
            print('1. RECORD')
            print('2. PLAYBACK')

            try:
                self.method_int = int(input('\n'))
            except:
                print("INVALID INPUT")

        if self.method_int == 0:
            print('GOOD BYE!')
            exit()
        else:
            self.method = self.methods[self.method_int-1]
            print(self.method)

    def select_start(self):
        while not (self.start_loc_int in [0, 1, 2, 3]):
            print('\n')
            print('PLEASE SELECT START LOCATION:')
            print('0. STOP_PROGRAM')
            print('1. CURRENT')
            print('2. PREVIOUS_TASK')

            try:
                self.start_loc_int = int(input('\n'))
            except:
                print("INVALID INPUT")

        if self.start_loc_int == 0:
            print('GOOD BYE!')
            exit()
        else:
            self.start = self.start_locations[self.start_loc_int-1]
            print(self.start)

    def select_choices(self):
         # 1. Selection action
        self.select_task()

        # 2. Select Recording/Playback
        self.select_method()

        # 3. Select use start location
        if self.task_int==1:
            self.start = 'CURRENT'
            print(self.start)
        elif self.method_int==1:
            self.select_start()
        
    def save_trajectory(self):
        while not (self.save in [0,1]):
            print("SAVE CURRENT RECORDING? 0 = NO, 1 = YES")
            try:
                self.save = int(input('\n'))
            except:
                print("INVALID INPUT")

        if self.save:
            self.LfD.save(self.task)
            print('task saved!')
        else:
            print('task not saved!')

    def run_choices(self):
        #############################
        # ADD ALL THE FUNCTIONS BELOW
        #############################
        # 
        if self.start == 'PREVIOUS_TASK':
            #### LOAD PREVIOUS TASK LfD.recorded_traj
            start = PoseStamped()
            self.LfD.load(self.tasks[self.task_int-2])
            
            start.pose.position.x = self.LfD.recorded_traj[0][-1]
            start.pose.position.y = self.LfD.recorded_traj[1][-1]
            start.pose.position.z = self.LfD.recorded_traj[2][-1]
            start.pose.orientation.w = self.LfD.recorded_ori[0][-1]
            start.pose.orientation.x = self.LfD.recorded_ori[1][-1]
            start.pose.orientation.y = self.LfD.recorded_ori[2][-1]
            start.pose.orientation.z = self.LfD.recorded_ori[3][-1]

            print("Going to end of ", self.tasks[self.task_int-2])
            self.LfD.go_to_pose(start)
            

        if self.method == 'RECORD':
            self.LfD.traj_rec()
            rospy.sleep(1)

            start = PoseStamped()
    
            start.pose.position.x = self.LfD.recorded_traj[0][0]
            start.pose.position.y = self.LfD.recorded_traj[1][0]
            start.pose.position.z = self.LfD.recorded_traj[2][0]

            start.pose.orientation.w = self.LfD.recorded_ori[0][0]
            start.pose.orientation.x = self.LfD.recorded_ori[1][0]
            start.pose.orientation.y = self.LfD.recorded_ori[2][0]
            start.pose.orientation.z = self.LfD.recorded_ori[3][0]
            self.LfD.go_to_pose(start)
            
        elif self.method == 'PLAYBACK':
            self.LfD.load(self.task)

            self.LfD.execute()

        ###################
        # ALWAYS ASK TO SAVE THE TRAJECTORY!!
        ###################

        self.save_trajectory()





    
if __name__=="__main__":
    rospy.init_node("record_trajectory")

    print("recording trajectory node started")
    pipeline = Pipeline()
    time.sleep(1)

    pipeline.select_choices()

    pipeline.run_choices()
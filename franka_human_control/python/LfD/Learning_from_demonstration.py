#%%
#!/usr/bin/env python
import rospy
import sys
import math
import numpy as np
import quaternion # pip install numpy-quaternion
import time
import pathlib
from datetime import datetime
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray, Float32
import dynamic_reconfigure.client
from pynput.keyboard import Listener, KeyCode
import tf2_ros
from pyquaternion import Quaternion
#from pyquaternion import Quaternion

class LfD():
    def __init__(self):
        self.r=rospy.Rate(100)
        self.K_pos=1000
        self.K_ori=30
        self.K_ns=10
        self.feedback=np.zeros(3)
        self.feedback_gain=0.002
        self.length_scale = 0.1
        self.correction_window = 300
        self.curr_pos=None
        self.curr_ori=None
        self.width=None
        self.pick = 0
        self.place = 0
        self.recorded_traj = None 
        self.recorded_ori=None
        self.recorded_gripper= None
        self.end=False
        self.grip_value=1
        self.pos_sub=rospy.Subscriber("/cartesian_pose", PoseStamped, self.ee_pos_callback)
        self.gripper_sub=rospy.Subscriber("/joint_states", JointState, self.gripper_callback)
        self.goal_pub = rospy.Publisher('/equilibrium_pose', PoseStamped, queue_size=0)
        self.goal_sub = rospy.Subscriber('/goal_pose', PoseStamped, self.goal_checker)
        self.grip_pub = rospy.Publisher('/gripper_online', Float32, queue_size=0)
        self.set_K = dynamic_reconfigure.client.Client('/dynamic_reconfigure_compliance_param_node')
        # self.stiff_pub = rospy.Publisher('/stiffness', Float32MultiArray, queue_size=0) 
        self.listener = Listener(on_press=self._on_press)
        self.listener.start()

    def _on_press(self, key):
        # This function runs on the background and checks if a keyboard key was pressed
        if key == KeyCode.from_char('e'):
            self.end = True
        # Feedback for translate forward/backward
        if key == KeyCode.from_char('w'):
            self.feedback[0] = self.feedback_gain
        if key == KeyCode.from_char('s'):
            self.feedback[0] = -self.feedback_gain
        # Feedback for translate left/right
        if key == KeyCode.from_char('a'):
            self.feedback[1] = self.feedback_gain
        if key == KeyCode.from_char('d'):
            self.feedback[1] = -self.feedback_gain
        # Feedback for translate up/down
        if key == KeyCode.from_char('u'):
            self.feedback[2] = self.feedback_gain
        if key == KeyCode.from_char('j'):
            self.feedback[2] = -self.feedback_gain
        # Close/open gripper
        if key == KeyCode.from_char('c'):
            self.grip_value = 0
            grip_command = Float32()
            grip_command.data = self.grip_value
            self.grip_pub.publish(grip_command)
            print('pressed c grip_value is ', grip_command.data)
        if key == KeyCode.from_char('o'):
            self.grip_value = 1
            grip_command = Float32()
            grip_command.data = self.grip_value
            self.grip_pub.publish(grip_command)
            print('pressed o grip_value is ', grip_command.data)

    def ee_pos_callback(self, data):
        self.curr_pos = np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z])
        self.curr_ori = np.array([data.pose.orientation.w, data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z])
        #rospy.loginfo([data.x, data.y, data.z])

    def gripper_callback(self, data):
        self.width =data.position[7]+data.position[8]

    def set_stiffness(self, k_t1, k_t2, k_t3,k_r1,k_r2,k_r3, k_ns):
        
        #set_K = dynamic_reconfigure.client.Client('/dynamic_reconfigure_compliance_param_node', config_callback=None)
        self.set_K.update_configuration({"translational_stiffness_X": k_t1})
        self.set_K.update_configuration({"translational_stiffness_Y": k_t2})
        self.set_K.update_configuration({"translational_stiffness_Z": k_t3})        
        self.set_K.update_configuration({"rotational_stiffness_X": k_r1}) 
        self.set_K.update_configuration({"rotational_stiffness_Y": k_r2}) 
        self.set_K.update_configuration({"rotational_stiffness_Z": k_r3})
        self.set_K.update_configuration({"nullspace_stiffness": k_ns})
    
    # def set_stiffness(self, k_t1, k_t2, k_t3,k_r1,k_r2,k_r3, k_ns):
    #     stiff_des = Float32MultiArray()
    #     stiff_des.data = np.array([k_t1,k_t2, k_t3, k_r1, k_r2, k_r3, k_ns]).astype(np.float32)
    # #    print(stiff_des)
    #     self.stiff_pub.publish(stiff_des)   

    def traj_rec(self, trigger=0.005, rec_position=True, rec_orientation=True):
        # trigger for starting the recording
        if rec_position==True: 
            self.set_K = dynamic_reconfigure.client.Client('/dynamic_reconfigure_compliance_param_node', config_callback=None)
            self.set_K.update_configuration({"translational_stiffness_X": 0})
            self.set_K.update_configuration({"translational_stiffness_Y": 0})
            self.set_K.update_configuration({"translational_stiffness_Z": 0})  
        if rec_orientation==True: 
            self.set_K.update_configuration({"rotational_stiffness_X": 0})
            self.set_K.update_configuration({"rotational_stiffness_Y": 0})
            self.set_K.update_configuration({"rotational_stiffness_Z": 0})

        # self.set_stiffness(0.0, 0.0, 0.0 , 0.0, 0.0, 0.0, 0.0)

        init_pos = self.curr_pos
        print(init_pos)
        vel = 0
        print("Move robot to start recording.")
        while vel < trigger:
            vel = math.sqrt((self.curr_pos[0]-init_pos[0])**2 + (self.curr_pos[1]-init_pos[1])**2 + (self.curr_pos[2]-init_pos[2])**2)
        
        self.recorded_traj = self.curr_pos
        self.recorded_ori = self.curr_ori
        self.recorded_gripper= self.grip_value
        key_pressed = False
        print("Recording started. Press e to stop.")
        self.end=False
        while not self.end:
            now = time.time()            # get the time

            self.recorded_traj = np.c_[self.recorded_traj, self.curr_pos]
            self.recorded_ori  = np.c_[self.recorded_ori, self.curr_ori]
            self.recorded_gripper = np.c_[self.recorded_gripper, self.grip_value]

            self.r.sleep()

    # control robot to desired goal position
    def go_to_pose(self, goal_pose):
        # the goal pose should be of type PoseStamped. E.g. goal_pose=PoseStampled()
        start = self.curr_pos
        start_ori=self.curr_ori
        goal_=np.array([goal_pose.pose.position.x, goal_pose.pose.position.y, goal_pose.pose.position.z])
        # interpolate from start to goal with attractor distance of approx 1 cm
        print(start, goal_)
        squared_dist = np.sum(np.subtract(start, goal_)**2, axis=0)
        dist = np.sqrt(squared_dist)
        print("dist", dist)
        interp_dist = 0.001/3  # [m]
        step_num_lin = math.floor(dist / interp_dist)
        
        print("num of steps linear", step_num_lin)

        q_start=np.quaternion(start_ori[0], start_ori[1], start_ori[2], start_ori[3])
        print("q_start", q_start)
        q_goal=np.quaternion(goal_pose.pose.orientation.w, goal_pose.pose.orientation.x, goal_pose.pose.orientation.y, goal_pose.pose.orientation.z)
        inner_prod=q_start.x*q_goal.x+q_start.y*q_goal.y+q_start.z*q_goal.z+q_start.w*q_goal.w
        if inner_prod < 0:
            q_start.x=-q_start.x
            q_start.y=-q_start.y
            q_start.z=-q_start.z
            q_start.w=-q_start.w
        inner_prod=q_start.x*q_goal.x+q_start.y*q_goal.y+q_start.z*q_goal.z+q_start.w*q_goal.w
        theta= np.arccos(np.abs(inner_prod))
        print(theta)
        interp_dist_polar = 0.001/3
        step_num_polar = math.floor(theta / interp_dist_polar)

        
        print("num of steps polar", step_num_polar)
        
        step_num=np.max([step_num_polar,step_num_lin])
        
        print("num of steps", step_num)
        x = np.linspace(start[0], goal_pose.pose.position.x, step_num)
        y = np.linspace(start[1], goal_pose.pose.position.y, step_num)
        z = np.linspace(start[2], goal_pose.pose.position.z, step_num)
        
        goal = PoseStamped()
        
        goal.pose.position.x = x[0]
        goal.pose.position.y = y[0]
        goal.pose.position.z = z[0]
        
        
        quat=np.slerp_vectorized(q_start, q_goal, 0.0)
        goal.pose.orientation.x = quat.x
        goal.pose.orientation.y = quat.y
        goal.pose.orientation.z = quat.z
        goal.pose.orientation.w = quat.w

        self.goal_pub.publish(goal)

        self.set_stiffness(self.K_pos, self.K_pos, self.K_pos ,self.K_ori,self.K_ori,self.K_ori, 0.0)

        goal = PoseStamped()
        for i in range(step_num):
            now = time.time()         
            goal.header.seq = 1
            goal.header.stamp = rospy.Time.now()
            goal.header.frame_id = "map"

            goal.pose.position.x = x[i]
            goal.pose.position.y = y[i]
            goal.pose.position.z = z[i]
            quat=np.slerp_vectorized(q_start, q_goal, i/step_num)
            #print("quat", quat) 
            goal.pose.orientation.x = quat.x
            goal.pose.orientation.y = quat.y
            goal.pose.orientation.z = quat.z
            goal.pose.orientation.w = quat.w
            self.goal_pub.publish(goal)
            self.r.sleep()   

    def square_exp(self, ind_curr, ind_j):
        d = np.sqrt((self.recorded_traj[0][ind_curr]-self.recorded_traj[0][ind_j])**2+(self.recorded_traj[1][ind_curr]-self.recorded_traj[1][ind_j])**2+(self.recorded_traj[2][ind_curr]-self.recorded_traj[2][ind_j])**2)
        return np.exp(-d**2/self.length_scale**2)

    def execute(self):
        self.new_grip = np.copy(self.recorded_gripper)
        grip_command_old=0
        for i in range (self.recorded_traj.shape[1]):
            goal = PoseStamped()

            goal.header.seq = 1
            goal.header.stamp = rospy.Time.now()
            goal.header.frame_id = "map"

            goal.pose.position.x = self.recorded_traj[0][i] 
            goal.pose.position.y = self.recorded_traj[1][i]
            goal.pose.position.z = self.recorded_traj[2][i]

            goal.pose.orientation.w = self.recorded_ori[0][i] 
            goal.pose.orientation.x = self.recorded_ori[1][i] 
            goal.pose.orientation.y = self.recorded_ori[2][i] 
            goal.pose.orientation.z = self.recorded_ori[3][i] 

            # print(goal)

            if np.sum(self.feedback)!=0:
                print('feedback is:')
                print(self.feedback)
                for j in range(max(0,i-self.correction_window),min(i+self.correction_window, self.recorded_traj.shape[1])):
                    x = self.feedback[0]*self.square_exp(i, j)
                    y = self.feedback[1]*self.square_exp(i, j)
                    z = self.feedback[2]*self.square_exp(i, j)
                    print(x, y, z)

                    self.recorded_traj[0][j] += x
                    self.recorded_traj[1][j] += y
                    self.recorded_traj[2][j] += z

                print('first value')
                print(self.recorded_traj[:,i])
                print('new value')
                self.feedback = np.zeros(3)

            self.goal_pub.publish(goal)
            
            grip_command = Float32()

            grip_command.data = self.recorded_gripper[0][i]
            

            self.grip_pub.publish(grip_command)
            if (grip_command_old-grip_command.data)>0.5:
                time.sleep(0.1)

            grip_command_old = grip_command.data
            self.r.sleep()


    def save(self, data='last'):
        np.savez(str(pathlib.Path().resolve()) + '/data/' + str(data) + '.npz',
                 traj=self.recorded_traj,
                 ori=self.recorded_ori,
                 grip=self.new_grip)

    def load(self, file='last'):
        data = np.load(str(pathlib.Path().resolve()) + '/data/' + str(file) + '.npz')

        self.recorded_traj = data['traj']
        self.recorded_ori = data['ori']
        self.recorded_gripper = data['grip']

    def goal_checker(self, goal):
        rospy.loginfo('moving to the goal point')
        self.go_to_pose(goal)

    def transpose_to_new_box(self):
        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)

        trans1 = tfBuffer.lookup_transform('/detectedBox' , '/base_link', rospy.Time()) ####### CORRECT FRAME NAMES
        trans2 = tfBuffer.lookup_transform('/referenceBox', '/base_link', rospy.Time()) ####### CORRECT FRAME NAMES

        # Global frame
        p0pos = np.array([0, 0, 0])  # x y z
        p0orient = Quaternion(1, 0, 0, 0)  # w x y z

        # Training box frame
        #p1pos = np.array([0.516573309298, 0.209258280499, 0.111873932258])  # x y z
        p1pos = np.array([trans1.transform.translation.x, trans1.transform.translation.y, trans1.transform.translation.z])
        #p1orient = Quaternion(0, 1, 0, 0)  # w x y z
        p1orient = Quaternion(trans1.transform.rotation.w, trans1.transform.rotation.x, trans1.transform.rotation.y, trans1.transform.rotation.z)

        # Testing box frame
        #p2pos = np.array([0.558391547136, 0.223972650574, 0.110788910536])
        #p2pos = np.array([0.626262545733,0.0598715897239, 0.113789855194])  # x y z
        p2pos = np.array([trans2.transform.translation.x, trans2.transform.translation.y, trans2.transform.translation.z])
        #p2orient = Quaternion(0, -0.7071, 0.7071, 0)
        p2orient = Quaternion(trans2.transform.rotation.w, trans2.transform.rotation.x, trans2.transform.rotation.y, trans2.transform.rotation.z)
        #p2orient = Quaternion(1, 0, 0, 0)

        translation_vector_p0p1 = p1pos - p0pos
        translation_vector_p0p2 = p2pos - p0pos
        translation_vector_p1p2 = p2pos - p1pos

        #print('Original pos in global frame', LfD.recorded_traj[:, 2])

        self.recorded_traj[0, :] -= translation_vector_p0p1[0]
        self.recorded_traj[1, :] -= translation_vector_p0p1[1]
        self.recorded_traj[2, :] -= translation_vector_p0p1[2]
        #print('Original pos in training box frame', LfD.recorded_traj[:, 2])
        
        for i in range(len(LfD.recorded_traj[0, :])):
            self.recorded_traj[:, i] = p1orient.inverse.rotate(LfD.recorded_traj[:, i])
            self.recorded_traj[:, i] = p2orient.rotate(LfD.recorded_traj[:, i])
        #print('Rotated ', LfD.recorded_traj[:, 2])

        self.recorded_traj[0, :] += translation_vector_p0p2[0]
        self.recorded_traj[1, :] += translation_vector_p0p2[1]
        self.recorded_traj[2, :] += translation_vector_p0p2[2]

        #print('Translated pos in training box frame', LfD.recorded_traj)

    def transpose_to_new_box_revised(self):
        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)

        #Check if transforms exist
        trans_to_ref = tfBuffer.can_transform('/referenceBox' , '/baselink', rospy.Time()) ####### CORRECT FRAME NAMES
        trans_to_base = tfBuffer.can_transform('/baselink' , '/detectedBox', rospy.Time()) ####### CORRECT FRAME NAMES

        #Get 3d carthesian data in the baselink
        #Load data and make sure it is PoseStamped with frame, not just xyz and wxyz
        LfD.recorded_traj.header='/baselink'

        #Transfrom 3d carthesian data from baselink to reference box
        #LfD.recorded_traj = trans_to_ref(LfD.recorded_traj, header='/referenceBox')
        #LfD.recorded_traj = tfBuffer.transform(LfD.recorded_traj, 'referenceBox')

        #Do it stepwise for each pose if a full list is not available
        for POSE in range(len(LfD.recorded_traj)):
            LfD.recorded_traj[POSE] = tfBuffer.transform(LfD.recorded_traj[POSE], '/referenceBox')

        #Change the header to detected box
        LfD.recorded_traj.header = '/detectedBox'

        #Transform back from detected box to base frame
        #LfD.recorded_traj = trans_to_base(LfD.recorded_traj, header='/baselink')
        for POSE in range(len(LfD.recorded_traj)):
            LfD.recorded_traj[POSE] = tfBuffer.transform(LfD.recorded_traj[POSE], '/baselink')
        

        #Put everything in a single loop
        for POSE in range(len(LfD.recorded_traj)):
            currentPose = LfD.recorded_traj[POSE]                   #Get Pose
            currentPose = PoseStamped()                             # BLANK FOR TESTING
            #currentPose.header.stamp = rospy.Time.now()             #Set time to now, might not be needed
            currentPose.header.frame_id = '/baselink'               #Set right header
            tfBuffer.transform(currentPose, '/referenceBox', rospy.Duration(0), PoseStamped())        #Transform to box
            
            currentPose.header.frame_id = '/detectedBox'            #Move box by changing header
            tfBuffer.transform(currentPose, '/baselink')            #Transform back to base
            LfD.recorded_traj[POSE] = currentPose                   #Save Pose


#%%
if __name__ == '__main__':
    rospy.init_node('LfD', anonymous=True)
#%%    
    LfD=LfD()
    time.sleep(1)
#%%
    print(sys.argv)
    if len(sys.argv)<3:
        LfD.traj_rec()

    else:
        LfD.load(sys.argv[2])
        print("Recorded trajectory full",LfD.recorded_traj)
        print("Recorded trajectory x",LfD.recorded_traj[0])
        print("Current Position", LfD.curr_pos)


    

#%%

    start = PoseStamped()
#
    start.pose.position.x = LfD.recorded_traj[0][0]
    start.pose.position.y = LfD.recorded_traj[1][0]
    start.pose.position.z = LfD.recorded_traj[2][0]

    start.pose.orientation.w = LfD.recorded_ori[0][0]
    start.pose.orientation.x = LfD.recorded_ori[1][0]
    start.pose.orientation.y = LfD.recorded_ori[2][0]
    start.pose.orientation.z = LfD.recorded_ori[3][0]
    LfD.go_to_pose(start)


#%%
    LfD.execute()

    print('Save new changes')

    LfD.save(sys.argv[-1])


    # goal = PoseStamped()

    # goal.header.seq = 1
    # goal.header.stamp = rospy.Time.now()
    # goal.header.frame_id = "map"

    # goal.pose.position.x = -0.03
    # goal.pose.position.y = -0.5
    # goal.pose.position.z = 0.4

    # goal.pose.orientation.w = 0.0
    # goal.pose.orientation.x = 0.0
    # goal.pose.orientation.y = 1.0
    # goal.pose.orientation.z = 0.0
    # LfD.go_to_pose(goal)

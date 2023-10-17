import numpy as np
import rospy
import dynamic_reconfigure.client
from panda_ros import Panda
from panda_ros.pose_transform_functions import array_quat_2_pose, pose_2_transformation, orientation_2_quaternion, list_2_quaternion
from geometry_msgs.msg import PoseStamped
from quaternion_algebra.algebra import quaternion_divide, to_euler_angles
from quaternion import as_euler_angles
class MassEstimator(Panda):
    def __init__(self):
        rospy.init_node("mass_estimator_node")
        super(MassEstimator, self).__init__()
        self.set_mass = dynamic_reconfigure.client.Client('/dynamic_reconfigure_mass_param_node', config_callback=None)
        self.goal_sub = rospy.Subscriber('/equilibrium_pose', PoseStamped, self.ee_pos_goal_callback)
        rospy.sleep(1)    
        self.estimated_mass=0
        self.estimated_offset_x=0
        self.estimated_offset_y=0
        self.estimated_offset_z=0

    def rotate_right(self):
        pos_array = np.array([0.6, 0, 0.4])
        quat = np.quaternion(0, 1, 0, 0)
        goal = array_quat_2_pose(pos_array, quat)
        goal.header.seq = 1
        goal.header.stamp = rospy.Time.now()

        ns_msg = [0, 0, 0, -2.4, 0, 2.4, 0]
        self.go_to_pose(goal)
        self.set_configuration(ns_msg)
        self.set_K.update_configuration({"nullspace_stiffness":10})

        rospy.sleep(rospy.Duration(secs=5))

        self.set_K.update_configuration({"nullspace_stiffness":0})

    def ee_pos_goal_callback(self, curr_conf):
        self.curr_pos_goal = np.array([curr_conf.pose.position.x, curr_conf.pose.position.y, curr_conf.pose.position.z])
        self.curr_ori_goal = np.array([curr_conf.pose.orientation.w, curr_conf.pose.orientation.x, curr_conf.pose.orientation.y, curr_conf.pose.orientation.z])


    def esitmate_mass(self):
        self.estimated_mass=self.estimated_mass+np.sign(self.curr_pos_goal[2]-self.curr_pos[2])
        self.set_mass.update_configuration({"mass": self.estimated_mass})
        
        curr_quat= list_2_quaternion(self.curr_ori)
        # print(curr_quat)
        curr_pose= array_quat_2_pose(self.curr_pos, curr_quat)
        curr_tranformation= pose_2_transformation(curr_pose.pose)
        orientation_matrix_transpose= np.transpose(curr_tranformation[0:2,:])
        goal_quat = list_2_quaternion(self.curr_ori_goal)
        # print("goal_quat")
        # print(goal_quat)
        quat_diff= quaternion_divide(goal_quat, curr_quat ) # curr -gaol
        # print("quat_diff")
        # print(quat_diff)
        euler= to_euler_angles(quat_diff)

        print("euler")
        print(euler)

        gradient= orientation_matrix_transpose @ np.array([euler[1], - euler[0]])
        # print("gradiet")
        # print(gradiet)
        self.estimated_offset_x=self.estimated_offset_x-np.sign(gradient[0])
        self.estimated_offset_y=self.estimated_offset_y-np.sign(gradient[1])
        self.estimated_offset_z=self.estimated_offset_z-np.sign(gradient[2])
        self.set_mass.update_configuration({"offset_x": self.estimated_offset_x})
        self.set_mass.update_configuration({"offset_y": self.estimated_offset_y})
        self.set_mass.update_configuration({"offset_z": self.estimated_offset_z})


        

if __name__ == '__main__':

    Estimator=MassEstimator()
    Estimator.rotate_right()
    for i in range (2000):
        Estimator.esitmate_mass()
        rospy.sleep(0.05)


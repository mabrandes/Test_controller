#!/usr/bin/env python3

import torch
from torch.autograd import Function, Variable
import torch.nn.functional as F
from torch import nn
from torch.nn.parameter import Parameter

from mpc import mpc
from mpc.mpc import QuadCost, LinDx, GradMethods
from mpc import util
#from mpc.env_dx import pendulum

import numpy as np
import numpy.random as npr

import math

from tqdm import tqdm

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from rowesys_navigation_msgs.msg import AutonomousMode, RowPosition
from rowesys_base_msgs.msg import MotorInputs

from tf.transformations import quaternion_matrix
from tf.transformations import euler_from_quaternion, quaternion_from_euler



# contains the robot model
class RosieDx(nn.Module):

    def __init__(self, speed_x=None):
        super().__init__()

        
        self.dt = 0.05
        self.n_state = 2
        self.n_ctrl = 2

        self.d1 = 0.5
        self.d2 = 0.8
        
        if speed_x is None:
            self.speed_x = 0.2
        else:
            self.speed_x = speed_x

        self.goal_state = torch.Tensor([0., 0.])
        self.goal_weights = torch.Tensor([1., 5.])
        self.ctrl_penalty = 0.001
        self.lower = -0.1 #torch.Tensor([-0.5, -0.5])
        self.upper = 0.1 #torch.Tensor([0.5, 0.5])

        #self.mpc_eps = 1e-3
        self.linesearch_decay = 0.2
        self.max_linesearch_iter = 5

        self.max_angvel = 0.1
        self.max_speedy = 0.1

        #self.max_angacc = 2.0 * self.dt
        #self.max_linacc = 2.0 * self.dt

        self.last_ang_vel = 0
        self.last_speed_y = 0

    def forward(self, x, u):
        squeeze = x.ndimension() == 1

        if squeeze:
            x = x.unsqueeze(0)
            u = u.unsqueeze(0)

        assert x.ndimension() == 2
        assert x.shape[0] == u.shape[0]
        assert x.shape[1] == 2
        assert u.shape[1] == 2
        assert u.ndimension() == 2
                
        u_ang_vel, u_speed_y = torch.unbind(u, dim=1)
        wheel_offset, gf_offset = torch.unbind(x, dim=1)

        #u_ang_vel = torch.clamp(u, -self.max_angvel, self.max_angvel)[:,0]
        #u_speed_y = torch.clamp(u, -self.max_speedy, self.max_speedy)[:,1]
        
        # take max acceleration into account
        #if u_ang_vel - self.last_ang_vel > self.max_angacc:
        #    u_ang_vel = self.last_ang_vel + self.max_angacc + (u_ang_vel - self.last_ang_vel - self.max_angacc)*0.1

        #elif u_ang_vel - self.last_ang_vel < -self.max_angacc:
        #    u_ang_vel = self.last_ang_vel - self.max_angacc + (u_ang_vel - self.last_ang_vel + self.max_angacc)*0.1

        #if u_speed_y - self.last_speed_y > self.max_linacc:
        #    u_speed_y = self.last_speed_y + self.max_linacc + (u_speed_y - self.last_speed_y - self.max_linacc)*0.1

        #elif u_speed_y - self.last_speed_y < -self.max_linacc:
        #    u_speed_y = self.last_speed_y - self.max_linacc + (u_speed_y - self.last_speed_y + self.max_linacc)*0.1

        #self.last_ang_vel = u_ang_vel
        #self.last_speed_y = u_speed_y


        #calculate new offsets and states
        d_wheel_offset = -u_speed_y - u_ang_vel*self.d1 + self.speed_x*(wheel_offset-gf_offset)/(self.d1 + self.d2)
        d_gf_offset    = -u_speed_y + u_ang_vel*self.d2 + self.speed_x*(wheel_offset-gf_offset)/(self.d1 + self.d2)
        
        state_y1 = wheel_offset + d_wheel_offset*self.dt
        state_y2 = gf_offset    + d_gf_offset*self.dt

        state = torch.stack((state_y1, state_y2), dim=1)

        if squeeze:
            state = state.squeeze(0)
        return state


class MPC():

    def __init__(self):

        self.desired_speed_x = 0.3
        self.start_mpc = True
        self.offset_y1 = 0
        self.offset_y2 = 0

        self.dt = 0
        self.timestamp = rospy.get_rostime()
        self.input_list = None
        self.steps = 20

        #subscribe to line offsets
        self.line_subscriber = rospy.Subscriber('/ov_msckf/pathimu', RowPosition, self.line_position_callback, queue_size=1)
        #self.line_subscriber = rospy.Subscriber('/rowesys/robot_wheel_odometry', Odometry, self.line_position_callback, queue_size=1)

        #self.autonomous_mode_subscriber = rospy.Subscriber('/rowesys/robot_autonomous_mode', AutonomousMode, self.autonomous_callback, queue_size=1)
        
        #self.swerve_input_subscriber = rospy.Subscriber('/cmd_vel', MotorInputs, self.swerve_input_callback, queue_size=1)


    def execute_mpc(self):

        if self.start_mpc:

            dx = RosieDx(speed_x=self.desired_speed_x)

            n_batch, T, mpc_T = 1, 1, self.steps

            x_init = torch.tensor([[self.offset_y1, self.offset_y2]]) #sample coordinates y1,y2

            x = x_init
            u_init = None


            goal_weights = torch.Tensor((1., 5.))
            goal_state = torch.Tensor((0., 0.))
            ctrl_penalty = 0.2
            q = torch.cat((
                goal_weights,
                ctrl_penalty*torch.ones(dx.n_ctrl)
            ))
            px = -torch.sqrt(goal_weights)*goal_state
            p = torch.cat((px, torch.zeros(dx.n_ctrl)))
            Q = torch.diag(q).unsqueeze(0).unsqueeze(0).repeat(
                mpc_T, n_batch, 1, 1
            )
            p = p.unsqueeze(0).repeat(mpc_T, n_batch, 1)

            for t in tqdm(range(T)):
                nominal_states, nominal_actions, nominal_objs = mpc.MPC(
                    dx.n_state, dx.n_ctrl, mpc_T,
                    u_init=u_init,
                    u_lower=dx.lower, u_upper=dx.upper,
                    lqr_iter=20,
                    verbose=0,
                    exit_unconverged=False,
                    detach_unconverged=False,
                    linesearch_decay=dx.linesearch_decay,
                    max_linesearch_iter=dx.max_linesearch_iter,
                    slew_rate_penalty=5,
                    grad_method=GradMethods.AUTO_DIFF,
                    eps=10e-2,
                )(x, QuadCost(Q, p), dx)
            
            #u_list = nominal_actions[0,:].tolist()
            #print(u_list[0][1])
            
            self.input_list = nominal_actions
            self.timestamp = rospy.get_rostime()
            self.dt = dx.dt


    # callback, if autonomous mode is linefollowing
    def autonomous_callback(self, data):

        if data.autonomous_mode == AutonomousMode.AUTONOMOUS_IN_FIELD:
            self.start_mpc = True
        else:
            self.start_mpc = False

    def line_position_callback(self, data):
        # define angle and offset
        #self.line_offset = data.offset
        #self.line_angle = data.angle
	

        self.line_offset = data.pose.pose.position.y
        self.line_angle = data.pose.pose.orientation.z

        self.offset_y1 = self.line_offset
        self.offset_y2 = self.line_offset - 1.3*math.atan(self.line_angle)
        

        if self.start_mpc:
            self.execute_mpc()

    
    def get_rotation (msg):
        global roll, pitch, yaw
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
    

    
    def swerve_input_callback(self, data):

        if data.left_front_wheel == 0 and data.left_back_wheel == 0 and data.right_front_wheel == 0 and data.right_back_wheel == 0:
            self.timestamp = rospy.get_rostime()



def main():

    rospy.init_node('rowesys_simple_mpc_node')

    
    mpc = MPC()

    # Create a rate
    rate = rospy.Rate(30)

    while not rospy.is_shutdown():
        if mpc.start_mpc and not mpc.dt == 0:
            
            # publish robot twist

            robot_twist_pub = rospy.Publisher('/rowesys/rowesys_ackermann_steering_controller/cmd_vel', Twist, queue_size=10)

            twist_msg = Twist()

            sum_dt = (rospy.get_rostime()).to_sec() - mpc.timestamp.to_sec()

            index = int(sum_dt/mpc.dt)

            if(index >= mpc.steps):
                twist_msg.linear.x = 0
                twist_msg.linear.y = 0
                twist_msg.angular.z = 0
            else:
                u_list = mpc.input_list[index,:].tolist()

                twist_msg.linear.x = mpc.desired_speed_x
                twist_msg.linear.y = u_list[0][1]
                twist_msg.angular.z = u_list[0][0]

            robot_twist_pub.publish(twist_msg)

        rate.sleep()


if __name__ == '__main__':
    main()

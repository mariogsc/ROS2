#!/usr/bin/env python3

import rclpy 
import math
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

class pid:
    def __init__(self,P=2.0,I=0.0,D=1.0):
        self.Kp=P
        self.Ki=I
        self.Kd=D

        self.P_value=0.0
        self.D_value=0.0
        self.I_value=0.0
        self.set_point=0.0 
        self.error=0.0

    def update(self,error):
        self.P_value=self.Kp*error
        self.D_value=self.Kd*(error-self.error)
        self.I_value=self.Ki*(self.I_value+error)
        # valores máximos para evitar el wind up
        if(self.I_value>100):self.I_value=100
        elif(self.I_value<-100):self.I_value=-100
        self.error=error
        return (self.P_value+self.I_value+self.D_value)
    
    def set_parameters(self,P,I,D):
        self.Kp=P
        self.Ki=I
        self.Kd=D
    
    def reset(self):
        self.P_value=0.0
        self.D_value=0.0
        self.I_value=0.0

    def setPoint(self,sp):
        self.setPoint=sp
        self.reset()

class PidController(Node):
    def __init__(self):
        super().__init__('turtle_pid')
        self.desired_x=1.0
        self.desired_y=1.0
        #self.get_logger().info('turtle pid has been started')
        self.cmd_vel_publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel',10)
        self.pose_subscriber_= self.create_subscription(Pose, "/turtle1/pose", self.pose_callback, 10)
    
    def velocity_control(self,vel,ang):
        msg=Twist()
        msg.linear.x=vel
        msg.angular.z=ang
        self.cmd_vel_publisher_.publish(msg)


    def pose_callback(self, msg: Pose):
        #Calculate errors    
        err_x=self.desired_x-msg.x
        err_y=self.desired_y-msg.y
        err_dist=(err_x**2+err_y**2)**0.5
        desired_theta=math.atan2(err_y,err_x)
        err_theta=desired_theta-msg.theta

        # Acota el error de theta
        while(err_theta>math.pi):
            err_theta-=2.0*math.pi
        while(err_theta<-math.pi):
            err_theta+=2.0*math.pi
        
        controller_lin=pid(0.4,0.05,0.1)
        controller_ang=pid(1.5,0,0)
        
        vel=controller_lin.update(err_dist)
        ang=controller_ang.update(err_theta)

        self.get_logger().info("[err_dist="+str(err_dist)+", err_theta="+str(err_dist)+"]")
        self.velocity_control(vel,ang)

    

def main(args=None):
    rclpy.init(args=args)
    node=PidController()
    rclpy.spin(node)
    rclpy.shutdown()
#!/usr/bin/env python
import rospy
import numpy as np
from math import *
from rospy.timer import sleep
from geometry_msgs.msg import PoseStamped,Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from PID import PIDController
from RobotAction import RobotAction

kp1 = 0.2
ki1 = 0
kd1 = 0
v_min = 0.05 # m/s
v_max = 0.25

kp2 = 0.05
ki2 = 0
kd2 = 0.015
w_min = -2.5 # rad/s
w_max = 2.5

class ControllRobot():
    def __init__(self,kp1,ki1,kd1,kp2,ki2,kd2,v_min,v_max,w_min,w_max):
        
        rospy.init_node('controll_xe',anonymous=True)
        self.sub_odom=rospy.Subscriber('/odom',Odometry,self.get_state)
        self.sub_goal=rospy.Subscriber('/move_base_simple/goal',PoseStamped,self.get_goal)
        self.pid_linear = PIDController(kp1,ki1,kd1,v_min,v_max)
        self.pid_rotate = PIDController(kp2,ki2,kd2,w_min,w_max)

        self.x_current=None
        self.y_current=None
        self.theta_current=None

        self.x_goal=None
        self.y_goal=None
        self.theta_goal=None

        self.delta_t = 0
        self.current_time = rospy.Time.now()
        self.last_time = rospy.Time.now()
        self.last_callback_time = None 

        self.action = RobotAction.none
        self.move = False

    def get_state(self,msg):
        current_time = rospy.get_time() 
        self.x_current = msg.pose.pose.position.x
        self.y_current = msg.pose.pose.position.y
        (roll,pitch,self.theta_current) = euler_from_quaternion([msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w])
        self.theta_current = degrees(self.theta_current)

        if self.last_callback_time is not None:
            self.delta_t = current_time - self.last_callback_time + 1e-6 # Tính khoảng thời gian từ lần gọi hàm trước đó
        self.last_callback_time = current_time

    def get_goal(self,msg):
        self.x_goal = msg.pose.position.x
        self.y_goal = msg.pose.position.y
        (roll,pitch,self.theta_goal) = euler_from_quaternion([msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w])
        self.theta_goal = degrees(self.theta_goal)
        self.move = True
        self.action = RobotAction.move

    def distance2point(self,x1, y1, x2, y2):
        return sqrt(pow(x2-x1,2)+pow(y2-y1,2))
    
    def limitangle(self,angle):
        if angle > 180:
            angle -= 360
        elif angle < -180:
            angle += 360
        return angle

    def Calculator(self,x_current,y_current,theta_current,x_goal,y_goal,theta_goal):
        # Tính toán khoảng cách giữa robot và điểm đích theo các trục x và y
        delta_x = x_goal - x_current
        delta_y = y_goal - y_current

        # Tính toán góc giữa trục ox và hướng điểm đích của robot
        alpha = degrees(atan2(delta_y, delta_x))
        direction = self.limitangle(alpha-theta_current)
        
        distance = self.distance2point(x_current,y_current,x_goal,y_goal)
        orientation = self.limitangle(theta_goal-theta_current)

        return direction,distance,orientation

    def RunController(self):

        rospy.init_node('controll_xe',anonymous=True)
        pub_vel=rospy.Publisher('/cmd_vel',Twist,queue_size = 1)
        speed = Twist()
        
        while not rospy.is_shutdown():

            if(self.action != RobotAction.none):
                direction,distance,orientation=self.Calculator(self.x_current,self.y_current,self.theta_current,self.x_goal,self.y_goal,self.theta_goal)
                if(self.move == True):
                    print("new_goal")
                    self.action = RobotAction.direct
                    self.move = False

                if(self.action == RobotAction.direct):
                    angular_velocy = self.pid_rotate.compute(direction,self.delta_t)
                    if( 0<angular_velocy<0.2):
                        angular_velocy = 0.2
                    elif(-0.2<angular_velocy<0):
                        angular_velocy = -0.2
                    speed.angular.z = round(angular_velocy,4)
                    if(abs(direction)<=0.05):
                        speed.angular.z = 0
                        self.action = RobotAction.linear
                        pub_vel.publish(speed)
                
                if(self.action == RobotAction.linear):
                    linear_velocy = self.pid_linear.compute(distance,self.delta_t)
                    speed.linear.x = linear_velocy

                    angular_velocy = self.pid_rotate.compute(direction,self.delta_t)
                    if( 0<angular_velocy<0.2):
                        angular_velocy = 0.2
                    elif(-0.2<angular_velocy<0):
                        angular_velocy = -0.2
                    speed.angular.z = round(angular_velocy,4)
                    if(distance <= 0.01):
                        speed.linear.x = 0
                        speed.angular.z = 0
                        self.action = RobotAction.orient
                        pub_vel.publish(speed)

                if(self.action == RobotAction.orient):
                    angular_velocy = self.pid_rotate.compute(orientation,self.delta_t)
                    if( 0<angular_velocy<0.2 ):
                        angular_velocy = 0.2
                    elif( -0.2<angular_velocy<0 ):
                        angular_velocy = -0.2
                    speed.angular.z = round(angular_velocy,4)
                    if(abs(orientation)<=0.05):
                        speed.angular.z = 0
                        self.action = RobotAction.done
                        pub_vel.publish(speed)
                
                if(self.action == RobotAction.done):
                    print('done')
                    speed.linear.x = 0
                    speed.angular.z = 0
                    self.move = False
                    self.action = RobotAction.none
                    
                speed.linear.y=0
                speed.linear.z=0
                speed.angular.x=0
                speed.angular.y=0
                pub_vel.publish(speed)
                # rospy.sleep(0.01)


if __name__ == '__main__':
    try:
        controll = ControllRobot(kp1,ki1,kd1,kp2,ki2,kd2,v_min,v_max,w_min,w_max)
        controll.RunController()
    except rospy.ROSInterruptException: 
        pass
#rosrun rosserial_python serial_node.py /dev/ttyACM0
#!/usr/bin/env python
import rospy
import numpy as np
from math import *
from rospy.timer import sleep
from std_msgs import msg
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped,Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
#toa do xe hien tai
x_xe=0
y_xe=0
thetax_xe=0
thetay_xe=0
thetaz_xe=0
thetaw_xe=0
#toa do vi tri dich
goal_x=0
goal_y=0
thetax_goal=0
thetay_goal=0
thetaz_goal=0
thetaw_goal=0
yaw2=0
#thong tin xe voi vi tri dich 
anpha=0
dis=0
theta=0
#ham di chuyen
# t=0.1#tang x
# n=0#dem
# a=0#chan duoi
# b=0#chan tren
next=0
def toa_do_xe(data):
    global x_xe,y_xe,thetax_xe,thetay_xe,thetaz_xe,thetaw_xe,dis,anpha,theta
    pose=data
    x_xe=round(pose.pose.pose.position.x,4)
    y_xe=round(pose.pose.pose.position.y,4)
    thetax_xe=round(pose.pose.pose.orientation.x,4)
    thetay_xe=round(pose.pose.pose.orientation.y,4)
    thetaz_xe=round(pose.pose.pose.orientation.z,4)
    thetaw_xe=round(pose.pose.pose.orientation.w,4)
    (roll1, pitch1, yaw1) = euler_from_quaternion ([thetax_xe,thetay_xe,thetaz_xe,thetaw_xe])
#toa do thuan nhat goal voi goc
    Tgoc_goal=np.array([[cos(yaw2) ,-sin(yaw2) ,0 ,goal_x],
                            [sin(yaw2) ,cos(yaw2) ,0 ,goal_y],
                            [0 ,0 ,1 ,0],
                            [0 ,0 ,0 ,1]])
#toa do thuan nhat xe voi goc
    Tgoc_xe=np.array([[cos(yaw1) ,-sin(yaw1) ,0 ,x_xe],
                          [sin(yaw1) ,cos(yaw1) ,0 ,y_xe],
                          [0 ,0 ,1 ,0],
                          [0 ,0 ,0 ,1]])
#toa do thuan nhat goal voi xe
    i=Tgoc_xe.item((0,3))
    j=Tgoc_xe.item((1,3))
    a=i*Tgoc_xe.item((0,0))+j*Tgoc_xe.item((1,0))
    b=i*Tgoc_xe.item((0,1))+j*Tgoc_xe.item((1,1))
    Txe_goc=np.array([[cos(yaw1) ,sin(yaw1) ,0 ,-a],
                          [-sin(yaw1) ,cos(yaw1) ,0 ,-b],
                          [0 ,0 ,1 ,0],
                          [0 ,0 ,0 ,1]])
    Txe_goal=Txe_goc.dot(Tgoc_goal)

    dis=sqrt(pow(Txe_goal.item((0,3)),2)+pow(Txe_goal.item((1,3)),2))
    anpha=atan2(Txe_goal.item((1,3)),Txe_goal.item((0,3)))
    theta=atan2(Txe_goal.item((1,0)),Txe_goal.item((0,0)))
def van_toc_dai(dis,KP=1.5):
    return KP*dis
def van_toc_goc(anpha,KP=1.5):
    return KP*anpha  
    
def dich(msg):
    global goal_x,goal_y,thetax_goal,thetay_goal,thetaz_goal,thetaw_goal,yaw2
    goal_x=msg.pose.position.x 
    goal_y=msg.pose.position.y
    thetax_goal=msg.pose.orientation.x
    thetay_goal=msg.pose.orientation.y
    thetaz_goal=msg.pose.orientation.z
    thetaw_goal=msg.pose.orientation.w
    (roll2, pitch2, yaw2) = euler_from_quaternion ([thetax_goal,thetay_goal,thetaz_goal,thetaw_goal])
vi_tri_xe_subscriber=rospy.Subscriber('/Diff_Drive/diff_drive_controller/odom',Odometry,toa_do_xe)
dich_subscriber=rospy.Subscriber('/move_base_simple/goal',PoseStamped,dich)

def dieu_khien_xe():
    global n
    rospy.init_node('controll_xe',anonymous=True)
    van_toc_publisher=rospy.Publisher('/Diff_Drive/diff_drive_controller/cmd_vel',Twist,queue_size = 1)
    
    while not rospy.is_shutdown():

        speed=Twist()
        print(str(dis)+" --- "+str(anpha)+" --- "+str(theta))
        if(abs(anpha)>0.785 or (0<dis<1) or (abs(anpha)>0.785 and (0<dis<1))):
            while(dis>0.05 and next==0):       
                speed.linear.x=0
                speed.linear.y=0        
                speed.linear.z=0
                speed.angular.x=0
                speed.angular.y=0
                if(van_toc_goc(anpha)>1.5):
                    speed.angular.z=1.5
                if(van_toc_goc(anpha)<-1.5):
                    speed.angular.z=-1.5
                else:
                    speed.angular.z=van_toc_goc(anpha)
                van_toc_publisher.publish(speed) 
                rospy.sleep(0.1)
                if(abs(anpha)<0.05):
                    next=1
                    break
            #continue        
        while(dis>0.01):  
            if(van_toc_dai(dis)>0.2):
                speed.linear.x=0.2
            else:                
                speed.linear.x=van_toc_dai(dis)
            speed.linear.y=0        
            speed.linear.z=0      
            speed.angular.x=0
            speed.angular.y=0
            if(van_toc_goc(anpha)>1.5):
                speed.angular.z=1.5
            if(van_toc_goc(anpha)<-1.5):
                speed.angular.z=-1.5
            else:
                speed.angular.z=van_toc_goc(anpha) 
            van_toc_publisher.publish(speed)
            rospy.sleep(0.1)  
        speed.linear.x=0
        speed.angular.z=0
        van_toc_publisher.publish(speed)
        while(abs(theta)>0.02):
            speed.linear.x=0
            speed.linear.y=0        
            speed.linear.z=0
            speed.angular.x=0
            speed.angular.y=0
            if(van_toc_goc(theta)>1.5):
                speed.angular.z=1.5
            if(van_toc_goc(theta)<-1.5):
                speed.angular.z=-1.5
            else:
                speed.angular.z=van_toc_goc(theta)   
            van_toc_publisher.publish(speed)
            rospy.sleep(0.1)
        next=0
        speed.linear.x=0
        speed.angular.z=0
        van_toc_publisher.publish(speed)
        rospy.sleep(0.1)
if __name__ == '__main__':
    try:
        dieu_khien_xe()
    except rospy.ROSInterruptException: 
        pass
#rosrun rosserial_python serial_node.py /dev/ttyACM0
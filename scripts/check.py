from math import *

def distance2point(x1, y1, x2, y2):
    return sqrt(pow(x2-x1,2)+pow(y2-y1,2))

def angle2vector(x1, y1, x2, y2):
    theta = degrees(atan2(y2, x2) - atan2(y1, x1))
    if theta > 180:
        theta -= 360
    elif theta < -180:
        theta += 360
    return theta

def limitangle(angle):
    if angle > 180:
        angle -= 360
    elif angle < -180:
        angle += 360
    return angle
def convert_to_robot_coordinate(x_r, y_r, theta_r, x_g, y_g):
    # Tính toán khoảng cách giữa robot và điểm đích theo các trục x và y
    delta_x = x_g - x_r
    delta_y = y_g - y_r

    # Tính toán góc giữa trục x và hướng của robot
    alpha = degrees(atan2(delta_y, delta_x))

    # Tính toán khoảng cách Euclid giữa robot và điểm đích
    distance = sqrt(delta_x**2 + delta_y**2)

    # Tính toán tọa độ điểm G theo hệ tọa độ robot
    x_g_robot = distance * cos(radians(alpha - theta_r))
    y_g_robot = distance * sin(radians(alpha - theta_r))

    # Trả về tọa độ điểm G theo hệ tọa độ robot
    return x_g_robot, y_g_robot
def Calculator(x_current,y_current,theta_current,x_goal,y_goal,theta_goal):
    direction = limitangle(angle2vector(1,0,x_goal-x_current,y_goal-y_current)-theta_current)
    
    
    distance = distance2point(x_current,y_current,x_goal,y_goal)
    orientation = limitangle(theta_goal-theta_current)

    return direction,distance,orientation

# direction,distance,orientation=Calculator(1,1,145,-2,-1,0)
# print(direction)
# print(distance)
# print(orientation)
print(cos(radians(45)))
xgr,ygr = convert_to_robot_coordinate(1,1,0,-2,-1)
print(xgr)
print(ygr)

# from enum import Enum

# class RobotAction(Enum):
#     non = 1
#     direct = 2
#     linear = 3
#     orient = 4
#     done = 5

# # Sử dụng enum
# action = RobotAction.direct
# print(action)  # RobotAction.direct
# action = RobotAction.linear
# print(action)
# So sánh các giá trị enum
# if action == RobotAction.done:
#     print("Hành động đã hoàn thành")
# else:
#     print("Hành động chưa hoàn thành")
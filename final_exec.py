#!/usr/bin/env python

import sys
import copy
import time
import rospy
import cv2

import numpy as np
from final_header import *
from final_func import *


################ Pre-defined parameters and functions below (can change if needed) ################

# 20Hz
SPIN_RATE = 20  

# UR3 home location
home = [174.44*PI/180, -68.32*PI/180.0, 91.11*PI/180.0, -106.41*PI/180.0, -88.41*PI/180.0, 85.25*PI/180.0]  

# UR3 current position, using home position for initialization
current_position = copy.deepcopy(home)  

thetas = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

digital_in_0 = 0
analog_in_0 = 0.0

suction_on = True
suction_off = False

current_io_0 = False
current_position_set = False


"""
Whenever ur3/gripper_input publishes info this callback function is called.
"""
def input_callback(msg):
    global digital_in_0
    digital_in_0 = msg.DIGIN
    digital_in_0 = digital_in_0 & 1 # Only look at least significant bit, meaning index 0


"""
Whenever ur3/position publishes info, this callback function is called.
"""
def position_callback(msg):
    global thetas
    global current_position
    global current_position_set

    thetas[0] = msg.position[0]
    thetas[1] = msg.position[1]
    thetas[2] = msg.position[2]
    thetas[3] = msg.position[3]
    thetas[4] = msg.position[4]
    thetas[5] = msg.position[5]

    current_position[0] = thetas[0]
    current_position[1] = thetas[1]
    current_position[2] = thetas[2]
    current_position[3] = thetas[3]
    current_position[4] = thetas[4]
    current_position[5] = thetas[5]

    current_position_set = True


"""
Function to control the suction cup on/off
"""
def gripper(pub_cmd, loop_rate, io_0):
    global SPIN_RATE
    global thetas
    global current_io_0
    global current_position

    error = 0
    spin_count = 0
    at_goal = 0

    current_io_0 = io_0

    driver_msg = command()
    driver_msg.destination = current_position
    driver_msg.v = 1.0
    driver_msg.a = 1.0
    driver_msg.io_0 = io_0
    pub_cmd.publish(driver_msg)

    while(at_goal == 0):

        if( abs(thetas[0]-driver_msg.destination[0]) < 0.0005 and \
            abs(thetas[1]-driver_msg.destination[1]) < 0.0005 and \
            abs(thetas[2]-driver_msg.destination[2]) < 0.0005 and \
            abs(thetas[3]-driver_msg.destination[3]) < 0.0005 and \
            abs(thetas[4]-driver_msg.destination[4]) < 0.0005 and \
            abs(thetas[5]-driver_msg.destination[5]) < 0.0005 ):

            rospy.loginfo("Goal is reached!")
            at_goal = 1

        loop_rate.sleep()

        if(spin_count >  SPIN_RATE*5):

            pub_cmd.publish(driver_msg)
            rospy.loginfo("Just published again driver_msg")
            spin_count = 0

        spin_count = spin_count + 1

    return error


"""
Move robot arm from one position to another
"""
def move_arm(pub_cmd, loop_rate, dest, vel, accel, move_type):
    global thetas
    global SPIN_RATE

    error = 0
    spin_count = 0
    at_goal = 0

    driver_msg = command()
    driver_msg.destination = dest
    driver_msg.v = vel
    driver_msg.a = accel
    driver_msg.io_0 = current_io_0
    driver_msg.move_type = move_type  # Move type (MoveJ or MoveL)
    pub_cmd.publish(driver_msg)

    loop_rate.sleep()

    while(at_goal == 0):

        if( abs(thetas[0]-driver_msg.destination[0]) < 0.0005 and \
            abs(thetas[1]-driver_msg.destination[1]) < 0.0005 and \
            abs(thetas[2]-driver_msg.destination[2]) < 0.0005 and \
            abs(thetas[3]-driver_msg.destination[3]) < 0.0005 and \
            abs(thetas[4]-driver_msg.destination[4]) < 0.0005 and \
            abs(thetas[5]-driver_msg.destination[5]) < 0.0005 ):

            at_goal = 1
            rospy.loginfo("Goal is reached!")

        loop_rate.sleep()

        if(spin_count >  SPIN_RATE*5):

            pub_cmd.publish(driver_msg)
            rospy.loginfo("Just published again driver_msg")
            spin_count = 0

        spin_count = spin_count + 1

    return error

################ Pre-defined parameters and functions above (can change if needed) ################

##========= TODO: Helper Functions =========##

def filter_contours_by_distance(contours, min_distance):
    """
    Filters contours based on the spatial spread of their points.
    
    A contour is retained if at least one pair of points within it 
    has a distance greater than or equal to the specified min_distance.

    Parameters
    ----------
    contours : list of np.ndarray
        List of contours (each contour is an array of points).
    min_distance : float
        Minimum required distance between any two points in a contour.

    Returns
    -------
    list of np.ndarray
        Contours that satisfy the minimum distance criterion.
    """
    min_dist_squared = min_distance ** 2  # Use squared distance for efficiency
    filtered_contours = []

    for contour in contours:
        keep = False
        for i in range(len(contour)):
            for j in range(i + 1, len(contour)):  # Avoid redundant comparisons
                # Calculate squared distance between two points
                dist_squared = (contour[i][0] - contour[j][0])**2 + (contour[i][1] - contour[j][1])**2
                if dist_squared >= min_dist_squared:
                    keep = True
                    break
            if keep:
                break
        if keep:
            filtered_contours.append(contour)

    return filtered_contours

def filter_contour_by_points(contours, threshold): 
    filtered_contours = [contour for contour in contours if len(contour) > threshold]
    return filtered_contours

# def filter_contours_by_distance(contours, distance_threshold):
#     filtered_contours = []

#     for contour in contours:
#         # contour is Nx2
#         filtered_contour = [contour[0]]  # Start with the first point as is

#         for point in contour[1:]:
#             # Calculate Euclidean distance between the last appended point and the current point
#             last_point = filtered_contour[-1]
#             euclidean_distance = np.linalg.norm(point - last_point)

#             if euclidean_distance > distance_threshold:
#                 filtered_contour.append(point)  # Append if the distance is above the threshold

#         filtered_contours.append(np.array(filtered_contour))

#     return filtered_contours

# def sample_contours(contours, divisor):
#     sampled_contours = []
    
#     for contour in contours:
#         contour_length = len(contour)  # Get the number of points in the contour
#         interval = max(1, contour_length // divisor)  # Ensure interval is at least 1
        
#         # Sample every `interval` points
#         sampled_contour = contour[::interval]
        
#         # Ensure the last point is included (if desired, e.g. for closed shapes)
#         if len(contour) > 1 and not np.array_equal(contour[-1], sampled_contour[-1]):
#             sampled_contour = np.vstack([sampled_contour, contour[-1]])
        
#         sampled_contours.append(sampled_contour)
    
#     return sampled_contours

import cv2
import numpy as np

def find_keypoints(image):
    # Convert image to grayscale
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blurred_image = cv2.bilateralFilter(gray_image, d=9, sigmaColor=75, sigmaSpace=75)
    
    # Detect edges using Canny edge detector
    edges = cv2.Canny(blurred_image, 100, 200)

    # Find contours using the Canny edges
    contours, _ = cv2.findContours(edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    
    # Approximate each contour with fewer vertices using approxPolyDP
    epsilon = 0.02 * cv2.arcLength(contours[0], True)  # Set epsilon as a percentage of the contour perimeter
    approximated_contours = [cv2.approxPolyDP(contour, epsilon, True) for contour in contours]
    
    # Reshape contours from (N, 1, 2) to (N, 2) for easier handling (if required)
    approximated_contours = [c.reshape(-1, 2) for c in approximated_contours]

    threshold = 5
    filteredcontours = filter_contour_by_points(approximated_contours, threshold)
    sampledcontoursd = filter_contours_by_distance(filteredcontours, 3)

    keypoints = []

    # Convert each contour of keypoints into a Nx2 NumPy array
    for contour in sampledcontoursd:
        contour_keypoints = []
        for point in contour:
            x, y = point[0], point[1]
            xw, yw = IMG2W(x, y, image)
            contour_keypoints.append([xw, yw])

        # Convert to NumPy array to ensure the requested structure
        contour_keypoints = np.array(contour_keypoints)
        keypoints.append(contour_keypoints)
    return keypoints

#     # gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
#     # blurred_image = cv2.GaussianBlur(gray_image, (5, 5), 0)
#     # edges = cv2.Canny(blurred_image, 100, 200)
#     # # cv2.imshow('aaa',image)
#     # # cv2.imshow('Canny Edge Detection', edges)
#     # # cv2.imshow('Normal image', image)
#     # # cv2.waitKey(0)
#     # # cv2.destroyAllWindowsddd()

#     # # Edge detection
#     # # edges = cv2.Canny(blurred_image, 50, 150, None, 3)
    
#     # #  Standard Hough Line Transform
#     # lines = cv2.HoughLinesP(
#     #     edges, 
#     #     rho=1, 
#     #     theta=np.pi / 180, 
#     #     threshold=50, 
#     #     minLineLength=100,
#     #     maxLineGap=10
#     #     )
    
#     # """Gets keypoints from the given image

#     # Parameters
#     # ----------
#     # image : np.ndarray
#     #     The given image (before or after preprocessing)

#     # Returns
#     # -------
#     # keypoints
#     #     a list of keypoints detected in image coordinates
#     # """

#     # keypoints = []
#     # if lines is not None:
#     #     for line in lines:
#     #         x1,y1,x2,y2=line[0]
#     #         keypoints.append((x1,y1))
#     #         keypoints.append((x2,y2))
#     # keypoints=np.unique(np.array(keypoints),axis=0)
#     # print(len(keypoints))

#     # return keypoints
#     gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
#     blurred_image = cv2.GaussianBlur(gray_image, (5, 5), 0)
#     edges = cv2.Canny(blurred_image, 100, 200)

#     contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#     keypoints = []
#     for contour in contours:
#         epsilon = 0.005 * cv2.arcLength(contour, True)  # Adjust epsilon for approximation
#         approx = cv2.approxPolyDP(contour, epsilon, True)
#         keypoints.extend(approx)
#     return keypoints

# def sort_points_by_proximity(points):
#     """Sort points to minimize distance between consecutive points"""
#     if len(points) <= 1:
#         return points
        
#     sorted_points = [points[0]]
#     points = list(points[1:])
    
#     while points:
#         last_point = sorted_points[-1]
#         next_point = min(points, key=lambda p: np.sqrt(
#             (p[0] - last_point[0])**2 + (p[1] - last_point[1])**2
#         ))
#         sorted_points.append(next_point)
#         points.remove(next_point)
    
#     return np.array(sorted_points)
#     # cv2.imshow('aaa',image)
#     # cv2.imshow('Canny Edge Detection', edges)
#     # cv2.imshow('Normal image', image)
# def IMG2W(row, col, image):
#     """Transform image coordinates to world coordinates"""
#     # Define the paper dimensions in the world frame (meters)
#     paper_width = 0.2  # 20cm
#     paper_height = 0.3  # 30cm
    
#     # Get image dimensions
#     img_height, img_width = image.shape[:2]
    
#     # Calculate scaling factors
#     scale_x = paper_width / img_width
#     scale_y = paper_height / img_height
    
#     # Convert to world coordinates
#     # Assuming paper is centered at (0.3, 0, 0) in world frame
#     x = 0.3 - (paper_width/2) + col * scale_x
#     y = (paper_height/2) - row * scale_y







def IMG2W(row, col, image):
    top_left = [15, 6.5] #in cm
    #paper_dimensions = [27.94, 21.59] #in cm
    paper_dimensions = [24, 17] #in cm, ADJUSTED FOR NEW TOP LEFT SO PROTECTIVE STOP WON'T HAPPEN AT TOP LEFT
    paper_dim_ratio = paper_dimensions[0]/paper_dimensions[1] #NOW USING ADJUSTED paper_dimensions
    image_dim_ratio = image.shape[0]/image.shape[1]
    

    if image_dim_ratio > paper_dim_ratio:
        #scale vertical
        pixel_spacing = paper_dimensions[0]/image.shape[0]
        
    else:
        #scale horizontal
        pixel_spacing = paper_dimensions[1]/image.shape[1]

    
    x = (top_left[0] + row*pixel_spacing)/100
    y = (top_left[1] + col*pixel_spacing)/100

    return x,y+0.04    

def draw_image(pub_cmd,loop_rate,vel,accel,world_keypoints):
    for contour_index, contour in enumerate(world_keypoints):
        first_point = contour[0]
        x, y = first_point[0], first_point[1]
        z_safe = 0.01
        thetas = lab_invk(x, y, z_safe, 0)
        move_arm(pub_cmd, loop_rate, thetas, vel, accel, 'J')

        # Process each point in the contour
        for point_index, point in enumerate(contour):
            x, y = point[0], point[1]
            z_draw = 0.021
            thetas = lab_invk(x, y, z_draw, 0)
            move_arm(pub_cmd, loop_rate, thetas, vel, accel, 'J')

        # Return to a safe height after completing the contour
        thetas = lab_invk(x, y, z_safe, 0)
        move_arm(pub_cmd, loop_rate, thetas, vel, accel, 'L')
    #    """Draw the image based on detecte keypoints in world coordinates

#     Parameters
#     ----------
#     world_keypoints:
#         a list of keypoints detected in world coordinates
#     """
#     pass


"""
Program run from here
"""
def main():
    global home
    # global variable1
    # global variable2

    # Initialize ROS node
    rospy.init_node('lab5node')

    # Initialize publisher for ur3/command with buffer size of 10
    pub_command = rospy.Publisher('ur3/command', command, queue_size=10)

    # Initialize subscriber to ur3/position & ur3/gripper_input and callback fuction
    # each time data is published
    sub_position = rospy.Subscriber('ur3/position', position, position_callback)
    sub_input = rospy.Subscriber('ur3/gripper_input', gripper_input, input_callback)

    # Check if ROS is ready for operation
    while(rospy.is_shutdown()):
        print("ROS is shutdown!")

    # Initialize the rate to publish to ur3/command
    loop_rate = rospy.Rate(SPIN_RATE)

    # Velocity and acceleration of the UR3 arm
    vel = 4.0
    accel = 4.0


    
    move_arm(pub_command, loop_rate, home, vel, accel, 'J')  # Move to the home position

    ##========= TODO: Read and draw a given image =========##

    image = cv2.imread("/home/ur3/catkin_meisai_final/src/finalpkg_py/scripts/images/turkey.png")
    keypoints = find_keypoints(image)
    # print("^^^^^^^^^^^^^^^^^^^^^^^^^^^")
    print(len(keypoints))

    move_arm(pub_command, loop_rate, home, vel, accel, 'J')

    # # Draw the image
    draw_image(pub_command, loop_rate, vel,accel,keypoints)

    # #Return homeworld_keypoints
    # move_arm(pub_command, loop_rate, home, vel, accel, 'J')


    move_arm(pub_command, loop_rate, home, vel, accel, 'J')  # Return to the home position
    rospy.loginfo("Task Completed!")
    print("Use Ctrl+C to exit program")
    rospy.spin()

if __name__ == '__main__':

    try:
        main()
    # When Ctrl+C is executed, it catches the exception
    except rospy.ROSInterruptException:
        pass


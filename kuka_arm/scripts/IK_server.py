#!/usr/bin/env python

# Copyright (C) 2017 Electric Movement Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
        # Define symbols for sympy
        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')

        # define a function for definition of a homogeneous transform matrix
        # this saves decent amount of code
        def tf_matrix_from_dh_params(alpha, a, d, q):
            return Matrix([[cos(q),            -sin(q),            0,              a],
                           [ sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                           [ sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
                           [                 0,                 0,           0,             1]])


        # define DH Parameter values
        dh_parameters = {   
            alpha0: 0,      a0:   0,      d1: 0.75,  q1: q1,
            alpha1: -pi/2,  a1:   0.35,   d2: 0,     q2: -pi/2 + q2,
            alpha2: 0,      a2:   1.25,   d3: 0,     q3: q3,
            alpha3: -pi/2,  a3:   -0.054, d4: 1.50,  q4: q4,
            alpha4: pi/2,   a4:   0,      d5: 0,     q5: q5,
            alpha5: -pi/2,  a5:   0,      d6: 0,     q6: q6,
            alpha6: 0,      a6:   0,      d7: 0.303, q7: 0,
        }

        # Define all the transformation matrices from i to i+1 joijt
        T0_1  = tf_matrix_from_dh_params(alpha0, a0, d1, q1).subs(dh_parameters)
        T1_2  = tf_matrix_from_dh_params(alpha1, a1, d2, q2).subs(dh_parameters)
        T2_3  = tf_matrix_from_dh_params(alpha2, a2, d3, q3).subs(dh_parameters)
        T3_4  = tf_matrix_from_dh_params(alpha3, a3, d4, q4).subs(dh_parameters)
        T4_5  = tf_matrix_from_dh_params(alpha4, a4, d5, q5).subs(dh_parameters)
        T5_6  = tf_matrix_from_dh_params(alpha5, a5, d6, q6).subs(dh_parameters)
        T6_EE = tf_matrix_from_dh_params(alpha6, a6, d7, q7).subs(dh_parameters)

        # Define transformation from base to end effector
        T0_EE = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_EE

        # Define rotation matrices with symbols
        r, p, y = symbols('r, p, y')
        ROT_X = Matrix([[1,      0,       0],
                        [0, cos(r), -sin(r)],
                        [0, sin(r),  cos(r)]])

        ROT_Y = Matrix([[cos(p),   0, sin(p)],
                        [0,        1,      0],
                        [-sin(p),  0, cos(p)]])

        ROT_Z = Matrix([[cos(y), -sin(y), 0],
                        [sin(y),  cos(y), 0],
                        [     0,       0, 1]])



        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

	        # Extract end effector position
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            # The message contains orientation as quaternion
            # first extract roll, pitch, yaw
            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])
     
            ### Your IK code here 
    	    # Compensate for rotation discrepancy between DH parameters and Gazebo
    	    #
            ROT_EE = ROT_Z * ROT_Y * ROT_X
            # Apply correction that arises from the fact how the model is oriented in xacro file/gazebo world as compared to our DH params orientation
            ROT_CORRECTION = ROT_Z.subs(y, radians(180)) * ROT_Y.subs(p, radians(-90))
            ROT_EE = ROT_EE * ROT_CORRECTION
            ROT_EE = ROT_EE.subs({'y': yaw, 'r': roll, 'p': pitch})

            EE =  Matrix([[px], [py], [pz]])
 
            WC = EE - (0.303) * ROT_EE[:,2]

    	    # Since we have one of the special cases we can calculate the
            # joint angles using Geometric IK method

            # First we need to get sides of the ABC triangle
            # See writeup for pictures and an outline why
            a_side = 1.501
            c_side = 1.25

            # b side is a bit more challenging
            b_rotated = (sqrt(WC[0] * WC[0] + WC[1] * WC[1]) - 0.35)
            b_in_z = WC[2] - 0.75
            b_side = sqrt(pow(b_rotated, 2) + pow(b_in_z, 2))

            # define cosine sentence equations
            # and get the angles once we have all the sides of the triangle
            angle_a = acos((- a_side * a_side + b_side * b_side + c_side * c_side) / (2 * b_side * c_side))
            angle_b = acos((- b_side * b_side + a_side * a_side + c_side * c_side) / (2 * a_side * c_side))
            angle_c = acos((- c_side * c_side + b_side * b_side + a_side * a_side) / (2 * b_side * a_side))
            
            # JUST FROM PLANE OF THE WC, WE CAN EVALUATE theta 1
            theta1 = atan2(WC[1], WC[0])
            theta2 = pi / 2 - angle_a - atan2(b_in_z, b_rotated)
            theta3 = pi / 2 - (angle_b + 0.036)
            
            # Now we need to compute theta 4-6
            # bascially we want to know what the rotation is at point 3 (wrist)
            # and we can extract angles from that matrix
            # We can achieve that by applying premultiplication both sides of
            # of equation R0_6 = Rrpy by inv(R0_3)
            # see here for details 
            # https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/a1abb738-84ee-48b1-82d7-ace881b5aec0
            R0_3 = T0_1[0:3,0:3] * T1_2[0:3,0:3] * T2_3[0:3,0:3]
            R0_3 = R0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})

            # LU does not seem to work
            #R3_6 = R0_3.inv("LU") * ROT_EE
            # transpose does. We can use it since this should be symetric matrix
            R3_6 = Transpose(R0_3) * ROT_EE
            
            # now we can just extract the angles from the matric
            # See the writeup for details why these equations look like they do
            theta4 = atan2(R3_6[2,2], -R3_6[0,2])
            theta5 = atan2(sqrt(R3_6[0,2]*R3_6[0,2] + R3_6[2,2]*R3_6[2,2]), R3_6[1,2])
            # Not sure why the sign is flipped here but it seems to perform better
            # Does not make the rotation during "pickup phase"
            # theta6 = atan2(R3_6[1,1], -R3_6[1,0])
            theta6 = atan2(-R3_6[1,1], R3_6[1,0])
            
            ###
            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
	    joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
	    joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()

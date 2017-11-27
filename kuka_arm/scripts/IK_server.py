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
        # The matrix is 4x4 since it is capturing both rotation and translation
        def tf_matrix_from_dh_params(alpha, a, d, q):
            return Matrix([[cos(q),            -sin(q),            0,              a],
                           [ sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                           [ sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
                           [                 0,                 0,           0,             1]])


        # define DH Parameter values
        # This table is verbatim copy of the table in writeup.
        # There it is also described how I arrived at the values
        dh_parameters = {   
            alpha0: 0,      a0:   0,      d1: 0.75,  q1: q1,
            alpha1: -pi/2,  a1:   0.35,   d2: 0,     q2: -pi/2 + q2,
            alpha2: 0,      a2:   1.25,   d3: 0,     q3: q3,
            alpha3: -pi/2,  a3:   -0.054, d4: 1.50,  q4: q4,
            alpha4: pi/2,   a4:   0,      d5: 0,     q5: q5,
            alpha5: -pi/2,  a5:   0,      d6: 0,     q6: q6,
            alpha6: 0,      a6:   0,      d7: 0.303, q7: 0,
        }

        # Define all the transformation matrices from i to i+1 joint
        # The robot has 6 joints and base abd end effector.
        # This gives us 7 tranformation matrices in between those points
        # the matric is simply taken by substituting the params into symbolic matric
        # defined above. Advantage is that we can manipulate the result and simplify it
        # with sympy simplify. This will come very handy later
        T0_1  = tf_matrix_from_dh_params(alpha0, a0, d1, q1).subs(dh_parameters)
        T1_2  = tf_matrix_from_dh_params(alpha1, a1, d2, q2).subs(dh_parameters)
        T2_3  = tf_matrix_from_dh_params(alpha2, a2, d3, q3).subs(dh_parameters)
        T3_4  = tf_matrix_from_dh_params(alpha3, a3, d4, q4).subs(dh_parameters)
        T4_5  = tf_matrix_from_dh_params(alpha4, a4, d5, q5).subs(dh_parameters)
        T5_6  = tf_matrix_from_dh_params(alpha5, a5, d6, q6).subs(dh_parameters)
        T6_EE = tf_matrix_from_dh_params(alpha6, a6, d7, q7).subs(dh_parameters)

        # Define transformation from base to end effector
        # This gives us a way how to transfer from the base to the
        # EE in one matrix operation
        T0_EE = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_EE

        # Define rotation matrices with symbols
        # This is just a definition of rotation matrix copied from wikipedia
        # Nothing new
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
        # The code seems to pass multiple poses in one message.
        # Let's iterate over them and collect the results into an array which
        # will be sent back to ROS
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
            # Apply correction that arises from the fact how the model is oriented in
            # xacro file/gazebo world as compared to our DH params orientation
            # to be honest I am not exactly sure why the rotation here has the sign it
            # has. To my understanding the discrepancy arises from the fact that in our model we assume that the arm is oriented along the x axis but the model in the world is actually rotated.
            # Initially I thought the problem was different world coordinates in gazebo vs Ros but that does not seem to be the case.
            ROT_CORRECTION = ROT_Z.subs(y, radians(180)) * ROT_Y.subs(p, radians(-90))
            ROT_EE = ROT_EE * ROT_CORRECTION
            ROT_EE = ROT_EE.subs({'y': yaw, 'r': roll, 'p': pitch})

            EE =  Matrix([[px], [py], [pz]])
 
 
            # what we try to do here is this.
            # We know where our end effector is.
            # We want to get to the wrist center since that will allow us to solve for theta1-3
            # We know what the "length" of EE is so we need to move  by that distance
            # The problem is we do not necessarily know what the direction of that is
            # Since we defined the kinematic chain as we did
            # translation from WC to EE is along the Z axis between them
            # The Z axis of the EE fram can be taken as ROT_EE[:,2]
            WC = EE - (0.303) * ROT_EE[:,2]

    	    # Since we have one of the special cases we can calculate the
            # joint angles using Geometric IK method
            
            # First we need to get sides of the ABC triangle
            # This is defined between joins 2, 3 and 4
            # See writeup for pictures and an outline why
            
            # side a is taken by
            # sqrt(1.5^2 + (-0.054)^2)
            # See the definition of link between 3-4. Cause by a weird shape of the arm
            a_side = 1.501
            # Taken directly from the link defintion 2-3
            c_side = 1.25

            # b side is a bit more challenging
            # See writeup for details
            # Bascally we need to project the 0-WC to x-y plane.
            # the 0.35 is the correction of the fact that base is not under joint 2 directly but 0.35 away.
            # we subtract  after sqrt because does not matter which way we rotate it is always 0.35 (draw a picture it is clearer from that)
            b_rotated = (sqrt(WC[0] * WC[0] + WC[1] * WC[1]) - 0.35)
            # This is similar but here we can get the z axis directly from z position
            # again corrected by the 0-2 distance
            b_in_z = WC[2] - 0.75
            # Now we do pythagoreas and we have b
            b_side = sqrt(pow(b_rotated, 2) + pow(b_in_z, 2))

            # We have all sides, we can get all angles
            # define cosine sentence equations
            # and get the angles once we have all the sides of the triangle
            angle_a = acos((- a_side * a_side + b_side * b_side + c_side * c_side) / (2 * b_side * c_side))
            angle_b = acos((- b_side * b_side + a_side * a_side + c_side * c_side) / (2 * a_side * c_side))
            angle_c = acos((- c_side * c_side + b_side * b_side + a_side * a_side) / (2 * b_side * a_side))
            
            # Since we know ABC, and alpha beta gamma we can try to tackle the thetas
            # JUST FROM PLANE OF THE WC, WE CAN EVALUATE theta 1
            # this is the rotation along the z axis
            theta1 = atan2(WC[1], WC[0])
            
            # Theta 2 can be computed from
            # theta2 + alpha + the_angle_that_completes_90_degrees
            # that last part can be computed from things we computed before
            # WC z height and projection to x-y plane of 0-WC (both corrected)
            # we have the numbers just applay a different trig function to get it
            theta2 = pi / 2 - angle_a - atan2(b_in_z, b_rotated)
            
            # this is a bit more convoluted
            # see the picture in the writeup
            # the basic idea is to realize
            # that beta can be flipped on the other side
            # beta_flipped = 180 - beta
            # beta_flipped - theta should form 90
            # You can play around with the sign of theta depending what you want
            # better to draw couple of pictures in different positions
            # Important to realize is that theta is 0 when link 2-3 and 3-4 are orthogonal
            # The number + 0.036 is a correction term that comes form the same fact as 1.501 above
            # math.atan2(0.054, 1.5)
            theta3 = pi / 2 - (angle_b + 0.036)
            
            # Now we need to compute theta 4-6
            # bascially we want to know what the rotation is at point 3 (wrist)
            # and we can extract angles from that matrix
            # We can achieve that by applying premultiplication both sides of
            # of equation R0_6 = Rrpy by inv(R0_3)
            # see here for details 
            # https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/a1abb738-84ee-48b1-82d7-ace881b5aec0
            # The T0_1[0:3,0:3] is to cut the rotational part out of homogeneous transform ignoring the translation
            R0_3 = T0_1[0:3,0:3] * T1_2[0:3,0:3] * T2_3[0:3,0:3]
            R0_3 = R0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})

            # LU does not seem to work
            #R3_6 = R0_3.inv("LU") * ROT_EE
            # transpose does. We can use it since this should be symetric matrix
            R3_6 = Transpose(R0_3) * ROT_EE
            
            # now we can just extract the angles from the matrix
            # See the writeup for details why these equations look like they do
            #
            # the key things to remember
            # tg(x) = sin(x)/cos(x)
            #
            # cos(x) = sqrt(cos^2(x)sin^2(y) + cos^2(x)cos^2(y))
            #        = sqrt(cos^2(x) * (sin^2(y) + cos^2(y))
            #
            # using substitution (sin^2(y) + cos^2(y)) = 1
            #
            #        = sqrt(cos^2(x) * 1)
            #        = cos(x)
            #
            #     and similar to the above can write
            #
            #     tg(2) = sin(2)/ sqrt(cos^2(x)sin^2(y) + cos^2(x)cos^2(y))
            #
            # also when you are given a rotational matrix with numbers does
            # not matter how you interpret it. You have to decide how you want to
            # get to that rotation. The elements you use for extracting angles depend
            # on the choice of that matrix though
            # In our case it is R3_EE
            # YOU can visualize it by using this snippet
            #
            # T3_EE = T3_4 * T4_5 * T5_6 * T6_EE
            # turn into rotational
            # R3_EE = T3_EE[0:3, 0:3]
            # pprint(simplify(R3_EE))
            # From there just use the tricks above to extract the angles
            theta4 = atan2(R3_6[2,2], -R3_6[0,2])
            theta5 = atan2(sqrt(R3_6[0,2]*R3_6[0,2] + R3_6[2,2]*R3_6[2,2]), R3_6[1,2])
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

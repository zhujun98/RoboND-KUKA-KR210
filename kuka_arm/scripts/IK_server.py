#!/usr/bin/env python
"""

"""
from __future__ import print_function
import numpy as np

import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose


# define constants
D1 = 0.75
A1 = 0.35
A2 = 1.25
D4 = 1.50
A3 = 0.054
DG = 0.303 + 0.15  # from joint 6 to the center of the gripper finger


def homogeneous_transform(alpha, a, d, theta):
    """Homogeneous transformation matrix using DH parameters"""
    return np.matrix([[np.cos(theta), -np.sin(theta), 0, a],
                      [np.sin(theta)*np.cos(alpha), np.cos(theta)*np.cos(alpha),
                       -np.sin(alpha), -np.sin(alpha)*d],
                      [np.sin(theta)*np.sin(alpha), np.cos(theta)*np.sin(alpha),
                       np.cos(alpha),  np.cos(alpha)*d],
                      [0, 0, 0, 1]])


def rotation_y(theta):
    """Matrix of rotation about y axis"""
    return np.matrix([[ np.cos(theta), 0,  np.sin(theta), 0],
                      [             0, 1,              0, 0],
                      [-np.sin(theta), 0,  np.cos(theta), 0],
                      [             0, 0,              0, 1]])


def rotation_x(theta):
    """Matrix of rotation about x axis"""
    return np.matrix([[ 1,             0,              0, 0],
                      [ 0, np.cos(theta), -np.sin(theta), 0],
                      [ 0, np.sin(theta),  np.cos(theta), 0],
                      [ 0,             0,              0, 1]])


def rotation_z(theta):
    """Matrix of rotation about z axis"""
    return np.matrix([[ np.cos(theta), -np.sin(theta), 0, 0],
                      [ np.sin(theta),  np.cos(theta), 0, 0],
                      [             0,              0, 1, 0],
                      [             0,              0, 0, 1]])


def xyz_fixed_angle(gamma, beta, alpha):
    """Rotation matrix of a X-Y-Z fix angle transform"""
    return rotation_z(alpha)*rotation_y(beta)*rotation_x(gamma)


def get_dh_parameters(theta1=0., theta2=0., theta3=0., theta4=0., theta5=0., theta6=0.):
    """Form the DH table

    @params theta1, theta2, theta3, theta4, theta5, theta6: float
        Angles defined in the DH table.

    @return: numpy.ndarray
        DH table.
    """
    # alpha, a, d, theta
    return [[0, 0, D1, theta1],
            [-np.pi / 2, A1, 0, theta2 - np.pi / 2],
            [0, A2, 0, theta3],
            [-np.pi / 2, -A3, D4, theta4],
            [np.pi / 2, 0, 0, theta5],
            [-np.pi / 2, 0, 0, theta6],
            [0, 0, DG, 0]]


def trans_base_link1(theta1):
    """Homogeneous transform matrix from base to link 1"""
    dh_params = get_dh_parameters(theta1)

    return homogeneous_transform(
        dh_params[0][0], dh_params[0][1], dh_params[0][2], dh_params[0][3])


def trans_base_link2(theta1, theta2):
    """Homogeneous transform matrix from base to link 2"""
    dh_params = get_dh_parameters(theta1, theta2)

    return np.matmul(trans_base_link1(theta1), homogeneous_transform(
        dh_params[1][0], dh_params[1][1], dh_params[1][2], dh_params[1][3]))


def trans_base_link3(theta1, theta2, theta3):
    """Homogeneous transform matrix from base to link 3"""
    dh_params = get_dh_parameters(theta1, theta2, theta3)

    return np.matmul(trans_base_link2(theta1, theta2), homogeneous_transform(
        dh_params[2][0], dh_params[2][1], dh_params[2][2], dh_params[2][3]))


def trans_base_link4(theta1, theta2, theta3, theta4):
    """Homogeneous transform matrix from base to link 4"""
    dh_params = get_dh_parameters(theta1, theta2, theta3, theta4)

    return np.matmul(trans_base_link3(theta1, theta2, theta3),
                     homogeneous_transform(dh_params[3][0], dh_params[3][1],
                                           dh_params[3][2], dh_params[3][3]))


def trans_base_link5(theta1, theta2, theta3, theta4, theta5):
    """Homogeneous transform matrix from base to link 5"""
    dh_params = get_dh_parameters(theta1, theta2, theta3, theta4, theta5)

    return np.matmul(trans_base_link4(theta1, theta2, theta3, theta4),
                     homogeneous_transform(dh_params[4][0], dh_params[4][1],
                                           dh_params[4][2], dh_params[4][3]))


def trans_base_link6(theta1, theta2, theta3, theta4, theta5, theta6):
    """Homogeneous transform matrix from base to link 6"""
    dh_params = get_dh_parameters(theta1, theta2, theta3, theta4, theta5, theta6)

    return np.matmul(trans_base_link5(theta1, theta2, theta3, theta4, theta5),
                     homogeneous_transform(dh_params[5][0], dh_params[5][1],
                                           dh_params[5][2], dh_params[5][3]))


def _trans_base_linkg(theta1, theta2, theta3, theta4, theta5, theta6):
    """Homogeneous transform matrix from base to gripper link"""

    dh_params = get_dh_parameters(theta1, theta2, theta3, theta4, theta5, theta6)

    return np.matmul(trans_base_link6(theta1, theta2, theta3, theta4, theta5, theta6),
                     homogeneous_transform(dh_params[6][0], dh_params[6][1],
                                           dh_params[6][2], dh_params[6][3]))


def trans_base_linkg(theta1, theta2, theta3, theta4, theta5, theta6):
    """Homogeneous transform matrix from base to gripper link with correction

    Transfrom from the DH coordinate system to the global coordinate
    system defined in the urdf file.
    """
    return np.matmul(np.matmul(
        _trans_base_linkg(theta1, theta2, theta3, theta4, theta5, theta6),
        rotation_z(np.pi)), rotation_y(-np.pi / 2))


def check_solution(theta2, theta3, psi, px, py, pz):
    """Check the correctness of theta2 and theta3"""
    l1 = A2*np.sin(theta2) + np.sqrt(D4**2 + A3**2)*np.cos(theta2 + theta3 + psi)
    r1 = np.sqrt(px**2 + py**2) - A1
    l2 = A2*np.cos(theta2) - np.sqrt(D4**2 + A3**2)*np.sin(theta2 + theta3 + psi)
    r2 = pz - D1
    if abs(l1 - r1) < 1e-6 and abs(l2 - r2) < 1e-6:
        return True
    else:
        return False


def solve_position_kinematics(px_wc, py_wc, pz_wc):
    """Solve the position kinematics for the first three joints

    @param px_wc, py_wc, pz_wc: float
        Coordinates of the spherical wrist center (link5)
        Note: in practical case, these coordinates should be derived
        from the position and orientation of the end effector.

    @return theta1, theta2, theta3: float
        Theta angles in the DH coordinate systems for the first
        three joints.
    """
    pr_wc = np.sqrt(px_wc ** 2 + py_wc ** 2)

    ######################################################
    # Calculate joint angles using Geometric IK method
    ######################################################

    # calculate the twist angles of the first three joints
    theta1 = np.arctan2(py_wc, px_wc)

    #
    # theta2 and theta3 are the solution of the following two equations:
    #
    # A2*sin(theta2) + (D4**2 + A3**2)^0.5*cos(theta2 + theta3 + psi)
    # = sqrt(px_wc^2 + py_wc^2) - A1
    # A2*cos(theta2) - (D4**2 + A3**2)^0.5*sin(theta2 + theta3 + psi)
    # = pz_wc - D1
    #
    psi = np.arctan2(A3, D4)
    R4 = np.sqrt(D4 ** 2 + A3 ** 2)

    sin_theta3_plus_psi = (A2 ** 2 + R4 ** 2 - (pr_wc - A1) ** 2 -
                           (pz_wc - D1) ** 2) / (2 * A2 * R4)
    cos_theta3_plus_psi = np.sqrt(1 - sin_theta3_plus_psi ** 2)
    possible_theta3 = np.array([np.arctan2(sin_theta3_plus_psi,
                                           cos_theta3_plus_psi) - psi,
                                np.arctan2(sin_theta3_plus_psi,
                                           -cos_theta3_plus_psi) - psi])

    sin_theta2 = ((pr_wc - A1)*(A2 - R4*np.sin(possible_theta3 + psi)) -
                  (pz_wc - D1)*R4*np.cos(possible_theta3 + psi)) / \
                 ((A2 - R4*np.sin(possible_theta3 + psi))**2 +
                  (R4*np.cos(possible_theta3 + psi))**2)
    cos_theta2 = np.sqrt(1 - sin_theta2**2)
    possible_theta2 = np.concatenate([np.arctan2(sin_theta2, cos_theta2),
                                      np.arctan2(sin_theta2, -cos_theta2)])

    # There are four possible solutions
    possible_solution = ((possible_theta2[0], possible_theta3[0]),
                         (possible_theta2[1], possible_theta3[1]),
                         (possible_theta2[2], possible_theta3[0]),
                         (possible_theta2[3], possible_theta3[1]))

    # Only two of them are the true solutions
    solution = []
    for slt in possible_solution:
        theta2 = slt[0]
        theta3 = slt[1]
        if check_solution(theta2, theta3, psi, px_wc, py_wc, pz_wc) is True:
            solution.append(slt)

    if len(solution) == 0:
        print("Failed to find a solution!")
        return theta1, 0., 0.
    else:
        assert (len(solution) == 2)  # should be two solutions

        # We will choose the solution with smaller rotation angles to
        # avoid reaching limitations of joints.
        if solution[0][0] ** 2 + solution[0][1] ** 2 < \
                solution[1][0] ** 2 + solution[1][1] ** 2:
            return theta1, solution[0][0], solution[0][1]
        else:
            return theta1, solution[1][0], solution[1][1]


def handle_calculate_IK(req):
    """"""
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print("No valid poses received")
        return -1
    else:
        # Initialize service response
        joint_trajectory_list = []
        for i in range(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

            # Extract end-effector position and orientation from request
            px = req.poses[i].position.x
            py = req.poses[i].position.y
            pz = req.poses[i].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[i].orientation.x, req.poses[i].orientation.y,
                    req.poses[i].orientation.z, req.poses[i].orientation.w])

            # Rotation matrix from the base to the end-effector
            trans0_G = xyz_fixed_angle(roll, pitch, yaw)[:3, :3]

            # coordinates of the spherical wrist center
            dp = DG * np.matmul(trans0_G, np.array([1, 0, 0]))
            px_wc = px - dp[0, 0]
            py_wc = py - dp[0, 1]
            pz_wc = pz - dp[0, 2]

            # obtain thetas for the first three joints
            tht1, tht2, tht3 = solve_position_kinematics(px_wc, py_wc, pz_wc)
            trans0_3 = trans_base_link3(tht1, tht2, tht3)[:3, :3]

            # The inverse matrix of R0_3 is its transpose matrix
            trans3_G = np.matmul(trans0_3.T, trans0_G)

            # r11 = -sin(theta5)*cos(theta4)
            # r31 = sin(theta5)*sin(theta4)
            tht4 = np.arctan2(trans3_G[2, 0], -trans3_G[0, 0])

            # r22 = sin(theta5)*sin(theta6)
            # r23 = sin(theta5)*cos(theta6)
            tht6 = np.arctan2(trans3_G[1, 1], trans3_G[1, 2])

            # r12 = cos(theta5)
            # There are two possible solutions for theta5, but only one of them is correct
            tht5 = np.arctan2(np.sqrt(1 - trans3_G[1, 0] ** 2), trans3_G[1, 0])
            if np.abs(np.sin(tht5) * np.sin(tht6) - trans3_G[1, 1]) < 1e-6 and \
                            np.abs(np.sin(tht5) * np.cos(tht6) - trans3_G[1, 2]) < 1e-6:
                pass
            else:
                tht5 = np.arctan2(-np.sqrt(1 - trans3_G[1, 0] ** 2), trans3_G[1, 0])
                if np.abs(np.sin(tht5) * np.sin(tht6) - trans3_G[1, 1]) < 1e-6 and \
                                np.abs(np.sin(tht5) * np.cos(tht6) - trans3_G[1, 2]) < 1e-6:
                    pass
                else:
                    print("Warning: Not found! Solution for theta5!")

            # print("The solution is: \ntheta1 = {}, theta2 = {}, theta3 = {}\n"
            #       "theta4 = {}, theta5 = {}, theta6 = {}".
            #       format(tht1, tht2, tht3, tht4, tht5, tht6))

            prediction = trans_base_linkg(tht1, tht2, tht3, tht4, tht5, tht6)
            print("px error: {:.4e}, py error {:.4e}, pz error {:.4e}".
                  format(prediction[0, 3] - px, prediction[1, 3] - py, prediction[2, 3] - pz))

            # Populate response for the IK request
            joint_trajectory_point.positions = [
                tht1, tht2, tht3, tht4, tht5, tht6]
            joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % 
                      len(joint_trajectory_list))

        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    # creates a CalculateIK type service with the name calculate_ik
    rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print("Ready to receive an IK request")
    rospy.spin()


if __name__ == "__main__":
    IK_server()

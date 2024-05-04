#!/usr/bin/env python 
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from math import radians
import matplotlib.pyplot as plt
import numpy as np

class MoveGroup(object):
    def __init__(self) -> None:
        super(MoveGroup, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_group_node', anonymous=True)
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        group_name = 'arm'
        group = moveit_commander.MoveGroupCommander(group_name)

        display_trajectory_publishr = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size= 20)
        planning_Frame = group.get_planning_frame()

        print("============ Planning frame: %s" % planning_Frame)

        # We can also print the name of the end-effector link for this group:
        eef_link = group.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        print("============ Available Planning Groups:", robot.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print("============ Printing robot state")
        print(robot.get_current_state())
        print("")

        self.box_name = ''
        self.robot = robot
        self.scene = scene
        self.group = group
        self.display_trajectory_publisher = display_trajectory_publishr
        self.planning_frame = planning_Frame
        self.eef_link = 'link4_1'
        self.group_names = group_names
    
    
    def rad(self, rpy):
        rpyrad = [radians(rpy[0]), radians(rpy[1]), radians(rpy[2])]
        return rpyrad

    def moveto_xyzrpy(self, xyzrpy):
        xyzrpy_list = list(xyzrpy)
    
    # Set pose target with the converted list
        self.group.set_pose_target(xyzrpy_list, end_effector_link=self.eef_link)
        plan = self.group.go(wait=True)
        self.group.stop()
        self.group.clear_pose_targets()
        current_pose = self.group.get_current_pose().pose
        return current_pose.position.x, current_pose.position.y, current_pose.position.z

def generate_curve_points():
    # Define a parameter range
    num_points = 20 
    t = np.linspace(1, np.pi*2, num_points)

    # Define parametric equations for x, y, z coordinates
    x = 0.4 * np.abs(np.sin(t))  # Ensure x is non-negative
    y = 0.4 * np.cos(t)  # Scale y to be under 0.4
    z = (0.4 / (2 * np.pi)) * t  # Scale z to be under 0.4


    # Return a list of (x, y, z) points
    return list(zip(x, y, z))
def main():
    try:
        armDriver = MoveGroup()
        rpy = armDriver.rad([0, 0, 0])


        # Generate curve points
        curve_points = generate_curve_points()
        x_traj, y_traj, z_traj = [], [], []
        
        for i in curve_points:
            print(i)
            xyzrpy = tuple(i) + tuple(rpy)
            x, y, z = armDriver.moveto_xyzrpy(xyzrpy)
            x_traj.append(x)
            y_traj.append(y)
            z_traj.append(z)
            print("---------------completed---------------")

        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.plot(x_traj, y_traj, z_traj, marker='o', linestyle='-')
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        plt.show()

    except rospy.ROSInterruptException:
        return
    # except KeyboardInterrupt:
    #     return

if __name__ == '__main__':
    main()


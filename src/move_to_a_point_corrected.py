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
        self.group.set_pose_target(xyzrpy, end_effector_link = self.eef_link)
        plan = self.group.go(wait = True)
        self.group.stop()
        self.group.clear_pose_targets()
        current_pose = self.group.get_current_pose().pose


def main():
    try:
        armDriver = MoveGroup()
        xyz = [0.4, 0.1, 0.4]
        rpy = armDriver.rad([0, 0, 0])
        xyzrpy = xyz + rpy
        armDriver.moveto_xyzrpy(xyzrpy)
        print("---------------completed---------------")
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__ == '__main__':
    main()


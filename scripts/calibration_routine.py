# Copyright 2025 Alex Arbogast
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import sys
import rospy
import moveit_commander
import numpy as np

from sensor_msgs.msg import JointState

HYDRA_GROUP = "hydra_planning_group"
ROB1_GROUP = "rob1_planning_group"
ROB2_GROUP = "rob2_planning_group"


class CalibrationRoutine(object):
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("hydra_calibration_routine")

        self.hydra_group = moveit_commander.MoveGroupCommander(HYDRA_GROUP)
        self.rob1_group = moveit_commander.MoveGroupCommander(ROB1_GROUP)
        self.rob2_group = moveit_commander.MoveGroupCommander(ROB2_GROUP)

        self.rob1_state = JointState()
        self.rob1_state.name = self.rob1_group.get_joints()[:-1]

        self.rob2_state = JointState()
        self.rob2_state.name = self.rob2_group.get_joints()[:-1]

        self.set_vel_scaling_factor(0.5)
        self.set_acc_scaling_factor(0.1)

        # initialize planning pipelines
        self.rob1_group.set_planning_pipeline_id("pilz_industrial_motion_planner")
        self.rob2_group.set_planning_pipeline_id("pilz_industrial_motion_planner")
        self.rob1_group.set_planner_id("PTP")
        self.rob2_group.set_planner_id("PTP")

        self.hydra_group.set_planning_pipeline_id("ompl")
        self.hydra_group.set_planner_id("RRTConnect")

    def exec(self):
        self.rob1_routine()
        self.wait_for_input("Press input to move to rob2 routine")
        self.rob2_routine()

        self.wait_for_input("Press input to move home")
        self.hydra_group.set_named_target("zeros")
        self.execute_plan(self.hydra_group)

    def rob1_routine(self):
        rospy.loginfo("Running rob1 calibration cube")
        self.rob2_state.position = [0.0, 0.0, -np.pi / 2, 0.0, 0.0, 0.0]
        self.rob1_state.position = [0, 0, 0, 0, 0, 0]

        self.hydra_group.set_joint_value_target(self.rob2_state)
        self.hydra_group.set_joint_value_target(self.rob1_state)
        self.execute_plan(self.hydra_group)

        self.cube_routine(self.rob1_group, "rob1_flange", [-0.5, 0.5, 0.5, 0.5])
        rospy.loginfo("Finished rob1 calibration cube")

    def rob2_routine(self):
        rospy.loginfo("Running rob2 calibration cube")
        self.rob1_state.position = [0.0, 0.0, -np.pi / 2, 0.0, 0.0, 0.0]
        self.rob2_state.position = [0, 0, 0, 0, 0, 0]

        self.hydra_group.set_joint_value_target(self.rob1_state)
        self.hydra_group.set_joint_value_target(self.rob2_state)
        self.execute_plan(self.hydra_group)

        self.cube_routine(self.rob2_group, "rob2_flange", [-0.5, 0.5, 0.5, 0.5])
        rospy.loginfo("Finished rob2 clibration cube")

    def cube_routine(self, group, eef, q):
        l, w, h = 0.2, 0.2, 0.2
        l2, w2, h2 = l / 2, w / 2, h / 2
        points = np.array(
            [
                [l2, w2, h2],
                [l2, -w2, h2],
                [-l2, -w2, h2],
                [-l2, w2, h2],
                [-l2, w2, -h2],
                [l2, w2, -h2],
                [l2, -w2, -h2],
                [-l2, -w2, -h2],
            ]
        )
        points += np.array([0.0, 0.0, 0.4])
        group.set_pose_reference_frame("hydra_base")

        for i in range(len(points)):
            pose = list(np.hstack((points[i], q)))
            group.set_pose_target(pose, eef)

            self.wait_for_input("Waiting for input ('x' to cancel)")
            rospy.loginfo(f"Moving to cube point {i + 1}: \npose={pose}\n")
            self.execute_plan(group)

    def set_pose_reference_frame(self, frame):
        self.hydra_group.set_pose_reference_frame(frame)
        self.rob1_group.set_pose_reference_frame(frame)
        self.rob2_group.set_pose_reference_frame(frame)

    def set_vel_scaling_factor(self, vel):
        self.hydra_group.set_max_velocity_scaling_factor(vel)
        self.rob1_group.set_max_velocity_scaling_factor(vel)
        self.rob2_group.set_max_velocity_scaling_factor(vel)

    def set_acc_scaling_factor(self, acc):
        self.hydra_group.set_max_acceleration_scaling_factor(acc)
        self.rob1_group.set_max_acceleration_scaling_factor(acc)
        self.rob2_group.set_max_acceleration_scaling_factor(acc)

    def execute_plan(self, group):
        group.go(wait=True)
        group.stop()

    def wait_for_input(self, msg=None):
        if msg is not None:
            rospy.loginfo(msg)

        x = input()
        if x == "x":
            exit()


def main():
    try:
        routine = CalibrationRoutine()
        routine.exec()

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == "__main__":
    main()

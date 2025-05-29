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

HYDRA_GROUP = "hydra_planning_group"
ROB1_GROUP = "rob1_planning_group"
ROB2_GROUP = "rob2_planning_group"


def wait_for_input(msg=None):
    if msg is not None:
        rospy.loginfo(msg)

    x = input()
    if x == "x":
        exit()


class CheckCalibration(object):
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("hydra_calibration_routine")

        self.hydra_group = moveit_commander.MoveGroupCommander(HYDRA_GROUP)

        self.hydra_group.set_max_velocity_scaling_factor(0.05)
        self.hydra_group.set_max_acceleration_scaling_factor(0.1)

        # initialize planning pipelines
        self.hydra_group.set_planning_pipeline_id("ompl")
        self.hydra_group.set_planner_id("RRTConnect")

    def exec(self):
        self.wait_for_input("Press enter to move to home")
        self.hydra_group.set_named_target("zeros")
        self.execute_plan(self.hydra_group)

        self.hydra_group.set_pose_reference_frame("work_object")
        h = 0.1

        self.wait_for_input("Press enter to move to first point ('x' to cancel)")
        pose1 = [0, 0.25, h, 0, 0, -0.7071068, 0.7071068]
        pose2 = [0, -0.25, h, 0, 0, 0.7071068, 0.7071068]
        print(f"Moving to points:\n{pose1}\n{pose2}")
        self.hydra_group.set_pose_target(pose1, "rob1_flange")
        self.hydra_group.set_pose_target(pose2, "rob2_flange")
        self.execute_plan(self.hydra_group)

        self.wait_for_input("Press enter to move to first point ('x' to cancel)")
        pose1 = [0, 0.02, h, 0, 0, -0.7071068, 0.7071068]
        pose2 = [0, -0.02, h, 0, 0, 0.7071068, 0.7071068]
        print(f"Moving to points:\n{pose1}\n{pose2}")
        self.hydra_group.set_pose_target(pose1, "rob1_flange")
        self.hydra_group.set_pose_target(pose2, "rob2_flange")
        self.execute_plan(self.hydra_group)

        self.wait_for_input("Press enter to move to first point ('x' to cancel)")
        self.hydra_group.set_max_velocity_scaling_factor(0.005)
        pose1 = [0, 0.005, h, 0, 0, -0.7071068, 0.7071068]
        pose2 = [0, -0.005, h, 0, 0, 0.7071068, 0.7071068]
        print(f"Moving to points:\n{pose1}\n{pose2}")
        self.hydra_group.set_pose_target(pose1, "rob1_flange")
        self.hydra_group.set_pose_target(pose2, "rob2_flange")
        self.execute_plan(self.hydra_group)

        self.wait_for_input("Press enter to move to first point ('x' to cancel)")
        self.hydra_group.set_max_velocity_scaling_factor(0.001)
        pose1 = [0, 0.001, h, 0, 0, -0.7071068, 0.7071068]
        pose2 = [0, -0.001, h, 0, 0, 0.7071068, 0.7071068]
        print(f"Moving to points:\n{pose1}\n{pose2}")
        self.hydra_group.set_pose_target(pose1, "rob1_flange")
        self.hydra_group.set_pose_target(pose2, "rob2_flange")
        self.execute_plan(self.hydra_group)

        self.wait_for_input("Press enter to move to first point ('x' to cancel)")
        self.hydra_group.set_max_velocity_scaling_factor(0.005)
        pose1 = [0, 0.02, h, 0, 0, -0.7071068, 0.7071068]
        pose2 = [0, -0.02, h, 0, 0, 0.7071068, 0.7071068]
        print(f"Moving to points:\n{pose1}\n{pose2}")
        self.hydra_group.set_pose_target(pose1, "rob1_flange")
        self.hydra_group.set_pose_target(pose2, "rob2_flange")
        self.execute_plan(self.hydra_group)

        self.wait_for_input("Press enter to move to first point ('x' to cancel)")
        self.hydra_group.set_max_velocity_scaling_factor(0.05)
        pose1 = [0, 0.25, h, 0, 0, -0.7071068, 0.7071068]
        pose2 = [0, -0.25, h, 0, 0, 0.7071068, 0.7071068]
        print(f"Moving to points:\n{pose1}\n{pose2}")
        self.hydra_group.set_pose_target(pose1, "rob1_flange")
        self.hydra_group.set_pose_target(pose2, "rob2_flange")
        self.execute_plan(self.hydra_group)

        self.wait_for_input("Press input to move home")
        self.hydra_group.set_named_target("zeros")
        self.execute_plan(self.hydra_group)

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
        app = CheckCalibration()
        app.exec()

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == "__main__":
    main()

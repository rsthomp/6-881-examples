import argparse
import numpy as np

from pydrake.common import FindResourceOrThrow

from plan_runner.manipulation_station_simulator import ManipulationStationSimulator
from plan_runner.manipulation_station_plan_runner import *
from plan_runner.open_left_door import (GenerateOpenLeftDoorPlansByTrajectory,
                                        GenerateOpenLeftDoorPlansByImpedanceOrPosition,)

from plan_runner.open_left_door import GenerateApproachHandlePlans
from plan_runner.plan_utils import ConnectPointsWithCubicPolynomial

if __name__ == '__main__':
    # Construct simulator system.
    object_file_path = FindResourceOrThrow(
        "drake/examples/manipulation_station/models/061_foam_brick.sdf")

    manip_station_sim = ManipulationStationSimulator(
        time_step=2e-3,
        object_file_path=object_file_path,
        object_base_link_name="base_link",)

    # Generate plans.
    def ReturnConstantOrientation(i, num_knot_points):
        return R_WEa_ref

    plan_list, gripper_setpoint_list, _ = GenerateApproachHandlePlans(ReturnConstantOrientation)


    xyz_traj = ConnectPointsWithCubicPolynomial([0, 0, 0], [0, 0, 0.1], 10)
    plan_list.append(IiwaTaskSpacePlan(10, xyz_traj, R_WEa_ref, p_EQ))
    gripper_setpoint_list = [0.02, 0.02, 0.02]

    # Run simulator
    q0 = [0, 0, 0, -1.75, 0, 1.0, 0]
    iiwa_position_command_log, iiwa_position_measured_log, iiwa_external_torque_log, \
        state_log = manip_station_sim.RunSimulation(
            plan_list, gripper_setpoint_list, extra_time=2.0, real_time_rate=0.0, q0_kuka=q0,)
    PlotExternalTorqueLog(iiwa_external_torque_log)
    PlotIiwaPositionLog(iiwa_position_command_log, iiwa_position_measured_log)

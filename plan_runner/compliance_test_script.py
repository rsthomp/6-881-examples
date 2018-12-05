import numpy as np
import meshcat
import meshcat.geometry as g
                                  
from manipulation_station_simulator import ManipulationStationSimulator
                                            
from pydrake.common import FindResourceOrThrow
from robot_plans import *
from grab_object_plans import *

vis = meshcat.Visualizer()

# Construct a ManipulationStation
object_file_path = FindResourceOrThrow(
        "drake/examples/manipulation_station/models/061_foam_brick.sdf")

manip_station_sim = ManipulationStationSimulator(
    time_step=2e-3,
    object_file_path=object_file_path,
    object_base_link_name="base_link")

# Generate plans.
q0 = [0, 0, 0, -1.75, 0, 1.0, 0]
# plan_list, gripper_setpoint_list = \
#     GenerateIiwaPlansAndGripperSetPoints(manip_station_sim, q0)

#Plan: Move arm to object, using the position controller
p_WQ_home = np.array([0.5, 0, 0.41])
p_WR_home = np.array([0.5, 0.03, 0.41])
p_WL_home = np.array([0.5, -0.03, 0.41])

plan_list = [GrabObjectPositionPlan(p_WQ_home, p_WR_home, p_WL_home, 
                                       manip_station_sim.X_WObject, duration=6.0)]
gripper_setpoint_list = [.07]
#Plan: Grasp object, using a hybrid position/force controller


# Run simulation
iiwa_position_command_log, iiwa_position_measured_log, iiwa_external_torque_log, \
        plant_state_log = \
    manip_station_sim.RunSimulation(plan_list, gripper_setpoint_list,
                                extra_time=2.0, real_time_rate=1.0, q0_kuka=q0)
import numpy as np
import meshcat
import meshcat.geometry as g
                                  
from manipulation_station_simulator import ManipulationStationSimulator
                                            
from pydrake.common import FindResourceOrThrow
from robot_plans import *
from grab_object_plans import *

vis = meshcat.Visualizer()

ObjectPaths = {
    "small_brick" : "./plan_runner/thin_brick.sdf",
    "tall_box"    : "./plan_runner/tall_box.sdf"
    "long_tube"   : "./plan_runner/long_tube.sdf"
    "small_ball"  : "./plan_runner/small_ball.sdf"
}

ObjectWidths = {
    "small_brick" : .012,
    "tall_box"    : .012,
    "long_tube"   : .02,
    "small_ball"  : .025,
}

ObjectGraspHeights = {
    "small_brick" : .03,
    "tall_box"    : .17,
    "long_tube"   : .03,
    "small_ball"  : .03,
}
p_WQ_home = np.array([0.5, 0, 0.41])
p_WR_home = np.array([0.5, 0.03, 0.43])
p_WL_home = np.array([0.5, -0.03, 0.43])

if __name__ == '__main__':

    # Construct a ManipulationStation
    object_file_path = "./plan_runner/thin_brick.sdf"

    manip_station_sim = ManipulationStationSimulator(
        time_step=2e-3,
        object_file_path=object_file_path,
        object_base_link_name="base_link")

    # Generate plans.
    q0 = [0, 0, 0, -1.75, 0, 1.0, 0]
    # plan_list, gripper_setpoint_list = \
    #     GenerateIiwaPlansAndGripperSetPoints(manip_station_sim, q0)

    #Plan: Move arm to object, using the position controller

    plan_list = [GrabObjectPositionPlan(p_WQ_home, p_WR_home, p_WL_home, 
                                           manip_station_sim.X_WObject, duration=6.0)]

    plan_list.append(GraspObjectCompliancePlan(plan_list[0], 5.0, max_force=4))

    #Plan: Grasp object, using a hybrid position/force controller
    #plan_list.append(OpenLeftDoorCompliancePlan(0))
    gripper_setpoint_list = [.055, 0.0]#, 0.0]
    # Run simulation
    iiwa_position_command_log, iiwa_position_measured_log, iiwa_external_torque_log, \
            plant_state_log = \
        manip_station_sim.RunSimulation(plan_list, gripper_setpoint_list,
                                    extra_time=2.0, real_time_rate=1.0, q0_kuka=q0)


def BrickGraspTest():
    pass

def BoxGraspTest():
    pass

def TubeGraspTest():
    pass

def BallGraspTest():
    pass

def CabinetTest():
    pass

def BrickPlaceTest():
    pass


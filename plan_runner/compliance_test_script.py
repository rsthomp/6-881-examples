#Skye Thompson
import numpy as np
import meshcat
import meshcat.geometry as g

from pydrake.math import RollPitchYaw             
from manipulation_station_simulator import ManipulationStationSimulator
                                            
from pydrake.common import FindResourceOrThrow
from robot_plans import *
from grab_object_plans import *

vis = meshcat.Visualizer()

#The location of each object file
ObjectPaths = {
    "small_brick" : "./plan_runner/thin_brick.sdf",
    "tall_box"    : "./plan_runner/tall_box.sdf",
    "long_tube"   : "./plan_runner/long_tube.sdf",
    "small_ball"  : "./plan_runner/small_ball.sdf"
}

#Minimum grasp width for each object
ObjectWidths = {
    "small_brick" : .04,
    "tall_box"    : .012,
    "long_tube"   : .025,
    "small_ball"  : .029,
    "cabinet_handle" : .0,
}

#The height at which to grasp the object
#TODO: way of specifying object transform and 
#      grasp pt separately
ObjectGraspHeights = {
    "small_brick" : .03,
    "tall_box"    : .145,
    "long_tube"   : .03,
    "small_ball"  : .03,
}

#Q - the grasp point in the EE frame
#R - model of the right fingertip in the EE frame
#L - model of the left fingertip in the EE frame
p_WQ_home = np.array([0.5, 0, 0.41])
p_WR_home = np.array([0.5, 0.03, 0.43])
p_WL_home = np.array([0.5, -0.03, 0.43])

#Tests grabbing the small brick
def BrickGraspTest():
    rot = np.pi/8
    q0 = [0, 0, 0, -1.75, 0, 1.0, 0]
    X_WObject_small_brick = Isometry3.Identity()
    X_WObject_small_brick.set_translation([.6, .1, ObjectGraspHeights['small_brick']])
    X_WObject_small_brick.set_rotation(RollPitchYaw(0, 0, rot).ToRotationMatrix().matrix())

    manip_station_sim = ManipulationStationSimulator(
        time_step=2e-3,
        object_file_path=ObjectPaths['small_brick'],
        object_base_link_name="base_link",
        X_WObject = X_WObject_small_brick)
    
    plan_list = [GrabObjectPositionPlan(p_WQ_home, p_WR_home, p_WL_home, 
                                           manip_station_sim.X_WObject, duration=6.0)]
    plan_list.append(GraspObjectCompliancePlan(plan_list[0], 5.0, max_force=4, grasp_width=ObjectWidths['small_brick']))

    X_WLift = Isometry3.Identity()
    X_WLift.set_translation([.6, .1, ObjectGraspHeights['tall_box'] + .1])

    new_WQ = X_WObject_small_brick.translation() 
    new_WR = X_WObject_small_brick.translation() + [.03 * np.cos(rot), .03 * np.sin(rot), -0.02]
    new_WL = X_WObject_small_brick.translation() + [-.03 * np.cos(rot), -.03 * np.sin(rot), -0.02]

    plan_list.append(GrabObjectPositionPlan(new_WQ, new_WR, new_WL, 
                                           X_WLift, duration=4.0))

    gripper_setpoint_list = [.055, 0.0, 0.0]

    iiwa_position_command_log, iiwa_position_measured_log, iiwa_external_torque_log,\
            plant_state_log = manip_station_sim.RunSimulation(plan_list, gripper_setpoint_list,
                                    extra_time=2.0, real_time_rate=1.0, q0_kuka=q0)

#Tests grabbing the tall box
def BoxGraspTest():
    rot = 0
    q0 = [0, 0, 0, -1.75, 0, 1.0, 0]
    X_WObject_small_brick = Isometry3.Identity()
    X_WObject_small_brick.set_translation([.6, .1, ObjectGraspHeights['tall_box']])

    manip_station_sim = ManipulationStationSimulator(
        time_step=2e-3,
        object_file_path=ObjectPaths['tall_box'],
        object_base_link_name="base_link",
        X_WObject = X_WObject_small_brick)
    
    plan_list = [GrabObjectPositionPlan(p_WQ_home, p_WR_home, p_WL_home, 
                                           manip_station_sim.X_WObject, duration=6.0)]
    plan_list.append(GraspObjectCompliancePlan(plan_list[0], 5.0, max_force=4, grasp_width=ObjectWidths['tall_box']))

    X_WLift = Isometry3.Identity()
    X_WLift.set_translation([.6, .1, ObjectGraspHeights['tall_box'] + .1])

    new_WQ = X_WObject_small_brick.translation() 
    new_WR = X_WObject_small_brick.translation() + [.03 * np.cos(rot), .03 * np.sin(rot), -0.02]
    new_WL = X_WObject_small_brick.translation() + [-.03 * np.cos(rot), -.03 * np.sin(rot), -0.02]

    plan_list.append(GrabObjectPositionPlan(new_WQ, new_WR, new_WL, 
                                           X_WLift, duration=4.0))

    gripper_setpoint_list = [.055, 0.0, 0.0]

    iiwa_position_command_log, iiwa_position_measured_log, iiwa_external_torque_log, \
            plant_state_log = \
        manip_station_sim.RunSimulation(plan_list, gripper_setpoint_list,
                                    extra_time=2.0, real_time_rate=1.0, q0_kuka=q0)

#Tests grabbing the tube
def TubeGraspTest():
    rot = 0
    q0 = [0, 0, 0, -1.75, 0, 1.0, 0]
    X_WObject_small_brick = Isometry3.Identity()
    X_WObject_small_brick.set_translation([.6, .1, ObjectGraspHeights['long_tube']])
    X_WObject_small_brick.set_rotation(RollPitchYaw(0, 0, 0).ToRotationMatrix().matrix())

    manip_station_sim = ManipulationStationSimulator(
        time_step=2e-3,
        object_file_path=ObjectPaths['long_tube'],
        object_base_link_name="base_link",
        X_WObject = X_WObject_small_brick)
    
    plan_list = [GrabObjectPositionPlan(p_WQ_home, p_WR_home, p_WL_home, 
                                           manip_station_sim.X_WObject, duration=6.0)]
    plan_list.append(GraspObjectCompliancePlan(plan_list[0], 5.0, max_force=4, grasp_width=ObjectWidths['long_tube']))

    X_WLift = Isometry3.Identity()
    X_WLift.set_translation([.6, .1, ObjectGraspHeights['tall_box'] + .1])

    new_WQ = X_WObject_small_brick.translation() 
    new_WR = X_WObject_small_brick.translation() + [.03 * np.cos(rot), .03 * np.sin(rot), -0.02]
    new_WL = X_WObject_small_brick.translation() + [-.03 * np.cos(rot), -.03 * np.sin(rot), -0.02]

    plan_list.append(GrabObjectPositionPlan(new_WQ, new_WR, new_WL, 
                                           X_WLift, duration=4.0))

    gripper_setpoint_list = [.055, 0.0, 0.0]

    iiwa_position_command_log, iiwa_position_measured_log, iiwa_external_torque_log, \
            plant_state_log = \
        manip_station_sim.RunSimulation(plan_list, gripper_setpoint_list,
                                    extra_time=2.0, real_time_rate=1.0, q0_kuka=q0)
#Tests grabbing the ball
def BallGraspTest():
    rot = 0
    q0 = [0, 0, 0, -1.75, 0, 1.0, 0]
    X_WObject_small_brick = Isometry3.Identity()
    X_WObject_small_brick.set_translation([.6, .1, ObjectGraspHeights['small_ball']])

    manip_station_sim = ManipulationStationSimulator(
        time_step=2e-3,
        object_file_path=ObjectPaths['small_ball'],
        object_base_link_name="base_link",
        X_WObject = X_WObject_small_brick)
    
    plan_list = [GrabObjectPositionPlan(p_WQ_home, p_WR_home, p_WL_home, 
                                           manip_station_sim.X_WObject, duration=6.0)]
    plan_list.append(GraspObjectCompliancePlan(plan_list[0], 5.0, max_force=4, grasp_width=ObjectWidths['small_ball']))
    
    X_WLift = Isometry3.Identity()
    X_WLift.set_translation([.6, .1, ObjectGraspHeights['tall_box'] + .1])

    new_WQ = X_WObject_small_brick.translation() 
    new_WR = X_WObject_small_brick.translation() + [.03 * np.cos(rot), .03 * np.sin(rot), -0.02]
    new_WL = X_WObject_small_brick.translation() + [-.03 * np.cos(rot), -.03 * np.sin(rot), -0.02]

    plan_list.append(GrabObjectPositionPlan(new_WQ, new_WR, new_WL, 
                                           X_WLift, duration=4.0))

    gripper_setpoint_list = [.055, 0.0, 0.0]

    iiwa_position_command_log, iiwa_position_measured_log, iiwa_external_torque_log, \
            plant_state_log = \
        manip_station_sim.RunSimulation(plan_list, gripper_setpoint_list,
                                    extra_time=2.0, real_time_rate=1.0, q0_kuka=q0)

#Tests grabbing the cabinet handle and opening the cabinet
def CabinetTest():
    manip_station_sim = ManipulationStationSimulator(
        time_step=2e-3,
        object_file_path=ObjectPaths['small_brick'],
        object_base_link_name="base_link")

    # Generate plans.
    q0 = [0, 0, 0, -1.75, 0, 1.0, 0]
    # plan_list, gripper_setpoint_list = \
    #     GenerateIiwaPlansAndGripperSetPoints(manip_station_sim, q0)

    #Plan: Move arm to object, using the position controller
    plan_list = [GrabObjectPositionPlan(p_WQ_home, p_WR_home, p_WL_home, 
                                           GetHandleFromWorld(0), duration=6.0)]
    plan_list.append(GraspObjectCompliancePlan(plan_list[0], 5.0, max_force=4, grasp_width=ObjectWidths['cabinet_handle']))

    #Plan: Grasp object, using a hybrid position/force controller
    plan_list.append(OpenLeftDoorCompliancePlan(0))
    gripper_setpoint_list = [.055, 0.0, 0.0]
    # Run simulation
    iiwa_position_command_log, iiwa_position_measured_log, iiwa_external_torque_log, \
            plant_state_log = \
        manip_station_sim.RunSimulation(plan_list, gripper_setpoint_list,
                                    extra_time=2.0, real_time_rate=1.0, q0_kuka=q0)


#Uncomment to test, one test at a time
if __name__ == '__main__':
    BrickGraspTest()
    #BallGraspTest()
    #BoxGraspTest()
    #CabinetTest()

    #Nonfunctioning:
    #TubeGraspTest()



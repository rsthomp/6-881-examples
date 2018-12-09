#Skye Thompson
import numpy as np
import meshcat
import meshcat.geometry as g

from pydrake.math import RollPitchYaw, RotationMatrix         
from manipulation_station_simulator import ManipulationStationSimulator
                                            
from pydrake.common import FindResourceOrThrow
from pydrake.common.eigen_geometry import Quaternion
from pyquaternion import Quaternion as PyQuat
from robot_plans import *
from grab_object_plans import *
from open_left_door import *

vis = meshcat.Visualizer()

#The location of each object file
ObjectPaths = {
    "small_brick" : "./plan_runner/thin_brick.sdf",
    "tall_box"    : "./plan_runner/tall_box.sdf",
    "long_tube"   : "./plan_runner/long_tube.sdf",
    "small_ball"  : "./plan_runner/small_ball.sdf",
    "apple" : "../pddl_planning/models/ycb_objects/apple.sdf", 
    "soup_can" : "../pddl_planning/models/ycb_objects/soup_can.sdf",
    "sugar_box": "../pddl_planning/models/ycb_objects/sugar_box.sdf",
    "cracker_box" : "../pddl_planning/models/ycb_objects/cracker_box.sdf", 
}

#Minimum grasp width for each object
ObjectWidths = {
    "small_brick" : .012,
    "tall_box"    : .008,
    "long_tube"   : .025,
    "small_ball"  : .029,
    "cabinet_handle" : .0,
    "apple" : .29, 
    "soup_can" : .012,
    "sugar_box": .012,
    "cracker_box" : .012, 
}

#The height at which to grasp the object
#TODO: way of specifying object transform and 
#      grasp pt separately
ObjectGraspHeights = {
    "small_brick" : .03,
    "tall_box"    : .145,
    "long_tube"   : .03,
    "small_ball"  : .03,
    "apple" : .05, 
    "soup_can" : .07,
    "sugar_box": .13,
    "cracker_box" : .15, 
}

p_WQ_home = np.array([0.5, 0, 0.41])

#Helper method for generating and executing joint space trajectories
def GenerateJointSpacePlan(InterpolateOrientation, goal, start=p_WQ_home, q_full=None):
    
    if q_full is None:
        q_home_full = GetHomeConfiguration(True)
    else:
        q_home_full = q_full

    def InterpolateStraightLine(p_WQ_start, p_WQ_end, num_knot_points, i):
        return (p_WQ_end - p_WQ_start)/num_knot_points*(i+1) + p_WQ_start

    # Generating trajectories
    num_knot_points = 10

    # move to grasp left door handle
    q_traj, q_knots_full = InverseKinPointwise(
        start, goal, duration=5.0,
        num_knot_points=num_knot_points, q_initial_guess=q_home_full,
        InterpolatePosition=InterpolateStraightLine,
        InterpolateOrientation=InterpolateOrientation,
        position_tolerance=0.001)

    q_traj_list = [q_traj]

    plan_list = []
    for q_traj in q_traj_list:
        plan_list.append(JointSpacePlan(q_traj))

    gripper_setpoint_list = [0.1] # robot

    q_final_full = q_knots_full[-1]
    return plan_list, gripper_setpoint_list, q_final_full

#Tests grabbing the cabinet handle and opening the cabinet
def CabinetTest():
    manip_station_sim = ManipulationStationSimulator(
        time_step=2e-3,
        object_file_path=ObjectPaths['small_brick'],
        object_base_link_name="base_link")

    # Generate plans.
    q0 = [0, 0, 0, -1.75, 0, 1.0, 0]

    X_ObjEr = Isometry3.Identity()
    X_ObjEr.set_rotation(RollPitchYaw(0, 5 * np.pi/8, 0).ToRotationMatrix().matrix())
    X_WEr = GetHandleFromWorld(0).multiply(X_ObjEr)

    def ReturnConstantOrientation(i, num_knot_points):
        return RotationMatrix(X_WEr.rotation())

    plan_list, gripper_setpoint_list, q_final_full = GenerateJointSpacePlan(ReturnConstantOrientation, 
                                                                  GetHandleFromWorld(0).translation())
    plan_list.append(GraspObjectCompliancePlan(GetHandleFromWorld(0), 5.0, 
                    max_force=4, grasp_width=ObjectWidths['cabinet_handle']))
    #Plan: Grasp object, using a hybrid position/force controller
    plan_list.append(OpenLeftDoorCompliancePlan(0))
    gripper_setpoint_list = [.04, 0.0, 0.0]
    # Run simulation
    iiwa_position_command_log, iiwa_position_measured_log, iiwa_external_torque_log, \
            plant_state_log = \
        manip_station_sim.RunSimulation(plan_list, gripper_setpoint_list,
                                    extra_time=2.0, real_time_rate=1.0, q0_kuka=q0)

def NewObjectTest(obj_name):
    rot = 0
    q0 = [0, 0, 0, -1.75, 0, 1.0, 0]
    X_WObject = Isometry3.Identity()
    X_WObject.set_translation([.6, .1, ObjectGraspHeights[obj_name]])
    X_WObject.set_rotation(RollPitchYaw(0, 0, rot).ToRotationMatrix().matrix())

    manip_station_sim = ManipulationStationSimulator(
        time_step=2e-3,
        object_file_path=ObjectPaths[obj_name],
        object_base_link_name="base_link",
        X_WObject = X_WObject)
    
    X_ObjEr = Isometry3.Identity()
    X_ObjEr.set_rotation(RollPitchYaw(0, 5 * np.pi/8, 0).ToRotationMatrix().matrix())
    X_WEr = X_WObject.multiply(X_ObjEr)

    
    plant_iiwa = station.get_controller_plant()
    tree_iiwa = plant_iiwa.tree()
    context_iiwa = plant_iiwa.CreateDefaultContext()
    l7_frame = plant_iiwa.GetFrameByName('iiwa_link_7')
    X_WL7 = tree_iiwa.CalcRelativeTransform(
            context_iiwa, frame_A=world_frame,
            frame_B=l7_frame)

    #Interpolates between two orientations
    def ReturnSLERPedOrientation(i, num_knot_points, start=X_WL7, end=X_WEr.rotation()):
        Q_WEr = PyQuat(matrix=RotationMatrix(end).matrix())
        Q_WE = PyQuat(matrix=R_WEa_ref.matrix())
        Q = PyQuat.slerp(Q_WE, Q_WEr, amount=i/float(num_knot_points))

        Q_ret = Quaternion()
        Q_ret.set_wxyz(Q.elements)
        return RotationMatrix(Q_ret)

    #Returns the desired orientation of the gripper relative to the world
    def ReturnConstantOrientation(i, num_knot_points):
        return RotationMatrix(X_WEr.rotation())

    plan_list, gripper_setpoint_list, q_final_full = GenerateJointSpacePlan(ReturnSLERPedOrientation, 
                                                                  X_WObject.translation())

    plan_list.append(GraspObjectCompliancePlan(X_WObject, 5.0, max_force=4, grasp_width=ObjectWidths[obj_name], max_grip=.1))

    plan_2, _, q_2 = GenerateJointSpacePlan(ReturnConstantOrientation, 
                     X_WObject.translation() + [0,0,.2], start=X_WObject.translation(),
                     q_full=q_final_full)

    plan_3, _, q_3 = GenerateJointSpacePlan(ReturnConstantOrientation, 
                     X_WObject.translation(), start=X_WObject.translation() + [0,0,.2],
                     q_full=q_2)
    plan_list.append(plan_2[0])
    plan_list.append(plan_3[0])
    plan_list.append(ReleaseObjectCompliancePlan(X_WObject, 5.0, max_force=4, grasp_width=ObjectWidths[obj_name], max_grip=.1))
    
    #To make the planner happy, not used
    gripper_setpoint_list = [0.1, 0.1, 0.0, 0.0, 0.1]

    q0 = [0, 0, 0, -1.75, 0, 1.0, 0]
    iiwa_position_command_log, iiwa_position_measured_log, iiwa_external_torque_log, \
        state_log = manip_station_sim.RunSimulation(
            plan_list, gripper_setpoint_list, extra_time=2.0, real_time_rate=0.0, q0_kuka=q0)


#Uncomment to test, one test at a time
if __name__ == '__main__':
    #CabinetTest()
    NewObjectTest("small_brick")




#Skye Thompson
from robot_plans import *
from open_left_door_plans import OpenLeftDoorPlan
from pydrake.math import RotationMatrix

#--------------------------------------------------------------------------------------------------
#------------------------------------ open left door related constants       ----------------
# Define positions and transforms used in various tasks, e.g. opening left door.
# home position of point Q in world frame.
p_WQ_home = np.array([0.5, 0, 0.41])

# L: frame the cupboard left door, whose origin is at the center of the door body.
p_WL = np.array([0.7477, 0.1445, 0.4148]) #+ [-0.1, 0, 0]
# center of the left hinge of the door in frame L and W
p_LC_left_hinge = np.array([0.008, 0.1395, 0])
p_WC_left_hinge = p_WL + p_LC_left_hinge

# center of handle in frame L and W
p_LC_handle = np.array([-0.033, -0.1245, 0])
p_WC_handle = p_WL + p_LC_handle

# distance between the hinge center and the handle center
p_handle_2_hinge = p_LC_handle - p_LC_left_hinge
r_handle = np.linalg.norm(p_handle_2_hinge)

# angle between the world y axis and the line connecting the hinge cneter to the
# handle center when the left door is fully closed (left_hinge_angle = 0).
theta0_hinge = np.arctan2(np.abs(p_handle_2_hinge[0]),
                          np.abs(p_handle_2_hinge[1]))

p_EQ = GetEndEffectorWorldAlignedFrame().multiply(np.array([0., 0., 0.095]))

# orientation of end effector aligned frame
R_WEa_ref = RollPitchYaw(0, np.pi / 180 * 135, 0).ToRotationMatrix()

#Returns the handle's transform from the world
def GetHandleFromWorld(angle):
    X_WHr = Isometry3()
    X_WHr.set_rotation(
        RollPitchYaw(0, 0, angle).ToRotationMatrix().matrix())
    X_WHr.set_translation(
        p_WC_left_hinge +
        [-r_handle * np.sin(angle) - .045,
         -r_handle * np.cos(angle) + .015, 0])
    return X_WHr

def CalcGain():
    pass

#Base plan for grasping an object
class GraspObjectPlan(PlanBase):
    #Kinematics plan is the plan used to get to this object
    #used because it has the relevant CalcKinematics CMD
    def __init__(self, X_WObj, duration=3.0, max_force = 5, grasp_width=0.0125, plan_type=None, ):
        #Generate trajectory - object should not move
        t_knots = [i * duration/9 for i in range(10)]
        self.path_knots = np.array([X_WObj.translation() for i in range(10)])
        traj = PiecewisePolynomial.Cubic(t_knots, self.path_knots.T,
                                               np.zeros(3), np.zeros(3))
        self.traj = traj

        #For kinematics and reference orientation in Obj frame
        self.obj_pos = X_WObj.translation()
        self.obj_rot = X_WObj.rotation()
        
        self.Q_ObjL7_ref = RollPitchYaw(0, 5*np.pi/8, 0).ToRotationMatrix().ToQuaternion()
        
        self.grasp_width = grasp_width
        self.max_force = max_force

        self.plant_iiwa = station.get_controller_plant()
        self.tree_iiwa =self.plant_iiwa.tree()
        self.context_iiwa = self.plant_iiwa.CreateDefaultContext()
        self.l7_frame = self.plant_iiwa.GetFrameByName('iiwa_link_7')

        PlanBase.__init__(self,
                          type=plan_type,
                          trajectory=traj)
        
    def CalcKinematics(self, l7_frame, world_frame,tree_iiwa, context_iiwa, t_plan):
        p_L7Q = X_L7E.multiply(p_EQ)

        self.tree_iiwa = tree_iiwa
        self.context_iiwa = context_iiwa
        self.l7_frame = l7_frame

        Jv_WL7q = self.tree_iiwa.CalcFrameGeometricJacobianExpressedInWorld(
            context=self.context_iiwa, frame_B=self.l7_frame,
            p_BoFo_B=p_L7Q)
        self.Jv_WL7q = Jv_WL7q
        
        X_WObj = Isometry3()
        X_WObj.set_translation(self.traj.value(t_plan).flatten())
        X_WObj.set_rotation(self.obj_rot)
        X_ObjW = X_WObj.inverse()
        self.X_ObjW = X_ObjW

        X_WL7 = self.tree_iiwa.CalcRelativeTransform(
            self.context_iiwa, frame_A=world_frame,
            frame_B=self.l7_frame)
        self.X_WL7 = X_WL7

        p_WQ = X_WL7.multiply(p_L7Q)

        p_ObjQ = X_ObjW.multiply(p_WQ)

        return Jv_WL7q, p_ObjQ

#Grasps an object using information from the gripper's force sensor
class GraspObjectCompliancePlan(GraspObjectPlan):
    def __init__(self, X_WObj, duration, max_force=2, grasp_width=0.0125, max_grip=.04):
        GraspObjectPlan.__init__(
             self,
             X_WObj, 
             duration, 
             max_force,
             grasp_width = grasp_width,
             plan_type = PlanTypes["GraspObjectCompliancePlan"]
            )
        self.q_iiwa_previous = np.zeros(7)

        #The assumed starting grip width
        self.max_grip = max_grip
        self.past_grip_pt = max_grip

    def CalcPositionCommand(self, t_plan, q_iiwa, p_ObjQ, control_period, force=0):
        #Freezes the arm at a max duration, min grasp width, and max force
        if t_plan < self.duration and \
           self.past_grip_pt > self.grasp_width and force < 1:
            # The width of the gripper, interpolates between open and the min width
            grip_pt = max((self.duration - t_plan)/self.duration * self.max_grip, \
                           self.grasp_width)

            X_ObjL7 = self.X_ObjW.multiply(self.X_WL7)
            Q_WL7 = self.X_WL7.quaternion()
            Q_ObjL7 = X_ObjL7.quaternion()
            Q_L7L7r = Q_ObjL7.inverse().multiply(self.Q_ObjL7_ref)

            # first 3: angular velocity, last 3: translational velocity
            v_ee_desired = np.zeros(6)
            # Translation
            kp_translation = np.array([-1., -1., -1.])
           
            v_ee_desired[3:6] = kp_translation * p_ObjQ

            #Rotation
            kp_rotation = np.array([1., 1., 1.])
            v_ee_desired[0:3] = Q_WL7.multiply(kp_rotation * Q_L7L7r.xyz())

            #Centers the gripper based on forces felt in the fingers
            if force < self.max_force and force > .005:
                v_ee_desired[4] -= abs(force) * .25\
                                   * v_ee_desired[4]/abs(v_ee_desired[4])  

            result = np.linalg.lstsq(self.Jv_WL7q, v_ee_desired)
            q_dot_des = np.clip(result[0], -2, 2)
            q_ref = q_iiwa + q_dot_des * control_period
            self.q_iiwa_previous[:] = q_iiwa
            self.past_grip_pt = grip_pt

            return q_ref, grip_pt
        else:
            #Stops the gripper from closing when a max force is achieved
            self.past_grip_pt = max((self.duration - t_plan)/self.duration * self.max_grip, \
                                     self.grasp_width)
            if force > 2.5:
                self.grasp_width = self.past_grip_pt
            return self.q_iiwa_previous, self.past_grip_pt

    def CalcTorqueCommand(self):
        return np.zeros(7)

class ReleaseObjectCompliancePlan(GraspObjectPlan):
    def __init__(self, X_WObj, duration, max_force=2, grasp_width=0.0125, max_grip=.04):
        GraspObjectPlan.__init__(
             self,
             X_WObj, 
             duration, 
             max_force,
             grasp_width = grasp_width,
             plan_type = PlanTypes["GraspObjectCompliancePlan"]
            )
        self.q_iiwa_previous = np.zeros(7)

        #The assumed starting grip width
        self.max_grip = max_grip
        self.past_grip_pt = grasp_width
        self.flag = False

    def CalcPositionCommand(self, t_plan, q_iiwa, p_ObjQ, control_period, force=0):
        #Freezes the arm at a max duration and above a max force
        if t_plan < self.duration  and force < 1:
            self.flag = True

            # The width of the gripper, interpolates between the min width and open
            grip_pt = min((t_plan)/self.duration * (self.max_grip-self.grasp_width) + self.grasp_width, \
                           self.max_grip)

            X_ObjL7 = self.X_ObjW.multiply(self.X_WL7)
            Q_WL7 = self.X_WL7.quaternion()
            Q_ObjL7 = X_ObjL7.quaternion()
            Q_L7L7r = Q_ObjL7.inverse().multiply(self.Q_ObjL7_ref)

            # first 3: angular velocity, last 3: translational velocity
            v_ee_desired = np.zeros(6)
            # Translation
            kp_translation = np.array([-1., -1., -1.])
           
            v_ee_desired[3:6] = kp_translation * p_ObjQ

            #Rotation
            kp_rotation = np.array([1., 1., 1.])
            v_ee_desired[0:3] = Q_WL7.multiply(kp_rotation * Q_L7L7r.xyz())

            #Centers the gripper based on forces felt in the fingers
            if force < self.max_force and force > .01:
                v_ee_desired[4] -= abs(force) * .25\
                                   * v_ee_desired[4]/abs(v_ee_desired[4])  
            result = np.linalg.lstsq(self.Jv_WL7q, v_ee_desired)
            q_dot_des = np.clip(result[0], -2, 2)
            q_ref = q_iiwa + q_dot_des * control_period
            self.q_iiwa_previous[:] = q_iiwa
            self.past_grip_pt = grip_pt

            return q_ref, grip_pt
        else:
            #Stops the gripper from translating or rotating before a benchmark force is reached
            self.past_grip_pt = min((t_plan)/self.duration * (self.max_grip-self.grasp_width) + self.grasp_width, \
                           self.max_grip)
            if not self.flag:
                
                return q_iiwa, self.past_grip_pt
            return self.q_iiwa_previous, self.past_grip_pt

    def CalcTorqueCommand(self):
        return np.zeros(7)

#Opens the left door using the gripper force to shape its path
class OpenLeftDoorCompliancePlan(OpenLeftDoorPlan):
    def __init__(self, angle_start, angle_end=3 *np.pi/8, duration=10.0, type=None):
        OpenLeftDoorPlan.__init__(
            self,
            angle_start=angle_start,
            angle_end=angle_end,
            duration=duration,
            type=PlanTypes["OpenLeftDoorCompliancePlan"])
        self.q_iiwa_previous = np.zeros(7)
        self.max_force = 1

        self.X_WObj = GetHandleFromWorld(0)
        self.X_ObjW = self.X_WObj.inverse()
        self.Q_ObjL7_ref = RollPitchYaw(0, 5*np.pi/8, 0).ToRotationMatrix().ToQuaternion()

        self.plant_iiwa = station.get_controller_plant()
        self.tree_iiwa =self.plant_iiwa.tree()
        self.context_iiwa = self.plant_iiwa.CreateDefaultContext()
        self.l7_frame = self.plant_iiwa.GetFrameByName('iiwa_link_7')

    def CalcKinematics(self, l7_frame, world_frame, tree_iiwa, context_iiwa, t_plan):
        p_L7Q = X_L7E.multiply(p_EQ)
       
        Jv_WL7q = tree_iiwa.CalcFrameGeometricJacobianExpressedInWorld(
            context=context_iiwa, frame_B=l7_frame,
            p_BoFo_B=p_L7Q)
        self.Jv_WL7q = Jv_WL7q

        self.tree_iiwa = tree_iiwa
        self.context_iiwa = context_iiwa
        self.l7_frame = l7_frame

        # Translation
        # Hr: handle reference frame
        # p_HrQ: position of point Q relative to frame Hr.
        # X_WHr: transformation from Hr to world frame W.
        X_WHr = Isometry3()
        handle_angle_ref = self.traj.value(t_plan).flatten()
        X_WHr.set_rotation(
            RollPitchYaw(0, 0, 0).ToRotationMatrix().matrix())
        X_WHr.set_translation(
            p_WC_left_hinge +
            [-r_handle * np.sin(handle_angle_ref),
             -r_handle * np.cos(handle_angle_ref), 0])
        X_HrW = X_WHr.inverse()

        X_WL7 = tree_iiwa.CalcRelativeTransform(
            context_iiwa, frame_A=world_frame,
            frame_B=l7_frame)
        self.X_WL7 = X_WL7

        p_WQ = X_WL7.multiply(p_L7Q)
        p_HrQ = X_HrW.multiply(p_WQ)

        return Jv_WL7q, p_HrQ

    def CalcPositionCommand(self, t_plan, q_iiwa, Jv_WL7q, p_ObjQ, control_period, force=0):
        if t_plan < self.duration:
            X_ObjL7 = self.X_ObjW.multiply(self.X_WL7)
            Q_WL7 = self.X_WL7.quaternion()
            Q_ObjL7 = X_ObjL7.quaternion()
            Q_L7L7r = Q_ObjL7.inverse().multiply(self.Q_ObjL7_ref)

            # first 3: angular velocity, last 3: translational velocity
            v_ee_desired = np.zeros(6)
            # Translation
            kp_translation = np.array([-1. - (max(2* (abs(force) - 1), 0)),\
                                       max(-1. + (abs(force) - 1), 0),\
                                        -1.])
            v_ee_desired[3:6] = kp_translation * p_ObjQ

            #Rotation
            kp_rotation = np.array([1., 1., 1.])
            v_ee_desired[0:3] = Q_WL7.multiply(kp_rotation * Q_L7L7r.xyz())

            result = np.linalg.lstsq(self.Jv_WL7q, v_ee_desired)
            q_dot_des = np.clip(result[0], -2, 2)
            q_ref = q_iiwa + q_dot_des * control_period
            self.q_iiwa_previous[:] = q_iiwa

            return q_ref
        else:
            return self.q_iiwa_previous

    def CalcTorqueCommand(self):
        return np.zeros(7)

#Navigates to an object at a specified position and orientation
class GrabObjectPlan(PlanBase):
    def __init__(self, p_WQ, X_WObj, duration=10.0, type=None):
        self.obj_pos = X_WObj.translation()
        self.obj_rot = X_WObj.rotation()
        self.Q_ObjL7_ref = RollPitchYaw(0, 3*np.pi/4, 0).ToRotationMatrix().ToQuaternion()

        self.plant_iiwa = station.get_controller_plant()
        self.tree_iiwa =self.plant_iiwa.tree()
        self.context_iiwa = self.plant_iiwa.CreateDefaultContext()
        self.l7_frame = self.plant_iiwa.GetFrameByName('iiwa_link_7')

        t_knots = [i * duration/9 for i in range(10)]
        def InterpolateStraightLine(p_WQ_start, p_WQ_end, num_knot_points, i):
            return (p_WQ_end - p_WQ_start)/num_knot_points*(i+1) + p_WQ_start
        self.path_knots = np.zeros((10,3))

        for i in range(len(self.path_knots)):
            self.path_knots[i] = InterpolateStraightLine(p_WQ, self.obj_pos, len(self.path_knots), i)
        traj = PiecewisePolynomial.Cubic(t_knots, self.path_knots.T,
                                               np.zeros(3), np.zeros(3))
        self.traj = traj
        PlanBase.__init__(self,
                          type=type,
                          trajectory=traj)

    #Calculates the position of the gripper relative to 
    def CalcKinematics(self, l7_frame, world_frame, tree_iiwa, context_iiwa, t_plan):
        p_L7Q = X_L7E.multiply(p_EQ)

        self.tree_iiwa = tree_iiwa
        self.context_iiwa = context_iiwa
        self.l7_frame = l7_frame

        Jv_WL7q = self.tree_iiwa.CalcFrameGeometricJacobianExpressedInWorld(
            context=self.context_iiwa, frame_B=self.l7_frame,
            p_BoFo_B=p_L7Q)
        self.Jv_WL7q = Jv_WL7q
        
        X_WObj = Isometry3()
        X_WObj.set_translation(self.traj.value(t_plan).flatten())
        X_WObj.set_rotation(self.obj_rot)
        X_ObjW = X_WObj.inverse()
        self.X_ObjW = X_ObjW

        X_WL7 = self.tree_iiwa.CalcRelativeTransform(
            self.context_iiwa, frame_A=world_frame,
            frame_B=self.l7_frame)
        self.X_WL7 = X_WL7

        p_WQ = X_WL7.multiply(p_L7Q)

        p_ObjQ = X_ObjW.multiply(p_WQ)

        return Jv_WL7q, p_ObjQ


#Deprecated
class GrabObjectPositionPlan(GrabObjectPlan):
    def __init__(self, p_WQ, X_WObj, duration=10.0):
        GrabObjectPlan.__init__(
            self,
            p_WQ,
            X_WObj,
            duration=duration,
            type=PlanTypes["GrabObjectPositionPlan"])
        self.q_iiwa_previous = np.zeros(7)


    def CalcPositionCommand(self, t_plan, q_iiwa, p_ObjQ, control_period):
        if t_plan < self.duration:
            X_ObjL7 = self.X_ObjW.multiply(self.X_WL7)
            Q_WL7 = self.X_WL7.quaternion()
            Q_ObjL7 = X_ObjL7.quaternion()
            Q_L7L7r = Q_ObjL7.inverse().multiply(self.Q_ObjL7_ref)

            # first 3: angular velocity, last 3: translational velocity
            v_ee_desired = np.zeros(6)
            # Translation
            kp_translation = np.array([-1., -1., -1.])
            v_ee_desired[3:6] = kp_translation * p_ObjQ

            #Rotation
            kp_rotation = np.array([1., 1., 1.])
            v_ee_desired[0:3] = Q_WL7.multiply(kp_rotation * Q_L7L7r.xyz())

            result = np.linalg.lstsq(self.Jv_WL7q, v_ee_desired)
            q_dot_des = np.clip(result[0], -2, 2)
            q_ref = q_iiwa + q_dot_des * control_period
            self.q_iiwa_previous[:] = q_iiwa

            return q_ref
        else:
            return self.q_iiwa_previous

    def CalcTorqueCommand(self):
        return np.zeros(7)
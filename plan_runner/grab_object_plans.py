#Skye Thompson
from robot_plans import *
from open_left_door_plans import OpenLeftDoorPlan

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

#TODO: add a little noise to the orientation as well to show
#      the controller's ability to adapt
p_EQ = GetEndEffectorWorldAlignedFrame().multiply(np.array([0., 0., 0.095]))

#position of left finger in EE frame
#TODO: make an actual function of finger width
p_EL = GetEndEffectorWorldAlignedFrame().multiply(np.array([0., 0. - .03, 0.095]))
#position of right finger in EE frame
p_ER = GetEndEffectorWorldAlignedFrame().multiply(np.array([0., 0. + .03, 0.095]))

# orientation of end effector aligned frame
R_WEa_ref = RollPitchYaw(0, np.pi / 180 * 135, 0).ToRotationMatrix()

#Returns the handle's transform from the world
def GetHandleFromWorld(angle):
    X_WHr = Isometry3()
    X_WHr.set_rotation(
        RollPitchYaw(0, 0, angle).ToRotationMatrix().matrix())
    X_WHr.set_translation(
        p_WC_left_hinge +
        [-r_handle * np.sin(angle) - .05,
         -r_handle * np.cos(angle) + .02, 0])
    #X_HrW = X_WHr.inverse()
    return X_WHr

#Base plan for grasping an object
class GraspObjectPlan(PlanBase):
    #Kinematics plan is the plan used to get to this object
    #used because it has the relevant CalcKinematics CMD
    def __init__(self, kinematics_plan, duration=3.0, max_force = 5, grasp_width=0.0125, plan_type=None, ):
        self.max_force = max_force
        self.plan = kinematics_plan
        t_knots = [i * duration/9 for i in range(10)]
        self.path_knots = np.array([self.plan.path_knots[-1] for i in range(10)])
        traj = PiecewisePolynomial.Cubic(t_knots, self.path_knots.T,
                                               np.zeros(3), np.zeros(3))
        self.traj = traj
        self.grasp_width = grasp_width
        PlanBase.__init__(self,
                          type=plan_type,
                          trajectory=traj)
        
    def CalcKinematics(self, l7_frame, world_frame, tree_iiwa, context_iiwa, t_plan):
        self.plan.traj = self.traj
        return self.plan.CalcKinematics(l7_frame, world_frame, tree_iiwa, context_iiwa, t_plan)

#Grasps an object using information from the gripper's force sensor
class GraspObjectCompliancePlan(GraspObjectPlan):
    def __init__(self, kinematics_plan, duration, max_force=.5, grasp_width=0.0125):
        GraspObjectPlan.__init__(
             self,
             kinematics_plan, 
             duration, 
             max_force,
             grasp_width = grasp_width,
             plan_type = PlanTypes["GraspObjectCompliancePlan"]
            )
        self.q_iiwa_previous = np.zeros(7)
        #The assumed starting grip width
        self.past_grip_pt = 0.055


    def CalcPositionCommand(self, t_plan, q_iiwa, Jv_WL7q, p_HrQ, p_HrR, p_HrL, control_period, force=0):
        #Freezes the arm at a max duration, min grasp width, and max force
        if t_plan < self.duration and \
           self.past_grip_pt > self.grasp_width and force < 1:

            # The width of the gripper, interpolates between open and the min width
            grip_pt = max((self.duration - t_plan)/self.duration * 0.055, \
                           self.grasp_width)

            #Calculates the base position/orientation controller
            e = self.plan.GainCalculation(t_plan, p_HrQ, p_HrR, p_HrL)

            #Centers the gripper based on forces felt in the fingers
            if force < self.max_force:
                e[4] -= abs(force) * .25 * e[4]/abs(e[4])  

            result = np.linalg.lstsq(Jv_WL7q, e)
            q_dot_des = result[0]
            q_ref = q_iiwa + q_dot_des * control_period
            self.q_iiwa_previous[:] = q_iiwa
            self.past_grip_pt = grip_pt
            return q_ref, grip_pt
        else:
            #Stops the gripper from closing when a max force is achieved
            self.past_grip_pt = max((self.duration - t_plan)/self.duration * 0.055, \
                                     self.grasp_width)
            if force > 2:
                self.grasp_width = self.past_grip_pt
            return self.q_iiwa_previous, self.past_grip_pt

    def CalcTorqueCommand(self):
        return np.zeros(7)

#Opens the left door using the gripper force to shape its path
class OpenLeftDoorCompliancePlan(OpenLeftDoorPlan):
    def __init__(self, angle_start, angle_end=np.pi/4, duration=10.0, type=None):
        OpenLeftDoorPlan.__init__(
            self,
            angle_start=angle_start,
            angle_end=angle_end,
            duration=duration,
            type=PlanTypes["OpenLeftDoorCompliancePlan"])
        self.q_iiwa_previous = np.zeros(7)
        self.max_force = 1

    def CalcKinematics(self, l7_frame, world_frame, tree_iiwa, context_iiwa, t_plan):
        p_L7Q = X_L7E.multiply(p_EQ)
        p_L7R = X_L7E.multiply(p_ER)
        p_L7L = X_L7E.multiply(p_EL)
        Jv_WL7q = tree_iiwa.CalcFrameGeometricJacobianExpressedInWorld(
            context=context_iiwa, frame_B=l7_frame,
            p_BoFo_B=p_L7Q)

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

        p_WQ = X_WL7.multiply(p_L7Q)
        p_WR = X_WL7.multiply(p_L7R)
        p_WL = X_WL7.multiply(p_L7L)

        p_HrQ = X_HrW.multiply(p_WQ)
        p_HrR = X_HrW.multiply(p_WR)
        p_HrL = X_HrW.multiply(p_WL)

        return Jv_WL7q, p_HrQ, p_HrR, p_HrL


    def GainCalculation(self, t_plan, p_HrQ, p_HrR, p_HrL, force=0):
        #Uses relation of forces to open the door
        g1 = ( - (abs(force) - 1))
        g2 = (5 - (abs(force) - 1) )

        gain = np.array([[g1[0], 0, 0],
                         [0, g2[0], 0],
                         [0, 0, 10]])

        #angular gain should be:
        # dif in z of fingers -> roll
        # dif in y of fingers -> yaw
        # pitch of fingers should move to ~pi/4
        finger_dif = p_HrR - p_HrL
        ang_gain = np.array( [[0, 0, 0],
                             [0, 0, -3],
                             [3, 0, 0]])
        #TODO: Try clipping finger_dif to get better performance?
        ang_e_des = np.array([0,0,0])#np.matmul(finger_dif, ang_gain)
        ang_e_des[1] = ((p_HrQ - p_HrR)[2] -.02) * -1
        e_des = np.matmul(p_HrQ, gain)
        e = - np.hstack([ang_e_des, e_des])

        return e

    def CalcPositionCommand(self, t_plan, q_iiwa, Jv_WL7q, p_HrQ, p_HrR, p_HrL, control_period, force=0):
        #Do same centering as in grabobjectplan with low gain
        #Add a higher-gain term for force
        if t_plan < self.duration :
            e = self.GainCalculation(t_plan, p_HrQ, p_HrR, p_HrL, force)
            result = np.linalg.lstsq(Jv_WL7q, e)
            q_dot_des = result[0]
            q_ref = q_iiwa + q_dot_des * control_period
            self.q_iiwa_previous[:] = q_iiwa

            return q_ref
        else:
            return self.q_iiwa_previous

    def CalcTorqueCommand(self):
        return np.zeros(7)

#Navigates to an object at a specified position and orientation
class GrabObjectPlan(PlanBase):
    def __init__(self, p_WQ, p_WR, p_WL, X_WObj, duration=20.0, type=None):
        t_knots = [i * duration/9 for i in range(10)]
        self.obj_pos = X_WObj.translation()
        self.obj_rot = X_WObj.rotation()
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
        p_L7R = X_L7E.multiply(p_ER)
        p_L7L = X_L7E.multiply(p_EL)

        Jv_WL7q = tree_iiwa.CalcFrameGeometricJacobianExpressedInWorld(
            context=context_iiwa, frame_B=l7_frame,
            p_BoFo_B=p_L7Q)
        X_WHr = Isometry3()

        X_WHr.set_translation(self.traj.value(t_plan).flatten())
        X_WHr.set_rotation(self.obj_rot)
        X_HrW = X_WHr.inverse()

        X_WL7 = tree_iiwa.CalcRelativeTransform(
            context_iiwa, frame_A=world_frame,
            frame_B=l7_frame)

        p_WQ = X_WL7.multiply(p_L7Q)
        p_WR = X_WL7.multiply(p_L7R)
        p_WL = X_WL7.multiply(p_L7L)

        p_HrQ = X_HrW.multiply(p_WQ)
        p_HrR = X_HrW.multiply(p_WR)
        p_HrL = X_HrW.multiply(p_WL)

        #TODO: return all three Hr relative values
        return Jv_WL7q, p_HrQ, p_HrR, p_HrL


class GrabObjectPositionPlan(GrabObjectPlan):
    def __init__(self, p_WQ, p_WR, p_WL, X_WObj, duration=10.0):
        GrabObjectPlan.__init__(
            self,
            p_WQ,
            p_WR, 
            p_WL,
            X_WObj,
            duration=duration,
            type=PlanTypes["GrabObjectPositionPlan"])
        self.q_iiwa_previous = np.zeros(7)

    def GainCalculation(self, t_plan, p_HrQ, p_HrR, p_HrL):
        for coord in range(len(p_HrQ)):
            if abs(p_HrQ[coord]) < .0075 and coord != 1:
                p_HrQ[coord] = 0
        gain = np.array([[10, 0, 0],
                         [0, 10, 0],
                         [0, 0, 10]])
        #angular gain should be:
        # dif in z of fingers -> roll
        # dif in y of fingers -> yaw
        finger_dif = p_HrR - p_HrL
        print("FINGER DIF")
        print(finger_dif)
        # for dif in range(len(finger_dif)):
        #     if abs(finger_dif[dif]) < .0075:
        #         finger_dif[dif] = 0
        ang_gain = np.array( [[0, 0, -3],
                             [0, 0, 0],
                             [3, 0, 0]])
        ang_e_des = np.matmul(finger_dif, ang_gain)
        ang_e_des[1] = ((p_HrQ - p_HrR)[2] - .02) * -1
        e_des = np.matmul(p_HrQ, gain)
        e = - np.hstack([ang_e_des, e_des])

        return e


    def CalcPositionCommand(self, t_plan, q_iiwa, Jv_WL7q, p_HrQ, p_HrR, p_HrL, control_period):
        '''
        :param t_plan: t_plan: time passed since the beginning of this Plan, expressed in seconds.
        :param q_iiwa: current configuration of the robot.
        :param Jv_WL7q: geometric jacboain of point Q in frame L7.
        :param p_HrQ: position of point Q relative to frame Hr.
        :param control_period: the amount of time between consecutive command updates.
        :return: position command to the robot.
        '''
        # Your code here
        #------------------------------------------------------------------------#
        if t_plan < self.duration:
            e = self.GainCalculation(t_plan, p_HrQ, p_HrR, p_HrL)
            result = np.linalg.lstsq(Jv_WL7q, e)
            q_dot_des = result[0]
            q_ref = q_iiwa + q_dot_des * control_period

            self.q_iiwa_previous[:] = q_iiwa
            return q_ref
        else:
            return self.q_iiwa_previous
        #------------------------------------------------------------------------#
        return q_iiwa

    def CalcTorqueCommand(self):
        return np.zeros(7)
from robot_plans import *

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
error = np.random.uniform(-.04, .04)
error = 0
print("ERROR")
print(error)
p_EQ = GetEndEffectorWorldAlignedFrame().multiply(np.array([0., 0. + error, 0.095]))

#position of left finger in EE frame
#TODO: make an actual function of finger width
p_EL = GetEndEffectorWorldAlignedFrame().multiply(np.array([0., 0. + error - .03, 0.095]))
#position of right finger in EE frame
p_ER = GetEndEffectorWorldAlignedFrame().multiply(np.array([0., 0. + error + .03, 0.095]))

# orientation of end effector aligned frame
R_WEa_ref = RollPitchYaw(0, np.pi / 180 * 135, 0).ToRotationMatrix()


class GraspObjectPlan(PlanBase):
    #TODO: Object width, max grip force?
    def __init__(self, kinematics_plan, duration, max_force = 5, plan_type=None):
        self.max_force = max_force
        self.plan = kinematics_plan
        t_knots = [i * duration/9 for i in range(10)]
        self.path_knots = [self.plan.path_knots[-1] for i in range(10)]
        traj = PiecewisePolynomial.Cubic(t_knots, path_knots.T,
                                               np.zeros(3), np.zeros(3))
        #Command Gripper?
        PlanBase.__init__(self,
                          type=plan_type,
                          trajectory=traj)
        
    def CalcKinematics(self, X_L7E, l7_frame, world_frame, tree_iiwa, context_iiwa, t_plan):
        return self.plan.CalcKinematics(X_L7E, l7_frame, world_frame, tree_iiwa, context_iiwa, t_plan)

class GraspObjectCompliancePlan(PlanBase):
    def __init__(self, kinematics_plan, duration, max_force=5):
        GraspObjectPlan.__init__(
             kinematics_plan, 
             duration, 
             max_force,
             plan_type = PlanTypes[7]
            )
        self.q_iiwa_previous = np.zeros(7)


    def CalcPositionCommand(self, t_plan, q_iiwa, Jv_WL7q, p_HrQ, p_HrR, p_HrL, control_period, force=0):
        #Do same centering as in grabobjectplan with low gain
        #Add a higher-gain term for force
        if t_plan < self.duration and abs(force) < self.max_force:
            e = self.plan.GainCalculation(_)
            e[4] -= abs(force) * .1 * e[4]/abs(e[4]) 
            result = np.linalg.lstsq(Jv_WL7q, e)
            q_dot_des = result[0]
            q_ref = q_iiwa + q_dot_des * control_period

            self.q_iiwa_previous[:] = q_iiwa
            return q_ref
        else:
            return self.q_iiwa_previous

    def CalcTorqueCommand(self):
        return np.zeros(7)

#TODO: Set up plan for grabbing handle using the modified controller
class GrabHandlePlan(PlanBase):
    def __init__(self):
        pass

    def CalcKinematics(self, X_L7E, l7_frame, world_frame, tree_iiwa, context_iiwa, t_plan):
        pass

#TODO:
class GrabHandlePositionPlan(GrabHandlePlan):
    def __init__(self):
        pass

    def CalcPositionCommand(self, t_plan, q_iiwa, Jv_WL7q, p_HrQ, p_HrR, p_HrL, control_period):
        pass


#TODO: Make more general?
class GrabObjectPlan(PlanBase):

    def __init__(self, p_WQ, p_WR, p_WL, X_WObj, duration=10.0, type=None):
        t_knots = [0, duration]
        self.obj_pos = X_WObj.translation()
        self.obj_rot = X_WObj.rotation()
        def InterpolateStraightLine(p_WQ_start, p_WQ_end, num_knot_points, i):
            return (p_WQ_end - p_WQ_start)/num_knot_points*(i+1) + p_WQ_start
        self.path_knots = np.zeros((2,3))
        self.path_knots[0] = InterpolateStraightLine(p_WQ, self.obj_pos, 2, 0)
        self.path_knots[1] = InterpolateStraightLine(p_WQ, self.obj_pos, 2, 1)
        
        traj = PiecewisePolynomial.Cubic(t_knots, self.path_knots.T,
                                               np.zeros(3), np.zeros(3))
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

        #Translation: Object Position
        X_WHr.set_translation(self.obj_pos)
        X_WHr.set_rotation(self.obj_rot)
        #RollPitchYaw(0,0, 0).ToRotationMatrix().matrix()
        #Orientation: Object Quat
        X_HrW = X_WHr.inverse()

        X_WL7 = tree_iiwa.CalcRelativeTransform(
            context_iiwa, frame_A=world_frame,
            frame_B=l7_frame)

        p_WQ = X_WL7.multiply(p_L7Q)
        p_WR = X_WL7.multiply(p_L7R)
        p_WL = X_WL7.multiply(p_L7L)

        p_HrQ = X_HrW.multiply(p_WQ)
        p_HrR = X_HrW.multiply(p_WQ)
        p_HrL = X_HrW.multiply(p_WQ)

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
        gain = np.array([[.4, 0, 0],
                         [0, .1, 0],
                         [0, 0, .11]])
        #angular gain should be:
        # dif in z of fingers -> roll
        # dif in y of fingers -> yaw
        finger_dif = p_HrR - p_HrL
        ang_gain = np.array( [[0, 0, 0],
                             [0, 0, 1],
                             [1, 0, 0]])
        ang_e_des = np.matmul(finger_dif, ang_gain)

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
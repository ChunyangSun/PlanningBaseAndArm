import logging, numpy, openravepy, time, IPython

class GraspModel(object):
    """ assorted functions for getting best score grasps from hw1_grasp """
    def __init__(self, robot, obj):
        self.s = [0, 0] # singmaMin range
        self.v = [0, 0] # volumeG range
        self.i = [0, 0] # isotropy range
        self.raw_scores = []
        self.robot = robot 
        self.obj = obj

        self.gmodel = openravepy.databases.grasping.GraspingModel(self.robot, self.obj)

        if not self.gmodel.load():
          self.gmodel.autogenerate()

        self.graspindices = self.gmodel.graspindices
        self.grasps = self.gmodel.grasps
        self.grasps_ordered = self.grasps.copy() #you should change the order of self.grasps_ordered
        self.delay = 20


    def order_grasps(self):
        """ order the grasps - call eval grasp on each, set the 'performance' index, and sort 
            call get_raw_score_range and combine the scores of each item 
            sort the scores from high to low 
            output: a list of grasps starting from the best 
        """

        # get raw scores before sorting
        self.get_raw_score_range()

        # normalize raw scores and linearly combine them to get the final score for each grasp
        # This cannot be done in eval() function because the run-time eval() doesn't have those ranges to normalize the metrics until all grasps are evaluated.
        for i, grasp in enumerate(self.grasps_ordered):
          sigmaMin = self.raw_scores[i][0]
          volumeG = self.raw_scores[i][1]
          isotropy = self.raw_scores[i][2]
          score = 2.0*self.normalize(sigmaMin, self.s) + 5.0*self.normalize(volumeG, self.v) + 10.0*self.normalize(isotropy, self.i)
          grasp[self.graspindices.get('performance')] = score

        # sort!
        order = numpy.argsort(self.grasps_ordered[:,self.graspindices.get('performance')[0]])
        order = order[::-1]
        self.grasps_ordered = self.grasps_ordered[order]

    def get_raw_score_range(self):
        """  inumpyut: self.grasps orginal, call eval_grasp to get scores
             output: range of scores for each evaluation items  
        """
        # go this loop to get raw scores and find the range of the raw scores
        for grasp in self.grasps_ordered:
          self.raw_scores.append(self.eval_grasp(grasp))

        # get the range of the three metric through the whole loop doing evaluation for each grasp
        all_sigmaMin = [n[0] for n in self.raw_scores]
        all_volumnG = [n[1] for n in self.raw_scores]
        all_isotropy = [n[2] for n in self.raw_scores]
        self.s = [min(all_sigmaMin), max(all_sigmaMin)]
        self.v = [min(all_volumnG), max(all_volumnG)]
        self.i = [min(all_isotropy), max(all_isotropy)]

    def eval_grasp(self, grasp):
        """ function to evaluate grasps, returns a score, which is some metric of the grasp, 
            higher score should be a better grasp 
        """

        with self.robot:
          #contacts is a 2d array, where contacts[i,0-2] are the positions of contact i and contacts[i,3-5] is the direction
          try:
            contacts,finalconfig,mindist,volume = self.gmodel.testGrasp(grasp=grasp,translate=True,forceclosure=False)

            obj_position = self.gmodel.target.GetTransform()[0:3,3]
            # for each contact
            G = numpy.zeros(shape=(6, len(contacts))) #the wrench matrix
            wrench = numpy.zeros(shape=(6,1))
            for i, c in enumerate(contacts):
              pos = c[0:3] - obj_position
              dir = -c[3:] #this is already a unit vector
              
              # fill G
              torque = numpy.cross(pos, dir)
              wrench = numpy.concatenate([dir, torque])

              G[:, i] = wrench
            
            # Use SVD to compute minimum score
            U, S, V = numpy.linalg.svd(G)
            sigmaMin = abs(S[-1])
            sigmaMax = abs(S[0])
            volumeG = numpy.linalg.det(numpy.dot(G, numpy.transpose(G))) ** 0.5
            isotropy = abs(float(sigmaMin) / sigmaMax)

            score = [sigmaMin, volumeG, isotropy]
            return score
          except openravepy.planning_error,e:
            #you get here if there is a failure in planning
            #example: if the hand is already intersecting the object at the initial position/orientation
            return [0.00, 0.00, 0.00]

    #displays the grasp
    def show_grasp(self, grasp, delay=1.5):
        with openravepy.RobotStateSaver(self.gmodel.robot):
          with self.gmodel.GripperVisibility(self.gmodel.manip):
            time.sleep(0.1) # let viewer update?
            try:
              with self.robot.GetEnv():
                contacts,finalconfig,mindist,volume = self.gmodel.testGrasp(grasp=grasp,translate=True,forceclosure=True)
                #if mindist == 0:
                # print 'grasp is not in force closure!'
                contactgraph = self.gmodel.drawContacts(contacts) if len(contacts) > 0 else None
                self.gmodel.robot.GetController().Reset(0)
                self.gmodel.robot.SetDOFValues(finalconfig[0])
                self.gmodel.robot.SetTransform(finalconfig[1])
                self.robot.GetEnv().UpdatePublishedBodies()
                time.sleep(delay)
            except openravepy.planning_error,e:
              print 'bad grasp!',e
    def normalize(self,n, range):
        return (1.0*n - range[0])/(range[1] - range[0])

class GraspPlanner(object):

    def __init__(self, robot, base_planner, arm_planner):
        self.robot = robot
        self.base_planner = base_planner
        self.arm_planner = arm_planner
        self.delay = 20

    def GetBasePoseForObjectGrasp(self, obj):
        # Load grasp database
        graspModel = GraspModel(self.robot, obj)
        graspModel.order_grasps()

        for i in range(6):
            print 'Showing grasp ', i
            graspModel.show_grasp(graspModel.grasps_ordered[i], delay=self.delay)
            print "using grasp 0 to generate Transform "
            Tgrasp = graspModel.gmodel.getGlobalGraspTransform(graspModel.grasps_ordered[i], collisionfree=True) # get the grasp transform
            if Tgrasp != None:
                break

        base_pose = None
        grasp_config = None
       
        ###################################################################
        # TODO: Here you will fill in the function to compute
        #  a base pose and associated grasp config for the 
        #  grasping the bottle
        ###################################################################
        self.irmodel = openravepy.databases.inversereachability.InverseReachabilityModel(self.robot)

        loaded = self.irmodel.load()
        if loaded: 
            densityfn,samplerfn,bounds = self.irmodel.computeBaseDistribution(Tgrasp)
        # densityfn: gaussian kernel density function taking poses of openrave quaternion type, returns probabilities
        # samplerfn: gaussian kernel sampler function taking number of sample and weight, returns robot base poses and joint states
        # bounds: 2x3 array, bounds of samples, [[min rotation, min x, min y],[max rotation, max x, max y]]

        goals = []
        numfailures = 0
        starttime = time.time()
        timeout = 5000
        N = 5 

        with self.robot:
            while len(goals) < N:
                if time.time()-starttime > timeout:
                    break
                poses,jointstate = samplerfn(N-len(goals))
                for pose in poses:
                    self.robot.SetTransform(pose)
                    self.robot.SetDOFValues(*jointstate)
                    # validate that base is not in collision
                    if not self.irmodel.manip.CheckIndependentCollision(openravepy.CollisionReport()):
                        q = self.irmodel.manip.FindIKSolution(Tgrasp,filteroptions=openravepy.IkFilterOptions.CheckEnvCollisions)
                        if q is not None:
                            values = self.robot.GetDOFValues()
                            values[self.irmodel.manip.GetArmIndices()] = q
                            goals.append((Tgrasp,pose,values))
                        elif self.irmodel.manip.FindIKSolution(Tgrasp,0) is None:
                            numfailures += 1

        self.armmanip = self.robot.GetManipulator('right_wam')
        self.robot.SetActiveManipulator('right_wam')
        print 'showing %d results'%N
        for ind,goal in enumerate(goals):
            raw_input('press ENTER to show goal %d'%ind)
            Tgrasp,base_pose, all_config = goal

            self.robot.SetTransform(base_pose)
            self.robot.SetDOFValues(all_config)

            # pose = numpy.array([0,0,0,0,0,0,0])
            
        # IPython.embed()
        idx = raw_input("Choose from goal index 0, 1, 2, 3, 4")
        goal_chosen = goals[int(idx)]
        Tgrasp,base_pose, all_config = goal
        self.robot.SetTransform(base_pose)
        self.robot.SetDOFValues(all_config)
        arm_config = self.robot.GetActiveDOFValues()

        return base_pose[4:], arm_config # base_pose [x, y] grasp_config [7 dof values]

    def PlanToGrasp(self, obj):

        # Next select a pose for the base and an associated ik for the arm
        base_pose, grasp_config = self.GetBasePoseForObjectGrasp(obj)

        if base_pose is None or grasp_config is None:
            print 'Failed to find solution'
            exit()

        # Now plan to the base pose
        start_pose = numpy.array(self.base_planner.planning_env.herb.GetCurrentConfiguration())
        base_plan = self.base_planner.Plan(start_pose, base_pose)
        base_traj = self.base_planner.planning_env.herb.ConvertPlanToTrajectory(base_plan)

        print 'Executing base trajectory'
        self.base_planner.planning_env.herb.ExecuteTrajectory(base_traj)

        # Now plan the arm to the grasp configuration
      
        start_config = numpy.array(self.arm_planner.planning_env.herb.GetCurrentConfiguration())
        arm_plan = self.arm_planner.Plan(start_config, grasp_config)
        arm_traj = self.arm_planner.planning_env.herb.ConvertPlanToTrajectory(arm_plan)

        print 'Executing arm trajectory'
        self.arm_planner.planning_env.herb.ExecuteTrajectory(arm_traj)

        # Grasp the bottle
        task_manipulation = openravepy.interfaces.TaskManipulation(self.robot)
        task_manipulation.CloseFingers()
    

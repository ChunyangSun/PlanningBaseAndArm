import numpy, operator
from RRTPlanner import RRTTree

class RRTConnectPlanner(object):

    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
        

    def Plan(self, start_config, goal_config, epsilon = 0.001):

	# count for executing timing
        import time
        t0 = time.clock()	

	# ftree start from start_config        
        ftree = RRTTree(self.planning_env, start_config)
	# rtree start from goal_config
        rtree = RRTTree(self.planning_env, goal_config)
        plan = []

        if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
            self.planning_env.InitializePlot(goal_config)
        # TODO: Here you will implement the rrt connect planner
        #  The return path should be an array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space
	
	# start the Bidirection RRT algorithm
        # set up the number of iterations we want to try before we give up
        num_iter = 1000

        # if two trees, ftree and rtree suceeed to connect each other, make isFail = False
        isFail = True
        import random
        import time
        num_vertex = 0
        for i in range(num_iter):
		# generate random configuration, config_q
                config_q = self.planning_env.GenerateRandomConfiguration()

                # get the nearest neighbor from the ftree and rtree, based on config_q
                f_mid, f_mdist = ftree.GetNearestVertex(config_q)
		r_mid, r_mdist = rtree.GetNearestVertex(config_q)
                
		# get the nearest neighbor from ftree and rtree, f_config_m and r_config_m
                f_config_m = ftree.vertices[f_mid]
		r_config_m = rtree.vertices[r_mid]

                # append f_config_n/r_config_n to the tree ftree/rtree if they are addable
                # where f/r_config_n  is the extended node between f/r_config_m and config_q
                f_config_n = self.planning_env.Extend(f_config_m,config_q)
		r_config_n = self.planning_env.Extend(r_config_m,config_q)
		
		# add f_config_n to ftree if addable
		if not numpy.array_equal(f_config_n, f_config_m):
			num_vertex = num_vertex + 1
			ftree.AddVertex(f_config_n)
                        ftree.AddEdge(f_mid,len(ftree.vertices)-1) # id of config_n is the last one in vertices
                        # draw the expended edge
                        if len(f_config_m) == 2 :
                        	self.planning_env.PlotEdge(f_config_m, f_config_n)
		# add r_config_n to rtree if addable
		if not numpy.array_equal(r_config_n, r_config_m):
			num_vertex = num_vertex + 1
                        rtree.AddVertex(r_config_n)
                        rtree.AddEdge(r_mid,len(rtree.vertices)-1) # id of config_n is the last one in vertices
                        # draw the expended edge
                        if len(r_config_m) == 2 :
                        	self.planning_env.PlotEdge(r_config_m, r_config_n)
		# break the loop if ftree and rtree connected
                if numpy.array_equal(f_config_n, r_config_n) and not numpy.array_equal(f_config_n, f_config_m) and not numpy.array_equal(r_config_n, r_config_m):
			isFail = False
			f_goal_id = len(ftree.vertices)-1
			r_goal_id = len(rtree.vertices)-1
			break
                        
	# recursive append the waypoints to the tree based on the tree.edges information
        # stop if we trace back to root, id = 0
	# we need to reverse for ftree not for rtree
	
	print "number of vertices: %d" %(num_vertex)
	
        # start here
        if isFail:
        	print "path length: 0"
        	print "plan time: %f" %(time.clock() - t0)
                return []
        else:
		# start to append waypoints from ftree
                current_id = f_goal_id
                while current_id != 0:
                        plan.append(ftree.vertices[current_id])
                        current_id = ftree.edges[current_id]
                plan.append(ftree.vertices[current_id]) # add start config to the plan
        	# reverse the order of plan
        	plan = plan[::-1]
		# keep adding waypoints from rtree, and ignore the overlapped config
		current_id = rtree.edges[r_goal_id] 
		while current_id != 0:
                        plan.append(rtree.vertices[current_id])
                        current_id = rtree.edges[current_id]
                plan.append(rtree.vertices[current_id]) # add goal config to the plan	
                
        path_length = 0
        for i in xrange(len(plan)-1):
            path_length = path_length + self.planning_env.ComputeDistance(plan[i], plan[i+1])
        print "path length: %f" %(path_length)
        print "plan time: %f" %(time.clock() - t0)
	
        return plan

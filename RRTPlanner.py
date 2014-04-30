import numpy
from RRTTree import RRTTree

class RRTPlanner(object):

    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
        

    def Plan(self, start_config, goal_config, epsilon = 0.001):
        
        # count for executing timing
        import time
        t0 = time.clock()
        
        tree = RRTTree(self.planning_env, start_config)
        plan = []
        if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
            self.planning_env.InitializePlot(goal_config)
        # TODO: Here you will implement the rrt planner
        #  The return path should be an array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space

        # start the Forward RRT algorithm
        # set up the number of iterations we want to try before we give up
        num_iter = 1000
        # set up the probability to generate goal config
        prob_goal = 0.2

        # if it suceeeds to connect the goal, make isFail = False
        isFail = True
        import random
        import time
        num_vertex = 0
        for i in xrange(num_iter):
            # generate random configuration, config_q
            prob = random.random()
            if prob < prob_goal:
                config_q = numpy.copy(goal_config)
            else:
                config_q = self.planning_env.GenerateRandomConfiguration()
            
            # get the nearest neighbor from the Tree, based on config_q
            mid, mdist = tree.GetNearestVertex(config_q)
            # get the nearest neighbor, config_m
            config_m = tree.vertices[mid]
            # append config_n to the tree if config_n is addable
            # where config_n is the extended node between config_m and config_q
            # in our case, if it is extendable, config_n always equals to config_q
            config_n = self.planning_env.Extend(config_m, config_q)
            if numpy.array_equal(config_n, config_m):
                continue
            else:
                num_vertex = num_vertex + 1
                tree.AddVertex(config_n)
                tree.AddEdge(mid,len(tree.vertices)-1) # id of config_n is the last one in vertices
                # draw the expended edge
                if len(config_m) == 2 :
                    self.planning_env.PlotEdge(config_m, config_n)      

            # check if config_n equals to goal_config, if yes, break
            if numpy.array_equal(config_n, goal_config):
                isFail = False
                goal_id = len(tree.vertices)-1
                break

        # recursive append the waypoints to the tree based on the tree.edges information
        # stop if we trace back to root, id = 0
        # start here
        
        print "number of vertices: %d" %(num_vertex)
        
        if isFail:
            print "path length: 0"
            print "plan time: %f" %(time.clock() - t0)
            return []
        else:
            current_id = goal_id
            while current_id != 0:
                plan.append(tree.vertices[current_id])
                current_id = tree.edges[current_id]
            plan.append(tree.vertices[current_id]) # add start config to the plan
        
        # reverse the order of plan
            plan = plan[::-1]
        
        path_length = 0
        for i in xrange(len(plan)-1):
            path_length = path_length + self.planning_env.ComputeDistance(plan[i], plan[i+1])
        print "path length: %f" %(path_length)
        print "plan time: %f" %(time.clock() - t0)
            
        return plan

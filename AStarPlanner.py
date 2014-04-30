import numpy
from time import time
import pylab as pl
from SimpleEnvironment import Action, Control

class AStarPlanner(object):

    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
        self.cost_optimal = dict()
        self.came_from = dict()
        self.visited = set()

    # Compute optimal cost to come to the node "cur"
    # if path is going through the node "parent"
    def cost_to_come(self, parent, cur):
        dist = self.cost_optimal[parent]
        # NOTE: in current implementation in could be just dist += 1
        # for a simple environment
        dist += self.planning_env.discrete_env.to_dec(self.planning_env.ComputeHeuristicCost(parent, cur))
        return dist

    def cost_to_go(self, cur, goal):
        return self.planning_env.discrete_env.to_dec(self.planning_env.ComputeHeuristicCost(cur, goal))


    # AStar planner
    #  The return path is a numpy array
    #  of dimension k x n where k is the number of waypoints
    #  and n is the dimension of the robots configuration space
    def Plan(self, start_config, goal_config):

        if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
            self.planning_env.InitializePlot(goal_config)
            showPath = True
        else:
            showPath = False

        print "AStar planner started"
        start_time = time()

        # Priority queue as a dict
        # Stores entries in the format:
        #   [ID, cost_heur(prority)]
        Q = dict()

        penv = self.planning_env
        denv = penv.discrete_env

        start = denv.ConfigurationToNodeId(start_config)
        goal = denv.ConfigurationToNodeId(goal_config)

        # Start from start node
        self.visited.add(start)
        self.cost_optimal[start] = denv.to_dec(0.0)
        self.came_from[start] = [-1, Action(Control(0,0,0), [start_config - start_config])]
        Q[start] = self.cost_to_go(start,goal)

        # And process nodes in the order of heuristic cost
        while Q:
            cur, priority = min(Q.items(),key=lambda x:x[1])
            # print Q
            #print cur, priority
            #print self.cost_optimal[cur], self.cost_to_go(cur,goal)
            # self.planning_env.isValidCoord(self.planning_env.discrete_env.NodeIdToGridCoord(cur))
            # raw_input("Top config...")

            del Q[cur]

            if (cur == goal):
                break

            for succ in self.planning_env.GetSuccessors(cur):
                n = succ.node_id
                act = succ.action
                #raw_input("Successor...")
                if (n not in self.visited):
                    # Add new nodes
                    cost_heur = self.cost_to_come(cur, n)
                    cost_heur += self.cost_to_go(n, goal)

                    self.visited.add(n)
                    self.cost_optimal[n] = self.cost_to_come(cur, n)
                    self.came_from[n] = [cur, act]
                    Q[n] = cost_heur
                    # if self.cost_optimal[n] == 0: import IPython; IPython.embed();
                    #print "1: ", n, cost_heur, self.cost_optimal[n]
                    #print "new ", cost_heur
                    if showPath:
                        penv.PlotEdge(denv.NodeIdToConfiguration(cur), denv.NodeIdToConfiguration(n))
                else:
                    # Update optimal cost for known nodes
                    if (self.cost_to_come(cur,n) < self.cost_optimal[n]):
                        self.cost_optimal[n] = self.cost_to_come(cur,n)
                        self.came_from[n] = [cur, act]
                        Q[n] = self.cost_optimal[n] + self.cost_to_go(n,goal)
                        #print "2: ", n, self.cost_optimal[n] + self.cost_to_go(n,goal), self.cost_optimal[n]
                        #print "updated ", Q[n]
                        if showPath:
                            penv.PlotEdge(denv.NodeIdToConfiguration(cur), denv.NodeIdToConfiguration(n))

        print start, goal

        # Reconstruct path
        plan_act = [self.came_from[goal][1]]
        x = goal

        while (x != start):
            if showPath:
                penv.PlotEdge2(denv.NodeIdToConfiguration(x), denv.NodeIdToConfiguration(self.came_from[x]))
            x = self.came_from[x][0]
            plan_act.append(self.came_from[x][1])

        if showPath:
            pl.draw()

        plan_arr = []

        for i in reversed(plan_act):
            plan_arr.append(i)

        plan = numpy.array(plan_arr)

        end_time = time()

        print "AStar planner completed"
        print "  time   = " + str((end_time - start_time))
        print "  nodes  = " + str(len(self.visited))

        return plan

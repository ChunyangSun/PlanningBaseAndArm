import numpy, openravepy
import pylab as pl
from DiscreteEnvironment import DiscreteEnvironment

class Control(object):
    def __init__(self, omega_left, omega_right, duration):
        self.ul = omega_left
        self.ur = omega_right
        self.dt = duration

class Action(object):
    def __init__(self, control, footprint):
        self.control = control
        self.footprint = footprint

class Successor(object):
    def __init__(self, node_id, action):
        self.node_id = node_id
        self.action = action

class SimpleEnvironment(object):

    def __init__(self, herb, resolution):
        self.herb = herb
        self.robot = herb.robot
        self.env = self.robot.GetEnv()
        self.table = self.env.GetBodies()[1] # table
        self.boundary_limits = [[-5., -5., -numpy.pi], [5., 5., numpy.pi]]
        self.lower_limits, self.upper_limits = self.boundary_limits
        self.discrete_env = DiscreteEnvironment(resolution, self.lower_limits, self.upper_limits)

        self.resolution = resolution
        self.ConstructActions()

    def GenerateFootprintFromControl(self, start_config, control, stepsize=0.01):

        # Extract the elements of the control
        ul = control.ul
        ur = control.ur
        dt = control.dt

        # Initialize the footprint
        config = start_config.copy()
        footprint = [numpy.array([0., 0., config[2]])]
        timecount = 0.0
        while timecount < dt:
            # Generate the velocities based on the forward model
            xdot = 0.5 * self.herb.wheel_radius * (ul + ur) * numpy.cos(config[2])
            ydot = 0.5 * self.herb.wheel_radius * (ul + ur) * numpy.sin(config[2])
            tdot = self.herb.wheel_radius * (ul - ur) / self.herb.wheel_distance

            # Feed forward the velocities
            if timecount + stepsize > dt:
                stepsize = dt - timecount
            config = config + stepsize*numpy.array([xdot, ydot, tdot])
            if config[2] > numpy.pi:
                config[2] -= 2.*numpy.pi
            if config[2] < -numpy.pi:
                config[2] += 2.*numpy.pi

            footprint_config = config.copy()
            footprint_config[:2] -= start_config[:2]
            footprint.append(footprint_config)

            timecount += stepsize

        # Add one more config that snaps the last point in the footprint to the center of the cell
        nid = self.discrete_env.ConfigurationToNodeId(config)
        snapped_config = self.discrete_env.NodeIdToConfiguration(nid)
        snapped_config[:2] -= start_config[:2]
        footprint.append(snapped_config)

        return footprint

    def GenerateControlResultConfig(self, start_config, control):
        ul = control.ul
        ur = control.ur
        dt = control.dt

        config = start_config.copy()

        xdot = 0.5 * self.herb.wheel_radius * (ul + ur) * numpy.cos(config[2])
        ydot = 0.5 * self.herb.wheel_radius * (ul + ur) * numpy.sin(config[2])
        tdot = self.herb.wheel_radius * (ul - ur) / self.herb.wheel_distance

        config += dt * numpy.array([xdot, ydot, tdot])

        coord = self.discrete_env.ConfigurationToGridCoord(config)
        coord[2] = coord[2] % self.discrete_env.num_cells[2]

        config = self.discrete_env.GridCoordToConfiguration(coord)

        return config



    def PlotActionFootprints(self, idx):

        actions = self.actions[idx]
        fig = pl.figure()
        lower_limits, upper_limits = self.boundary_limits
        pl.xlim([lower_limits[0], upper_limits[0]])
        pl.ylim([lower_limits[1], upper_limits[1]])

        for action in actions:
            xpoints = [config[0] for config in action.footprint]
            ypoints = [config[1] for config in action.footprint]
            pl.plot(xpoints, ypoints, 'k')

        pl.ion()
        pl.show()

    def ConstructActions(self):
        MOVE_DURATION = 0.5

        # Wheel dimensions
        R = self.herb.wheel_radius
        L = self.herb.wheel_distance

        # Actions is a dictionary that maps orientation of the robot to
        #  an action set
        self.actions = dict()

        wc = [0., 0., 0.]
        grid_coordinate = self.discrete_env.ConfigurationToGridCoord(wc)

        # Iterate through each possible starting orientation
        for idx in range(int(self.discrete_env.num_cells[2])+1):
            self.actions[idx] = []
            grid_coordinate[2] = idx
            start_config = self.discrete_env.GridCoordToConfiguration(grid_coordinate)

            # Add four types of actions
            # Move forward
            control = Control(1.,1.,MOVE_DURATION)
            action = Action(control, self.GenerateFootprintFromControl(start_config,control))
            self.actions[idx].append(action)

            # Move backward
            control = Control(-1.,-1.,MOVE_DURATION)
            action = Action(control, self.GenerateFootprintFromControl(start_config,control))
            self.actions[idx].append(action)

            rotate_duration = (self.resolution[2]) / (2. * R / L)
            # Rotate left
            control = Control(-1.,1., rotate_duration)
            action = Action(control, self.GenerateFootprintFromControl(start_config,control))
            self.actions[idx].append(action)

            # Rotate right
            control = Control(1.,-1., rotate_duration)
            action = Action(control, self.GenerateFootprintFromControl(start_config,control))
            self.actions[idx].append(action)

    # Return list of successor objects = node_id + action
    def GetSuccessors(self, node_id):

        successors = []

        coord = self.discrete_env.NodeIdToGridCoord(node_id);
        config = self.discrete_env.NodeIdToConfiguration(node_id);

        # For each action check whether generated footprint is collision free
        for action in self.actions[coord[2]]:
            has_collision = False
            for fp in action.footprint:
                fp_config = fp.copy()
                fp_config[0] += config[0]
                fp_config[1] += config[1]
                if (not self.isValidConfig(fp_config)):
                    has_collision = True
                    break

            if (not has_collision):
                fp_config = self.GenerateControlResultConfig(config, action.control)
                successors.append(Successor(self.discrete_env.ConfigurationToNodeId(fp_config), action))

        return successors

    def isValidConfig(self, config):

        env = self.robot.GetEnv()

        limits_valid = not ((config < self.lower_limits).any() or (config > self.upper_limits).any())

        if limits_valid:
            with env:
                # Check for collision
                pose = self.robot.GetTransform()
                pose[0:3,3] = numpy.array([config[0], config[1], 0.0])

                a = config[2]
                rot = numpy.array([[numpy.cos(a), -numpy.sin(a),0.0],[numpy.sin(a),numpy.cos(a),0.0],[0.0,0.0,1.0]])
                pose[0:3,0:3] = rot

                self.robot.SetTransform(pose)
                collide_free = not (env.CheckCollision(self.robot, self.table))
        else:
            collide_free = False

        returnVal = limits_valid and collide_free

        return returnVal



    def ComputeDistance(self, start_id, end_id):
        # Compute XY euclidean distance

        dist = 0

        start_config = self.discrete_env.NodeIdToConfiguration(start_id)[:1]
        end_config = self.discrete_env.NodeIdToConfiguration(end_id)[:1]

        dist = numpy.linalg.norm(end_config - start_config)

        return dist

    def ComputeHeuristicCost(self, start_id, goal_id):

        # Calculates Manhattan distance as heuristic measure
        cost = 0

        start_coord = self.discrete_env.NodeIdToGridCoord(start_id)
        goal_coord  = self.discrete_env.NodeIdToGridCoord(goal_id)

        diffCoord = goal_coord - start_coord

        for i in range(len(diffCoord)):
            cost = cost + abs(diffCoord[i])
            cost = cost * self.discrete_env.resolution[i]

        return cost



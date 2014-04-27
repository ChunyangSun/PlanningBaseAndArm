import numpy as np

try:
    print "[DiscreteEnvironment] Successfully found cdecimal library."
    import cdecimal as dec
except:
    print "[DiscreteEnvironment] Couldn't find cdecimal module on system, reverting to decimal."
    import decimal as dec   # consider cdecimal for future implementations (considerably faster)

class DiscreteEnvironment(object):

    def __init__(self, resolution, lower_limits, upper_limits):

        MAX_UNITS = 20
        dim = len(lower_limits)
        dim_res = len(str(resolution))

        # if (resolution < 0.1) and ((dim*dim_res) > MAX_UNITS):
        if ((dim*dim_res) > MAX_UNITS):
            self.USE_DECIMAL_CLASS = True
            print "[DiscreteEnvironment] Using Decimal Class..."
        else:
            self.USE_DECIMAL_CLASS = False

        if self.USE_DECIMAL_CLASS:
            self.dtype = np.dtype(dec.Decimal)
        else:
            self.dtype = np.dtype(float)

        # Store the resolution         
        self.resolution = self.to_dec(resolution)    # Resolution now works with very large range
        dec.getcontext().prec = 28

        # Store the bounds
        self.lower_limits = self.to_dec(lower_limits) #np.array(lower_limits)
        self.upper_limits = self.to_dec(upper_limits) #np.array(upper_limits) #np.array(upper_limits,dtype=self.dtype)
        self.lim_ranges = self.upper_limits - self.lower_limits
        self.bounds_epsilon = self.to_dec(0.01)*self.resolution

        # Calculate the dimension
        self.dimension = len(self.lower_limits)

        # Figure out the number of grid cells that are in each dimension
        self.num_cells = self.dimension*[0]
        for idx in range(self.dimension):
            self.num_cells[idx] = np.ceil((upper_limits[idx] - lower_limits[idx])/resolution)

        self.dtype_num_cells = self.to_dec(self.num_cells)

        # Grid to index basis
        self.basis_len = self.to_dec(100.0)/self.resolution #max(self.to_dec(100.0)/self.resolution,max(self.num_cells))
        if self.USE_DECIMAL_CLASS:
            self.basis_len = self.basis_len.to_integral()
        else:
            self.basis_len = round(self.basis_len)


        self.dtype_basis_len = self.to_dec(self.basis_len)
        self.basis = self.basis_len**self.to_dec(range(self.dimension))
        self.basis = self.basis[::-1]

        # config = np.array([ 5.65, -1.76, -0.26,  1.96, -1.15 , 0.87, -1.43 ]); node_id = self.ConfigurationToNodeId(config); config2 = self.NodeIdToConfiguration(node_id)
        # config = np.array([ 4.6, -1.76, 0.00, 1.96, -1.15, 0.87, -1.43]); node_id = self.ConfigurationToNodeId(config); config2 = self.NodeIdToConfiguration(node_id)
        # config = np.array([3.0, 0.0]); node_id = self.ConfigurationToNodeId(config); config2 = self.NodeIdToConfiguration(node_id)
        # coord1=self.ConfigurationToGridCoord(config)
        # coord2=self.NodeIdToGridCoord(node_id)

        # import IPython; IPython.embed()

    def to_dec(self,x):

        USE_DECIMAL_CLASS = self.USE_DECIMAL_CLASS # (self.dtype == np.dtype(dec.Decimal))

        arr_x = np.array(x)

        if arr_x.size > 1:
            dec_x = np.array(x,dtype=dec.Decimal)
            for i in xrange(dec_x.size):
                if USE_DECIMAL_CLASS:
                    dec_x[i] = dec.Context(prec=5, rounding=dec.ROUND_DOWN).create_decimal(dec_x[i])#dec.Decimal(dec_x[i])
                else:
                    dec_x[i] = round(dec_x[i],5) 
        elif not arr_x.shape:
            if USE_DECIMAL_CLASS:
                dec_x = dec.Context(prec=5, rounding=dec.ROUND_DOWN).create_decimal(x)
            else:
                dec_x = round(x,5)
        else:
            if USE_DECIMAL_CLASS:
                dec_x = dec.Context(prec=5, rounding=dec.ROUND_DOWN).create_decimal(arr_x[0])
            else:
                dec_x = round(arr_x[0],5)

        return dec_x

    def ConfigurationToNodeId(self, config):

        node_id = self.GridCoordToNodeId(self.ConfigurationToGridCoord(config))
        return node_id

    def NodeIdToConfiguration(self, nid):
        
        config = self.GridCoordToConfiguration(self.NodeIdToGridCoord(nid))
        return config
        
    # Grid coords start index at 0 (i.e. range of [0,num_cells[n]-1])
    def ConfigurationToGridCoord(self, config):
        
        config_diff = self.to_dec(config)-self.lower_limits
        dtype_coord = config_diff/self.lim_ranges*self.dtype_num_cells

        for i in xrange(dtype_coord.size): 
            if self.dtype == np.dtype(dec.Decimal):
                dtype_coord[i] = dtype_coord[i].to_integral()
            else:
                dtype_coord[i] = round(dtype_coord[i])
        
        coord = dtype_coord.astype(int)
        # coord = np.array(dtype_coord,dtype=int)

        # coord = np.floor((np.array(config)-np.array(self.lower_limits))/self.lim_ranges*self.num_cells)
        # coord = np.floor(scaled_diff)
        return coord

    def GridCoordToConfiguration(self, coord):
        
        dtype_config = self.to_dec(coord)/self.dtype_num_cells*self.lim_ranges+self.lower_limits
        # config = Decimal(np.array(coord))/self.num_cells*self.lim_ranges+np.array(self.lower_limits)

        # Ensure config is within limits

        # for i in xrange(dtype_config.size):
        #     if  dtype_config[i] > self.upper_limits[i]:   
        #         dtype_config[i] = self.upper_limits[i] - self.bounds_epsilon
        #         print dtype_config[i] 
        #     elif dtype_config[i] < self.lower_limits[i]: 
        #         dtype_config[i] = self.lower_limits[i] + self.bounds_epsilon
        #         print dtype_config[i] 

        config = dtype_config.astype(float)

        return config

    def GridCoordToNodeId(self,coord):
        
        dtype_node_id = np.sum(self.to_dec(abs(np.array(coord)))*self.basis)
        node_id = dtype_node_id     # np.int(dtype_node_id)
        # node_id = sum(Decimal(np.array(coord))*Decimal(self.basis))
        return node_id

    def NodeIdToGridCoord(self, node_id):

        dtype_coord = node_id/self.basis%self.dtype_basis_len
        for i in xrange(dtype_coord.size): 
            if self.dtype == np.dtype(dec.Decimal):
                dtype_coord[i] = dtype_coord[i].to_integral()
            else:
                dtype_coord[i] = round(dtype_coord[i])

        coord = dtype_coord.astype(int) #coord = dtype_coord.astype(self.dtype)
        # coord = np.floor(Decimal(node_id/self.basis)%self.basis_len)
        return coord



class DiscreteEnvironment_dtype_float(object):

    def __init__(self, resolution, lower_limits, upper_limits):

        # Store the resolution         
        self.resolution = resolution    # Resolution shouldn't be lower than 0.1 for 7DOF (in terms of order of magnitude), otherwise very large numbers will result during calculations

        # Store the bounds
        self.lower_limits = lower_limits
        self.upper_limits = upper_limits
        self.lim_ranges = np.array(self.upper_limits) - np.array(self.lower_limits)

        # Calculate the dimension
        self.dimension = len(self.lower_limits)

        # Figure out the number of grid cells that are in each dimension
        self.num_cells = self.dimension*[0]
        for idx in range(self.dimension):
            self.num_cells[idx] = np.ceil((upper_limits[idx] - lower_limits[idx])/resolution)

        # Grid to index basis
        self.basis_len = max(self.num_cells)
        self.basis = self.basis_len**range(self.dimension)

    def to_dec(self,x):

        return np.array(x,dtype=float)

    def ConfigurationToNodeId(self, config):

        node_id = self.GridCoordToNodeId(self.ConfigurationToGridCoord(config))
        return node_id

    def NodeIdToConfiguration(self, nid):
        
        config = self.GridCoordToConfiguration(self.NodeIdToGridCoord(nid))
        return config
        
    # Grid coords start index at 0 (i.e. range of [0,num_cells[n]-1])
    def ConfigurationToGridCoord(self, config):
        
        coord = np.floor((np.array(config)-np.array(self.lower_limits))/self.lim_ranges*self.num_cells)
        return coord

    def GridCoordToConfiguration(self, coord):
        
        config = np.array(coord)/self.num_cells*self.lim_ranges+np.array(self.lower_limits)
        return config

    def GridCoordToNodeId(self,coord):
        
        node_id = sum(np.array(coord)*self.basis)
        return node_id

    def NodeIdToGridCoord(self, node_id):

        coord = np.floor((node_id/self.basis)%self.basis_len)
        return coord        
       
# Appendix

# Debugging configs 
# self.ConfigurationToNodeId([1,0.5])
# self.ConfigurationToNodeId([ 4.6, -1.76, 0.00, 1.96, -1.15, 0.87, -1.43])   

# import IPython; IPython.embed()
# config = [ 0.52359878, -1.97222204, -2.74016692, -0.87266462, -4.79965543, -1.57079632, -3.00196630]
# config = [ 5.7595865,  1.9722220,  2.7401669,  3.1415926,  1.3089969, 1.5707963,  3.0019663]
# config = 
# config = 
#config = [ 5.65, -1.76, -0.26,  1.96, -1.15 , 0.87, -1.43 ]
#config = [ 0.64, -1.76,  0.26,  1.96,  1.16,  0.87,  1.43 ]
        
#coord = np.array([  1.,  0, 0,  0, 0,  6,  7])
#node_id = sum(np.array(coord)*self.basis)
#coord = np.floor(np.floor(node_id/self.basis)%self.basis_len)

# config1 = [ 0.556, -1.12, -1.23, -0.50, -3.51, -1.01, -2.05]
# import IPython; IPython.embed()
# #config1 = [1,0.5]
# node_id = self.ConfigurationToNodeId(config1)
# config2 = self.NodeIdToConfiguration(node_id)

# goal_config = numpy.array([ 4.6, -1.76, 0.00, 1.96, -1.15, 0.87, -1.43] )
# goal_config = numpy.array([3.0, 0.0])
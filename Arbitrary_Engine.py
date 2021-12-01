###PACKAGE IMPORTS
from tudatpy.kernel.numerical_simulation import environment
from numpy import sin, cos, pi, array, dot, ndarray
from tudatpy.kernel.math import interpolators
from tudatpy.kernel.astro import frame_conversion
###SETUP IMPORTS FROM OTHER MODULES
from SetUp import Falcon_9_engine, initial_time, guidance_nodes, time_nodes, mass_rate_nodes


class FalconEngineGuided:
    falcon9_engine_spf = Falcon_9_engine

    def __init__(self,
                 bodies : environment.SystemOfBodies,
                 Rocket : environment.Body,
                 M_total_initial: float,
                 M_propallant: float,
                 ):
        self.Thrust_level = (self.falcon9_engine_spf['F_sl'] + self.falcon9_engine_spf["F_vac"] )/ 2
        self.ISP_level = (self.falcon9_engine_spf['isp_sl']+self.falcon9_engine_spf['isp_vac'])/2
        self.Rocket_M_0 = M_total_initial
        self.Rocket_M_prop = M_propallant
        self.Rocket = Rocket
        self.bodies = bodies
        self.prop_margin = M_propallant * 0.01
        self.turn_rate = self.falcon9_engine_spf["turn_rate"]

        #place holder
        self.time_interval = 20
        self.guidance_mass_rate ={}


    def Thrust_Guidance(self, time):
        #get thrust direction from libary of thrust directions
        angle = self.thrust_angle_interpolator.interpolate(time)
        # Set thrust in vertical frame and transpose it
        "this will only work for a  2D take of in the x and y plane"
        current_thrust_direction = array([[sin(angle[0])],              #x-componend
                                         [cos(angle[0])],               #y-compnend
                                         [0]])                          #z-componend
       # todo add matrix that changes the angles to be relative to the current vehicle position


        """
        transform some angles to 
        """
        return current_thrust_direction

    def set_parameters(self, parameters):
        # check if problem is specified correctly
        if (time_nodes > guidance_nodes != 0) or (time_nodes > mass_rate_nodes != 0):
            print("time nodes cannot be more than guidance nodes or mass notes")

        ## SET GUIDANCE NODES
        time_stamp = initial_time
        # self.guidance_angles = {}
        self.guidance_angles = {time_stamp: array([0.5 * pi])}  # todo is the angle with respect to inertial frame
        if time_nodes == guidance_nodes:
            for t in enumerate(parameters[-time_nodes::]):
                time_stamp += t[1]
                self.guidance_angles[time_stamp] = array([parameters[t[0]]])
        elif time_nodes < guidance_nodes and time_nodes != 0:
            steps = round(guidance_nodes / time_nodes)
            rest = guidance_nodes - time_nodes * steps
            count = 0
            for t in parameters[-time_nodes::]:
                for t2 in range(steps):
                    time_stamp += (t2 + 1) * t
                    self.guidance_angles[time_stamp] = array([parameters[count]])
                    count += 1
            for t3 in range(rest):
                time_stamp += (t3 + 1) * parameters[-1]
                self.guidance_angles[time_stamp] = array([parameters[count]])
                count += 1
        elif time_nodes == 0:
            time_interval = 30  # [s] #todo remove or change this value
            for angle in parameters[:guidance_nodes]:
                time_stamp += time_interval
                self.guidance_angles[time_stamp] = array([angle])

        # create interpolator for guidance angles
        interpolator_settings = interpolators.linear_interpolation(
            lookup_scheme=interpolators.binary_search,
            boundary_interpolation=interpolators.use_boundary_value
        )
        # Create the interpolator between nodes and set it as attribute
        self.thrust_angle_interpolator = interpolators.create_one_dimensional_vector_interpolator(
            self.guidance_angles,
            interpolator_settings
        )

        ## SET MASS RATE NODES
        self.guidance_mass_rate = {}
        time_stamp = initial_time
        if mass_rate_nodes == time_nodes:
            for time in enumerate(parameters[-time_nodes::]):
                time_stamp += time[1]
                self.guidance_mass_rate[time_stamp] = array([parameters[mass_rate_nodes + time[0]]])
        elif mass_rate_nodes < time_nodes:
            pass  # todo mass rate is than same for first few nodes
        elif mass_rate_nodes == 0:
            pass  # todo mass rate is constant until it breaks of.
        # todo set up look up scheme that keeps the mass rate constant
    """
            # Set arguments as attributes
            parameters[::-1].sort()
            self.parameter_vetcor = parameters
            self.time_interval = 162/5
            # Prepare dictionary for thrust angles
            self.parameters = {0: array([0.5 * pi])}
    
            # Initialize time
            time_stamp = initial_time + self.time_interval
            # Loop over nodes
            for angle in parameters:
                # Store time as key, thrust angle as value
                self.parameters[time_stamp] = array([angle])
                # Increase time
                time_stamp += self.time_interval
            # Create interpolator settings
            interpolator_settings = interpolators.linear_interpolation(
                lookup_scheme=interpolators.binary_search,
                boundary_interpolation=interpolators.use_boundary_value
            )
    
            # Create the interpolator between nodes and set it as attribute
            self.thrust_angle_interpolator = interpolators.create_one_dimensional_vector_interpolator(
                self.parameters,
                interpolator_settings
            )
    """


    def Thrust_Magnitude(self, time):
        current_mass = self.Rocket.mass
        current_pressure = self.Rocket.flight_conditions.pressure
        current_airspeed = self.Rocket.flight_conditions.airspeed
        current_mach = self.Rocket.flight_conditions.mach_number
        if current_mass >= self.Rocket_M_0 - self.Rocket_M_prop + self.prop_margin:
            return self.Thrust_level
        elif current_mass < self.Rocket_M_0 - self.Rocket_M_prop + self.prop_margin:
            return 0.0

    def ISP_Magnitude(self, time):
        """
        This function might not even be used if a custom mass function is used
        :param time:
        :return:
        """
        return self.ISP_level

    def Mass_Rate_Function(self, time):
        pass



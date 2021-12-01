###MAIN ROCKET MOTION MODULE
import tudatpy.kernel.numerical_simulation.environment_setup
from tudatpy.kernel.numerical_simulation import environment_setup
from tudatpy.kernel.numerical_simulation import propagation_setup, SingleArcSimulator
from tudatpy.kernel.interface import spice_interface
import numpy as np
from numpy import array, pi, zeros, ones
from tudatpy.kernel.constants import GRAVITATIONAL_CONSTANT, JULIAN_DAY

# SETUP INTERFACE
from SetUp import M_prop, propellant_margin
'''
needed to ensure that the standard objects can be loaded in
'''
spice_interface.load_standard_kernels()


class SSTOAscent:
    # IMPORTS FROM SETUP -> ROCKET MOTION SETUP
    from SetUp import Bodies_to_generate, Body_set_as_origin, Orientation_to_use, dt
    # SET UP FOR THE CELESTIAL BODIES IN THE MODEL
    body_settings = environment_setup.get_default_body_settings(Bodies_to_generate,
                                                                Body_set_as_origin,
                                                                base_frame_orientation=Orientation_to_use)
    bodies = environment_setup.create_system_of_bodies(body_settings)
    Origin = Body_set_as_origin

    # TIME STAND START
    StartEpoch = 0.0

    # CHECK WHETHER MODULES ARE PROPERLY INTEGRATED
    Status_aerodynamic_model = None
    Status_thrust_model = None

    # SET UP INITIAL STATE FROM EQUATOR ON Y AXIS AT SPEED EQUAL TO ROTATION OF THE EARTH
    initial_state = array([spice_interface.get_average_radius(Origin),  # x position w.r.t Earth
                           0,  # y position w.r.t Earth
                           0,  # z position w.r.t Earth
                           0,  # x velocity w.r.t Earth
                           spice_interface.get_average_radius(Origin) * (2 * pi / JULIAN_DAY),
                           # y velocity w.r.t Earth
                           0])  # z velocity w.r.t Earth
    Outputs_names = ["Altitude",
                     f"Relative velocity w.r.t {Origin}",
                     "Airspeed",
                     f"Total acceleration w.r.t {Origin}",
                     "{vehicle name} mass",
                     "Altitude at periapsis"
                     ]

    # SET UP MODEL BOUNDARY CONDITIONS
    from SetUp import TargetAltitude, t_max_simulation
    t_max = t_max_simulation
    Target_Altitude = TargetAltitude
    Target_velocity = np.sqrt(GRAVITATIONAL_CONSTANT * bodies.get_body(Origin).mass / (
            spice_interface.get_average_radius(Origin) + Target_Altitude))
    Target_Conditions = f"orbit at {Target_Altitude/1E3} km, with an orbital velocity of {round(Target_velocity)} m/s"

    def __init__(self, Ascent_vehicle: str,
                 Vehicle_initial_mass: float,
                 Vehicle_propellant_mass: float
                 ):
        # create the ascent vehicle
        SSTOAscent.bodies.create_empty_body(Ascent_vehicle)
        SSTOAscent.bodies.get_body(Ascent_vehicle).set_constant_mass(Vehicle_initial_mass)

        self.bodies = SSTOAscent.bodies
        self.Rocket = self.bodies.get_body(Ascent_vehicle)
        self.M_0_rocket = Vehicle_initial_mass
        self.M_prop_rocket = Vehicle_propellant_mass
        self.vehicle = Ascent_vehicle

        # imports from SetUp model independent from other modules


        self.orbit = False


        # list of standard output variables that have to be saved
        """
        add more variables via the add_output_to_save
        """
        self.Outputs_to_save = [
            propagation_setup.dependent_variable.altitude(Ascent_vehicle, SSTOAscent.Origin),  # 0th column
            propagation_setup.dependent_variable.relative_velocity(Ascent_vehicle, SSTOAscent.Origin),  # 1-3 columns
            propagation_setup.dependent_variable.airspeed(Ascent_vehicle, SSTOAscent.Origin),  # 4th column
            propagation_setup.dependent_variable.total_acceleration(Ascent_vehicle),  # 5th-7th column
            propagation_setup.dependent_variable.body_mass(Ascent_vehicle),  # 8th column
            propagation_setup.dependent_variable.periapsis_altitude(Ascent_vehicle, SSTOAscent.Origin)  # 9 column
        ]
        self.Outputs_names[4] = f"{self.vehicle} mass"

    # _______________________________________________________________________________________________________________________

    ### BUILDING FUNCTIONS FOR MODEL DYNAMICS ###
    """"
        ALL FUNCTION REQUIRED TO SET UP THE DYNAMICS OF THE TUDAT MODEL. THIS EXCLUDES THE MODEL SETTINGS, WHICH 
        ARE UNDER THE NEXT CHAPTER. 
        function on can find here are dealing with the interaction of other modules/classes and their 
        underlying interactions. 
        this includes:
        -acceleration settings (completed for now)
        -mass rate settings (uncompleted)
        -aerodynamic settings (uncompleted now constant cd and no cl)
        -propagation settings (completed)
        """

    ### SETUP PROPAGATIONS FOR THE ROCKET MOTION MODULE
    # TODO | STATUS : COMPLETED
    def get_propagation_model(self):
        Acceleration_model = propagation_setup.create_acceleration_models(
            self.bodies,
            self.Acceleration_settings,
            [self.vehicle],
            [self.Body_set_as_origin]
        )
        Mass_Rate_model = propagation_setup.create_mass_rate_models(
            self.bodies,
            self.Mass_Rate_settings,
            Acceleration_model
        )
        Acceleration_propagation_settings = propagation_setup.propagator.translational(
            [self.Body_set_as_origin],
            Acceleration_model,
            [self.vehicle],
            self.initial_state,
            self.EndConditions
        )
        Mass_Rate_propagation_settings = propagation_setup.propagator.mass(
            [self.vehicle],
            Mass_Rate_model,
            np.array([self.M_0_rocket]),
            self.EndConditions
        )

        self.Propagation_settings = propagation_setup.propagator.multitype(
            [Acceleration_propagation_settings, Mass_Rate_propagation_settings],
            self.EndConditions,
            self.Outputs_to_save
        )
        return

    ### ACCELERATION FORCES FOR THE ROCKET MOTION
    # TODO | STATUS : COMPLETED
    def get_acceleration_settings(self):
        if self.Status_thrust_model == None:
            print("No Thrust Model was given")
        if self.Status_aerodynamic_model == None:
            print("NO AERODYNAMIC MODEL WAS GIVEN")
        # todo make the aerodynamic model a custom model dependent on inputs like angle of attack and mach number
        Acceleration_settings_on_vehicle = self.Thrust_settings
        for i in self.Bodies_to_generate:
            if i == "Earth":
                added_settings = {i: [propagation_setup.acceleration.point_mass_gravity(),
                                      propagation_setup.acceleration.aerodynamic()]
                                  }
            else:
                added_settings = {i: [propagation_setup.acceleration.point_mass_gravity()]}
            Acceleration_settings_on_vehicle = {**Acceleration_settings_on_vehicle, **added_settings}

        self.Acceleration_settings = {self.vehicle: Acceleration_settings_on_vehicle}
        return

    # INTEGRATION OF AERODYNAMIC MODULE WITH THE ROCKET MODULE
    # TODO | STATUS : UNCOMPLETED
    def set_aerodynamic_model(self):
        # todo currently use constant drag coefficient however this should be changed to a fucntion module
        from SetUp import A_ref, C_d0
        aero_coefficient_settings = environment_setup.aerodynamic_coefficients.constant(
            A_ref,
            [C_d0, 0, 0]
        )
        environment_setup.add_aerodynamic_coefficient_interface(
            self.bodies, self.vehicle, aero_coefficient_settings
        )
        self.Status_aerodynamic_model = "provided"  # check to see if modules are communicating
        return

    # INTERGRATION OF AERODYNAMIC MODULE WITH THE ROCKET MODULE
    # TODO | STATUS : SEMI-COMPLETED (must only intake the class of the engine)
    def set_thrust_model(self, ThrustMagnitude, Thrustguidance, ThrustISP, Thrust_class: object):
        # todo later just have the input the thurst class
        Thrust_magnitude = propagation_setup.thrust.custom_thrust_magnitude(ThrustMagnitude, ThrustISP)
        Thrust_guidance = propagation_setup.thrust.custom_thrust_direction(Thrustguidance)
        Thrust_model = propagation_setup.acceleration.thrust_from_direction_and_magnitude(
            thrust_magnitude_settings=Thrust_magnitude,
            thrust_direction_settings=Thrust_guidance
        )
        self.Thrust_settings = {self.vehicle: [Thrust_model]}
        self.Status_thrust_model = "provided"  # check to see if modules are communicating
        self.ThrustClass = Thrust_class
        return

    ### MASS RATE FOR ROCKET MOTION
    # TODO | STATUS : UNCOMPLETED
    def get_mass_rate_settings(self):
        # todo make into a custom function that gets it mass flow from the engineclass
        self.Mass_Rate_settings = {self.vehicle: [propagation_setup.mass_rate.from_thrust()]}
        return

    # ______________________________________________________________________________________________________________________

    ###FUNCTIONS FOR MODEL CONDITIONS ###
    """
        CONDITIONS THAT NEED NOT BE CHANGED 
        this everything that is constant between runs and forms the bases for the trajectory. 
        It needs no interaction with other classes/modules, yet can still be modified using the fuctions. 

        things included here are : 
        -termination conditions
        -initial state
        """

    ### GET INTEGRATION SETTINGS
    # TODO | STATUS : PLACE HOLDER (still figure out which intergrator is better)
    def get_integrator_settings(self):
        self.Integrator_settings = propagation_setup.integrator.runge_kutta_4(
            self.StartEpoch,
            self.dt
        )
        return

    ### GET TERMINATION SETTINGS
    # TODO | STATUS : COMPLETED
    def get_terminationConditions(self):
        '''sets the conditions for the mission without having to specify anything in the main document it is
        all automatically extracted fro the setup file. Currently the model still has a mass clause but this should be
        removed in future updates.'''
        # TERMINATION CONDITIONS REGARDLESS FROM MISSION SPECIFICS
        Mass_Termination = propagation_setup.propagator.dependent_variable_termination(
            dependent_variable_settings=propagation_setup.dependent_variable.body_mass(self.vehicle),
            limit_value=self.M_0_rocket - self.M_prop_rocket,
            use_as_lower_limit=True,
            terminate_exactly_on_final_condition=False
        )
        Lower_altitude_stop = propagation_setup.propagator.dependent_variable_termination(
            dependent_variable_settings=propagation_setup.dependent_variable.altitude(self.vehicle, self.Origin),
            limit_value=0.0,
            use_as_lower_limit=True,
            terminate_exactly_on_final_condition=False
        )
        Time_termination = propagation_setup.propagator.time_termination(
            self.StartEpoch + self.t_max,
            terminate_exactly_on_final_condition=False
        )

        # TERMINATION CONDITIONS DUE TO MISSION SPECIFICATIONS
        Orbit_termination = propagation_setup.propagator.custom_termination(self.OrbitTerminationCondition
                                                                            )

        # SETUP COMBINED TERMINATION CONDITIONS
        End_Conditions = propagation_setup.propagator.hybrid_termination([Mass_Termination,
                                                                          Lower_altitude_stop,
                                                                          Time_termination,
                                                                          Orbit_termination],
                                                                         fulfill_single_condition=True)
        self.EndConditions = End_Conditions
        return End_Conditions

    def OrbitTerminationCondition(self, time):
        """
        only used in function get_termination_conditions to check whether rocket has reached the orbit.
        Currently works on orbit velocity and current altitude.

        :param time: required input for tudat not used in the fucntion
        :return: bool whether rocket is in the target orbit
        """
        current_altitude = self.Rocket.flight_conditions.altitude
        current_state = self.Rocket.state
        current_pos = current_state[0:3]
        current_vel = current_state[3:6]
        current_angle = np.arctan(current_pos[1] / current_pos[0])  # y/x
        current_tangential_velocity = current_vel[0] * np.sin(current_angle) + current_vel[1] * np.cos(current_angle)

        if current_altitude >= self.Target_Altitude and current_tangential_velocity >= self.Target_velocity:
            self.orbit = True
            return True
        elif current_altitude < self.Target_Altitude or current_tangential_velocity < self.Target_velocity:
            return False

    ### INITIAL CONDITIONS FOR THE ROCKET MOTION MODULE
    # TODO | STATUS : COMPLETED
    def add_initial_state(self, optional_starting_state="LEFT EMPTY"):
        """"
        allows to change the standard take off state position and velocity
        """
        if optional_starting_state != "LEFT EMPTY":
            self.initial_state = optional_starting_state
        return self.initial_state

    ### FUNCTION TO ADD OUTPUT VARIABLES TO STANDARD VARIABLES
    # TODO | STATUS : COMPLETED
    def add_outputs_to_save(self, outputs_to_add: list, added_names: list, replace=False):
        """
        adds outputs to save to the standard outputs
        or replaces the entire outputs to save if replace = true
        standard outputs to save are class variables
        """
        if replace is False:
            if all(isinstance(x, propagation_setup.dependent_variable.SingleDependentVariableSaveSettings) for x in
                   outputs_to_add):
                self.Outputs_to_save = self.Outputs_to_save + outputs_to_add
                self.Outputs_names = self.Outputs_names + added_names
        elif replace is True:
            if all(isinstance(x, propagation_setup.dependent_variable.SingleDependentVariableSaveSettings) for x in
                   outputs_to_add):
                self.Outputs_to_save = outputs_to_add
                self.Outputs_names = added_names

    # ______________________________________________________________________________________________________________________

    ### FUNCTIONS TO ANALYSE THE ASCENTS AND CLASS
    """
    functions to better anaylse the class that has been created not to be used in the main file
    """

    ### FUNCTION TO RUN ONE INSTANCE OF THE FLIGHT
    # TODO | STATUS : UNCOMPLETED (need to be such that all inputs for guidance can be used)
    def run_simulation(self, new_parameters):
        # TODO SIMULATE THE ASCENT OF THE ROCKET IN TUDAT
        self.get_acceleration_settings()
        self.get_mass_rate_settings()
        self.get_propagation_model()
        self.get_integrator_settings()

        current_bodies = lambda: self.bodies
        current_propagator_settings = lambda: self.Propagation_settings
        current_integrator_settings = lambda: self.Integrator_settings
        self.ThrustClass.set_parameters(new_parameters)
        dynamic_simulator = SingleArcSimulator(current_bodies(),
                                               current_integrator_settings(),
                                               current_propagator_settings(),
                                               print_dependent_variable_data=False
                                               )

        self.current_states = np.vstack(list(dynamic_simulator.state_history.values()))
        self.current_output_variables = np.vstack(list(dynamic_simulator.dependent_variable_history.values()))
        self.time_range = list(dynamic_simulator.state_history.keys())
        max_altitude = max(self.current_output_variables[:, 0])
        return [max_altitude]

    ### FUNCTION TO PLOT THE OUTPUT VARIABLES IN THE GRAPHS
    # TODO | STATUS : NOT DEVELOPED (nice to have)
    def plot_output_data(self):
        pass

    ### OPTIMISATION PROBLEM FOR PYGMO
    """" 
    CONTAINS ALL CLASSES AND FUNCTION NEEDED TO PROVIDE A CURRENT PROBLEM TO PYGMO. 
    
    contains :
    -the problem class
    -function to call the current problem
    """

    ### FUNCTIONS TO GET THE PROBLEM CLASS FOR PYGMO
    # TODO | STATUS : NOT DEVELOPED YET
    def get_problem_class(self, current_vehicle_M_0, current_vehicle_M_prop):
        """

        :return: should return the current problem class with given setttings of the model.
        """
        #check for present thrust and aerodynamic model
        if any(isinstance(x, type(None)) for x in [self.Status_thrust_model, self.Status_aerodynamic_model]):
            print(
                f"Thrust model is {self.Status_thrust_model} and aerodynamic model is {self.Status_aerodynamic_model}")
            return

        #set current masses
        SSTOAscent.bodies.get_body(self.vehicle).set_constant_mass(current_vehicle_M_0)
        self.M_0_rocket = current_vehicle_M_0
        self.M_prop_rocket = current_vehicle_M_prop

        #get model settings
        self.get_terminationConditions()
        self.get_integrator_settings()

        #set model dynamics
        self.get_mass_rate_settings()
        self.get_acceleration_settings()
        self.get_propagation_model()

        #construct the current problem class
        current_problem = self.AscentOptimisation(self.bodies, self.Integrator_settings, self.Propagation_settings, self.ThrustClass)
        self.Current_problem = current_problem
        return current_problem

    class AscentOptimisation:
        def __init__(self, bodies: tudatpy.kernel.numerical_simulation.environment.SystemOfBodies,
                     integrator_settings,
                     propagator_settings,
                     thrust_class,
                     ):
            self.bodies_function = lambda: bodies
            self.integrator_function = lambda: integrator_settings
            self.propagator_function = lambda: propagator_settings
            self.simulation_function = lambda: None
            self.ThrustClass_function = lambda: thrust_class

            #set boundary conditions
            [self.lower_bounds, self.upper_bounds] = self.set_bounds()


        def fitness(self, x: list):
            #update the parameters
            self.ThrustClass_function().set_parameters(x)

            dynamic_simulator = SingleArcSimulator(self.bodies_function(),
                                                   self.integrator_function(),
                                                   self.propagator_function(),
                                                   print_dependent_variable_data=False
                                                   )

            self.Dynamic_simulator = lambda: dynamic_simulator
            self.current_states = np.vstack(list(dynamic_simulator.state_history.values()))
            self.current_output_variables = np.vstack(list(dynamic_simulator.dependent_variable_history.values()))
            self.time_range = list(dynamic_simulator.state_history.keys())
            max_altitude = max(self.current_output_variables[:, 0])

            return [-max_altitude]


        def get_bounds(self):
            return (self.lower_bounds , self.upper_bounds)


        def set_bounds(self):
            from SetUp import guidance_nodes, time_nodes, mass_rate_nodes, altitude_nodes, t_max_simulation
            lower_bounds = list(zeros(guidance_nodes + mass_rate_nodes + time_nodes))
            upper_bounds = list(ones(guidance_nodes)*0.5*pi) \
                        + list(ones(mass_rate_nodes) * M_prop * 0.001)\
                        + list(ones(time_nodes)*t_max_simulation*0.3/time_nodes) #todo change to t_duration previous iteration and change mass limit
            return lower_bounds, upper_bounds
    # __________________________________________________________________________________________________________________

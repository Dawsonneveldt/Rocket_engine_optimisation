###MAIN ROCKET MOTION MODULE
import tudatpy.kernel.numerical_simulation.environment_setup
from tudatpy.kernel.numerical_simulation import environment_setup
from tudatpy.kernel.numerical_simulation import propagation_setup, SingleArcSimulator
from tudatpy.kernel.interface import spice_interface
import numpy as np
from tudatpy.kernel.constants import GRAVITATIONAL_CONSTANT

# SETUP INTERFACE
'''needed to ensure that the standard objects can be loaded in'''
spice_interface.load_standard_kernels()


class SSTOAscent:
    # IMPORTS FROM SETUP -> ROCKET MOTION SETUP
    from SetUp import Bodies_to_generate, Body_set_as_origin, Orientation_to_use, dt
    body_settings = environment_setup.get_default_body_settings(Bodies_to_generate,
                                                                Body_set_as_origin,
                                                                base_frame_orientation=Orientation_to_use)
    bodies = environment_setup.create_system_of_bodies(body_settings)
    Origin = Body_set_as_origin

    # time stamp start
    StartEpoch = 0.0

    # CHECK WHETHER MODULES ARE PROPERLY INTEGRATED
    Status_aerodynamic_model = None
    Status_thrust_model = None
    Status_initial_state = None
    Status_setup = None

    def __init__(self, Ascent_vehicle: str,
                 Vehicle_initial_mass: float,
                 Vehicle_propellant_mass: float,
                 Output_variables_to_save : list
                 ):
        SSTOAscent.bodies.create_empty_body(Ascent_vehicle)
        SSTOAscent.bodies.get_body(Ascent_vehicle).set_constant_mass(Vehicle_initial_mass)

        self.bodies = SSTOAscent.bodies
        self.Rocket = self.bodies.get_body(Ascent_vehicle)
        self.M_0_rocket = Vehicle_initial_mass
        self.M_prop_rocket = Vehicle_propellant_mass
        self.vehicle = Ascent_vehicle
        self.Outputs_to_save = Output_variables_to_save


        #imports from SetUp model independent from other modules
        from SetUp import TargetAltitude, t_max_simulation
        self.t_max = t_max_simulation
        self.Target_Altitude = TargetAltitude
        self.orbit = False
        self.Target_velocity = np.sqrt(GRAVITATIONAL_CONSTANT*self.bodies.get_body(SSTOAscent.Origin).mass/(spice_interface.get_average_radius(SSTOAscent.Origin)+self.Target_Altitude))

### SETUP PROPAGATIONS FOR THE ROCEKT MOTION MODULE
    #TODO | STATUS : COMPLETED
    def get_propagation_model(self):
        if self.Status_initial_state == None:
            print("No initial position and velocities were provided")
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
    #TODO | STATUS : COMPLETED
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
    def set_thrust_model(self, ThrustMagnitude, Thrustguidance, ThrustISP , Thrust_class : object):
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
        print("Thrust class main", Thrust_class)
        print("Thrust class SSTO class", self.ThrustClass)
        return

### MASS RATE FOR ROCKET MOTION
    #TODO | STATUS : UNCOMPLETED
    def get_mass_rate_settings(self):
        #todo make into a custom function that gets it mass flow from the engineclass
        self.Mass_Rate_settings = {self.vehicle:[propagation_setup.mass_rate.from_thrust()]}
        return

### GET INTERGRATION SETTINGS
    #TODO | STATUS : PLACE HOLDER (still figure out which intergrator is better)
    def get_integrator_settings(self):
        self.Integrator_settings = propagation_setup.integrator.runge_kutta_4(
            self.StartEpoch,
            self.dt
        )
        return

### TERMINATION CONDITIONS FOR THE ROCKET MOTION MOUDEL
    #TODO | STATUS : COMPLETED
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
            terminate_exactly_on_final_condition = False
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
        current_altitude = self.Rocket.flight_conditions.altitude
        current_state = self.Rocket.state
        current_pos = current_state[0:3]
        current_vel = current_state[3:6]
        current_angle = np.arctan(current_pos[1]/current_pos[0]) #y/x
        current_tangential_velocity = current_vel[0] * np.sin(current_angle) + current_vel[1] * np.cos(current_angle)

        if current_altitude >= self.Target_Altitude and current_tangential_velocity >= self.Target_velocity:
            self.orbit = True
            return True
        elif current_altitude < self.Target_Altitude or current_tangential_velocity < self.Target_velocity:
            return False

### INITIAL CONDITIONS FOR THE ROCKET MOTION MODULE
    #TODO | STATUS : COMPLETED (COULD BE IMPROVED to be standard position)
    def set_initial_state(self, position_and_velocity: np.ndarray):
        self.Status_initial_state = "provided"
        self.initial_state = position_and_velocity
        return


    def run_simulation(self, new_turn_rate):
        # TODO SIMULATE THE ASCENT OF THE ROCKET IN TUDAT
        self.get_acceleration_settings()
        self.get_mass_rate_settings()
        self.get_propagation_model()
        self.get_integrator_settings()

        current_bodies = lambda : self.bodies
        current_propagator_settings = lambda : self.Propagation_settings
        current_integrator_settings = lambda : self.Integrator_settings
        self.ThrustClass.set_turn_rate(new_turn_rate)
        print(self.ThrustClass)
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


### FUNCTION TO SET OUTPUT VARIABLES
    #TODO | STATUS : COMPLETED (might be set to automated settings)
    def set_outputs_to_save(self, outputs_to_save: list):
        '''outputs to save'''
        self.Outputs_to_save = outputs_to_save

### OPTIMISATION PROBLEM
    class AscentOptimisation:
        def __init__(self, bodies: tudatpy.kernel.numerical_simulation.environment.SystemOfBodies,
                     integrator_settings,
                     propagator_settings,
                     ThrustClass,

                     ):
            #ThrustClass.set_turn_rate(x)
            print("input thrustclass in problem", ThrustClass)
            self.bodies_function = lambda : bodies
            self.integrator_function = lambda : integrator_settings
            self.propagator_function = lambda : propagator_settings
            self.simulation_function = lambda : None
            self.ThrustClass_problem = ThrustClass
            print("created_thrust_class in problem",self.ThrustClass_problem)



        def fitness(self, x: list):
            print(x[0])
            self.ThrustClass_problem.set_turn_rate(x[0])
            print("thrust class in fitness", self.ThrustClass_problem)


            dynamic_simulator = SingleArcSimulator(self.bodies_function(),
                                                   self.integrator_function(),
                                                   self.propagator_function(),
                                                   print_dependent_variable_data=False
                                                   )

            self.Dynamic_simulator = lambda : dynamic_simulator
            self.current_states = np.vstack(list(dynamic_simulator.state_history.values()))
            self.current_output_variables = np.vstack(list(dynamic_simulator.dependent_variable_history.values()))
            self.time_range = list(dynamic_simulator.state_history.keys())
            max_altitude = max(self.current_output_variables[:, 0])


            return [max_altitude]

        def get_bounds(self):
            return ([0],[5])


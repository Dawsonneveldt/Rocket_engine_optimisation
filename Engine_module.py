###IMPORT PACKAGES###
from tudatpy.kernel.numerical_simulation import propagation_setup
import tudatpy
from tudatpy.kernel.numerical_simulation import environment_setup, environment
import numpy as np

###IMPORT FROM OWN MODULES###
from SetUp import EngineType, EngineSetUp, dt, T_force_const, t_burnout

global EngineType, T_force_const, EngineSetUp, dt, t_burnout


###VERSION 1###
def Thrust_Size(time):
    if time <= t_burnout:
        return T_force_const
    elif time > t_burnout:
        return 0.0


def Thrust_Direction(time):
    return np.array([[1], [0], [0]])


def Isp_Function(time):
    return 300


#### VERSION X###
"""def Get_thrust_Engine(bodies):
    g_EA = bodies.get_body("Earth").gravitational_parameter
    TotalVelocity
    angle_to_aero = g_EA * np.cos(angle_inertialframe) / TotalVelocity
    # Set thrust in vertical frame and transpose it
    thrust_direction_vertical_frame = np.array([[0, np.cos(angle_to_aero), np.sin(angle_to_aero)]]).T
    # Update flight conditions (this is needed to let tudat know to update all variables)
    self.vehicle_body.flight_conditions.update_conditions(time)
    # Get aerodynamic angle calculator
    aerodynamic_angle_calculator = self.vehicle_body.flight_conditions.get_aerodynamic_angle_calculator()
    # Retrieve rotation matrix from vertical to inertial frame from the aerodynamic angle calculator
    vertical_to_inertial_frame = aerodynamic_angle_calculator.get_rotation_matrix_between_frames(
        frame_conversion.AerodynamicsReferenceFrames.vertical_frame,
        frame_conversion.AerodynamicsReferenceFrames.inertial_frame)
    # Compute the thrust in the inertial frame
    thrust_inertial_frame = np.dot(vertical_to_inertial_frame,
                                   thrust_direction_vertical_frame)
    return T_force_const"""

"""def Get_thrust_magnitude(time : float, current_engine_mode : str, bodies : tudatpy.kernel.numerical_simulation.environment_setup.SystemOfBodies) :
    if current_engine_mode == "BREATHINGSTAGE":
        if EngineType == 'PDE':
            IntakeConditions = get_intake_conditions_PDE(time, )

        elif EngineType == 'AEROSPIKE':
            IntakeConditions = get_intake_conditions_PHYBRID(time, )

    elif current_engine_mode == "rocketstage":
        IntakeConditions = RocketStageOxidiserConditons


    return T_force"""


###EXAMPLE FROM LUNAR ASCENT
def get_thrust_acceleration_model_from_parameters(thrust_parameters: list,
                                                  bodies: environment.SystemOfBodies,
                                                  initial_time: float,
                                                  specific_impulse: float) -> \
        tudatpy.kernel.numerical_simulation.propagation_setup.acceleration.ThrustAccelerationSettings:
    """
    Creates the thrust acceleration models from the LunarAscentThrustGuidance class and sets it in the propagator.

    Parameters
    ----------
    thrust_parameters : list of floats
        List of thrust parameters.
    bodies : tudatpy.kernel.simulation.environment_setup.SystemOfBodies
        System of bodies present in the simulation.
    initial_time : float
        The start time of the simulation in seconds.
    specific_impulse : float
        Specific impulse of the vehicle.

    Returns
    -------
    tudatpy.kernel.simulation.propagation_setup.acceleration.ThrustAccelerationSettings
        Thrust acceleration settings object.
    """
    # Create Thrust Guidance object
    thrust_guidance = LunarAscentThrustGuidance(bodies.get_body('Vehicle'),
                                                initial_time,
                                                thrust_parameters)
    # Retrieves thrust functions
    thrust_direction_function = thrust_guidance.get_current_thrust_direction
    thrust_magnitude_function = thrust_guidance.get_current_thrust_magnitude
    # Set thrust functions in the acceleration model
    thrust_direction_settings = propagation_setup.acceleration.custom_thrust_direction(thrust_direction_function)
    thrust_magnitude_settings = propagation_setup.acceleration.custom_thrust_magnitude(thrust_magnitude_function,
                                                                                       lambda time: specific_impulse)
    # Create and return thrust acceleration settings
    return propagation_setup.acceleration.ThrustAccelerationSettings(thrust_direction_settings,
                                                                     thrust_magnitude_settings)


###____________________________________________________________________________________###

'''first attempt of thrust model
def Engine_thrust(EngineType, NumberEngines)
    if EngineType = 'PDE':
        #todo follow procedure for PDE engines
        Thrust = "not yet implemented"
    elif EngineType = 'AEROSPIKE':
        #todo follow proceduce for aerospike engine
        Thrust = "not yet implemented"
    elif EngineType = 'HYBRID':
        #todo follow procedure for Pre-cooled hybrid airbreathing engine
        Thrust = "not yet implemented"
    return Thrust'''

### Rocket Motion module ###

"""Here the set up of the general model is made including the optimisation tool to reach teh best trajectory
end point should be set at 200-400 km orbit """
###IMPORTS FROM MODULES###
import numpy as np
import matplotlib.pyplot as plt
import tudatpy.kernel.numerical_simulation as tp
from tudatpy.kernel.interface import spice_interface
from tudatpy.kernel.numerical_simulation import environment_setup
from tudatpy.kernel.numerical_simulation import propagation_setup
from tudatpy.kernel.astro import element_conversion
import tudatpy.kernel.numerical_simulation.propagation_setup.propagator
from tudatpy.kernel.numerical_simulation import estimation_setup

###IMPORTS OWN MODULES###
# IMPORT FROM SETUP
from SetUp import TargetAltitude, EngineSetUp, T_force_const, M_0, Isp_const, C_d0, A_ref, t_max_simulation, M_dry

# IMPORT FROM ENGINE MODULE
from Engine_module import Thrust_Direction, Thrust_Size, Isp_Function

### SETUP ENVIROMENT AND INTERFACE ###
# interface set up
spice_interface.load_standard_kernels()  # check what other states do and whether this is the best option

# start and end conditions
StartEpoch = 0.0
Altitude_termination = propagation_setup.propagator.dependent_variable_termination(
    dependent_variable_settings=propagation_setup.dependent_variable.altitude("ROCKET", 'Earth'),
    limit_value=TargetAltitude,
    use_as_lower_limit=False,
    terminate_exactly_on_final_condition=False
)
TimeTermination = propagation_setup.propagator.time_termination(
    StartEpoch + t_max_simulation,
    terminate_exactly_on_final_condition=True,
)
Mass_Termination =  propagation_setup.propagator.dependent_variable_termination(
    dependent_variable_settings=propagation_setup.dependent_variable.body_mass("ROCKET"),
    limit_value=M_dry,
    use_as_lower_limit=True,
    terminate_exactly_on_final_condition=False

)
# todo add circular velocity termination term see get_termination_settings in LunarAscentUtilities

EndCondition = propagation_setup.propagator.hybrid_termination([Altitude_termination, TimeTermination, Mass_Termination],
                                                               fulfill_single_condition=True)

###DEFINE BODIES ###
BodiesToCreate = ["Earth"]
GlobalOrigin = BodiesToCreate[0]
GlobalOrientation = "J2000"
BodyToPropagate = ["ROCKET"]
OriginalRocketMass = np.array(M_0)

body_settings = environment_setup.get_default_body_settings(BodiesToCreate, GlobalOrigin,
                                                            base_frame_orientation=GlobalOrientation)
body_settings.get("Earth").ephemeris_settings = environment_setup.ephemeris.constant(np.zeros(6), "SSB", "J2000")
bodies = environment_setup.create_system_of_bodies(body_settings)

### define the ROCKET ###
bodies.create_empty_body("ROCKET")
bodies.get_body("ROCKET").set_constant_mass(M_0)

### INITITAL STATE SETUP ###
InitialState_rocket = [spice_interface.get_average_radius("Earth"), 0, 0]  # cartesian X Y Z
InitialState_Earth = [0, 0, 0]
InitialState = np.array(InitialState_rocket + InitialState_Earth)  # contains the coordinates for the rocket and earth



#______________________END OF BODIES SETUP___________________

### SETUP ACCELERATION AND MASS RATE MODEL ###
"MODEL SHOULD HERE INTRODUCE ALL FORCES (THRUST, AERODYNAMIC AND GRAVITY FORCES) ACTING ON THE VEHICLES. " \
"FURTHERMORE THE MASS CHANGE OF THE VEHICLE IS ALSO INTRODUCED HERE"

    #SETUP THRUST MAGNITUDE
"currently set at levels of falcon 9 and no thrust variation"
thrust_magnitude = (propagation_setup.thrust.constant_thrust_magnitude(thrust_magnitude=T_force_const,
                                                                       specific_impulse=Isp_const ))

Thrust_Magnitude = propagation_setup.thrust.custom_thrust_magnitude(Thrust_Size, Isp_Function)


    #SETUP THRUST DIRECTION
"""currently we only fly in x direction"""
ThrustDirection = propagation_setup.thrust.custom_thrust_direction(Thrust_Direction)

    #SETUP AERODYNAMIC FORCES
"CURRENTLY SET TO BE CONTSTANT"
aero_coefficient_settings = environment_setup.aerodynamic_coefficients.constant(
    A_ref,
    [C_d0, 0, 0]
)
environment_setup.add_aerodynamic_coefficient_interface(
    bodies, BodyToPropagate[0], aero_coefficient_settings
)

    ##COMBINE AND SETUP TOTAL ACCELERATIONS MODEL##

acceleration_settings_on_ROCKET = dict(
    ROCKET=[propagation_setup.acceleration.thrust_from_direction_and_magnitude(
        thrust_magnitude_settings=Thrust_Magnitude,
        thrust_direction_settings=ThrustDirection
    )
    ],
    Earth=[propagation_setup.acceleration.point_mass_gravity(),
           propagation_setup.acceleration.aerodynamic()
           ]
)
acceleration_settings = dict(ROCKET=acceleration_settings_on_ROCKET)

    #SETUP MASS RATE
# setup mass propagation
MassRateSettings = dict(ROCKET=propagation_setup.mass_rate.from_thrust())

print(type(MassRateSettings["ROCKET"]))

#_____________________END OF FORCE MODELS_______________________#

###SETUP PROPAGATIONS ###
# define variables to to store

dependent_variables_to_save = [
    propagation_setup.dependent_variable.altitude("ROCKET", "Earth"),  # 0th column
    propagation_setup.dependent_variable.relative_velocity("ROCKET", "Earth"), #1-3 colums
    propagation_setup.dependent_variable.airspeed("ROCKET", "Earth"), #4th colum
    propagation_setup.dependent_variable.total_acceleration("ROCKET"), #5th-7th column
    propagation_setup.dependent_variable.body_mass("ROCKET")

]

CentralBody = [GlobalOrigin]

# Set up accelerations propagations
AccelerationModel = propagation_setup.create_acceleration_models(
    bodies,
    acceleration_settings,
    BodyToPropagate,
    CentralBody
)

PropagationSettings = propagation_setup.propagator.translational(CentralBody,
                                                                 AccelerationModel,
                                                                 BodyToPropagate,
                                                                 InitialState,
                                                                 EndCondition,
                                                                 output_variables=dependent_variables_to_save
                                                                 )

"currently we haven't figured it out if we can use this function"
"""MassPropagatorSettings = propagation_setup.propagator.mass(BodyToPropagate,
                                                           MassRateSettings,
                                                           OriginalRocketMass,
                                                           EndCondition
                                                           )"""


###SETUP NUMERICAL SIMULATION ###

fixed_step_size = 1.0

IntergratorSettings = propagation_setup.integrator.runge_kutta_4(
    StartEpoch,
    fixed_step_size
)

### SIMULATION ###
DynamicsSimulator = tp.SingleArcSimulator(
    bodies, IntergratorSettings, PropagationSettings
)
# RETRIEVE OUTPUT VARIABLES
states = DynamicsSimulator.state_history

dependent_variables = DynamicsSimulator.dependent_variable_history

### DATA PROCESSING ###

time_seconds = states.keys()
carthersian_positions = np.vstack(list(states.values()))
dependent_variables_list = np.vstack(list(dependent_variables.values()))
altitude_list = dependent_variables_list[:, 0]

###DATA ANALYSIS

plt.rcParams.update({'font.size': 20})
plt.figure(figsize=(20, 10))
plt.plot(time_seconds, altitude_list)
plt.xlabel("time [s]")
plt.ylabel("altitude [m]")

plt.figure(figsize=(20, 17))
plt.title("trajcetory in inertial frame")
plt.ylabel("altitude")
plt.xlabel("Y-plane translation")
plt.plot(carthersian_positions[:, 1], altitude_list)


plt.figure(figsize=(20,17))
plt.title("Velocities")
plt.xlabel("time")
plt.ylabel("velocities")
plt.plot(time_seconds, dependent_variables_list[:, 1], color='blue')
plt.plot(time_seconds, dependent_variables_list[:, 2], color='green')
plt.plot(time_seconds, dependent_variables_list[:, 3], color='red')
plt.plot(time_seconds, dependent_variables_list[:, 4], color="orange")

plt.show()
print(dependent_variables_list[0:5, :])
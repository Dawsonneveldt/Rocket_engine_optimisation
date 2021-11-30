###Set Up###
"""
this script should contain all the variables that are set as constants during the run of the module
"""

### ROCKET MOTION PARAMETERS###
    #MISSION REQUIREMENTS
"terminations conditions for specifications of the mission"
TargetAltitude      = 400E+3    #[m]
M_payload           = 15e3      #[kg]
initial_time        = 0         #[s]

    #TERMINATION CONDITONS AND MODEL SET UP
t_max_simulation    = 1000      #[s]
dt                  = 1.0       #[s]

    #BODIES AND INTERACTIONS
Bodies_to_generate  = ["Earth"]
Orientation_to_use  = "J2000"
Body_set_as_origin  = Bodies_to_generate[0]
#_______________________________________________________________________________________________________________________

### todo: ENGINE PARAMETERS ###
    #Engine coefficients
"""
currently constant until model van be build
"""
T_force_const     = 6806 * 1000     #[N][kg m s^-2]
mass_flow         = 100             #[kg s^-1]
Isp_const         = 283             #[s^-1]


    #falcon9 engine specifications
Falcon_9_engine = dict(isp_vac=312,
                       isp_sl=283,
                       t_burnout = 162,
                       F_sl = 6806 * 1000,
                       F_vac = 7426 * 1000,
                       turn_rate= 0.8)

"""
INITILA START FOR 3 SPECIFIED ENGINE PARAMETERS 
EngineType      = 'PDE' #['AERO', 'HYBRID']
Airbreathing    = True

if EngineType == 'HYBRID':
    PratioTurbine = 3.    #[-]      #arbitrary chosen must be verified

EngineSetUp = {"Engine_type" : "PDE",
               "Oxidiser_intake_p" : 5E+6,
               "Oxidiser_intake_T" : 56,
               "Current_mode" : "BREATHINGSTAGE",
               "Fuel_intake_p" : 5E+6,
               "Fuel_intake_T" : 56
               }
"""
#_______________________________________________________________________________________________________________________


###PYGMO OPTIMISATION PARAMETERS###
    #TRAJECTORY OPTIMISATION
trajectory_populations = 20
trajectory_evolutions = 15
guidance_nodes = 5
time_nodes = 4
altitude_nodes = "not yet implemented" #todo still needs to be build and considered
mass_flow_nodes = 0

    #SIZING OPTIMISATION
"""
Might not even use the pygmo but an optimisation algorithm from economterics
"""
sizing_population = "not yet implemented"
sizing_evolutions = "not yet implemented"
#_______________________________________________________________________________________________________________________


### todo : VEHICLE SIZING PARAMETERS ###
    #falcon 9 sizing parameters
M_0         = 549054.  #[kg]
M_prop      = 4E+5  #[kg]
M_dry       = M_0 - M_prop
A_ref       = 100   #[m^2]
#_______________________________________________________________________________________________________________________


### todo : AERODYNAMIC PARAMETERS ###
    #falcon 9 aerodynamic coefficients
'currently const. until model can be build'
C_l0        = 0.0        #[-]
C_d0        = .25       #[-]
MachBeta    = 25        #[-]
#_______________________________________________________________________________________________________________________



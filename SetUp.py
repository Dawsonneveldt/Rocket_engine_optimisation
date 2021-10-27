###Set Up###
# this script should contain all the variables that are set as constants during the run of the module

#model settings
TargetAltitude      = 200E+3    #[m]
dt                  = 0.01      #[s]
t_max_simulation    = 300       #[s]

#aerodynamic coefficients
'currently const. until model can be build'
C_l0        = 2         #[-]
C_d0        = .25       #[-]
MachBeta    = 25        #[-]

#Engine coefficients
"currently constant until model van be build"
T_force_const     = 6806 * 1000     #[N][kg m s^-2]
mass_flow         = 100             #[kg s^-1]
Isp_const         = 283             #[s^-1]
t_burnout         = 200             #[s]

#Rocket Sizing
M_0         = 549054.  #[kg]
M_prop      = 4E+5  #[kg]
M_dry       = M_0 - M_prop
A_ref       = 100   #[m^2]

#Position of take-off
"most likely has to be changed to kepler elements"
x0      = 1         #[m]
y0      = 2         #[m]
z0      = 2         #[m]


#Engine Set-up
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

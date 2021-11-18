###PACKAGE IMPORTS
from tudatpy.kernel.numerical_simulation import environment
from numpy import sin, cos, pi, array


###SETUP IMPORTS FROM OTHER MODULES
from SetUp import Falcon_9_engine





class Falcon9Engine:
    falcon9_engine_spf = Falcon_9_engine

    def __init__(self, M_0 : float,
                 M_dry : float,
                 bodies : environment.SystemOfBodies,
                 Rocket : environment.Body
                 ):
        self.Thrust_level = (self.falcon9_engine_spf['F_sl'] + self.falcon9_engine_spf["F_vac"] )/ 2
        self.ISP_level = (self.falcon9_engine_spf['isp_sl']+self.falcon9_engine_spf['isp_vac'])/2
        self.Rocket_M_0 = M_0
        self.Rocket_M_dry = M_dry
        self.Rocket = lambda : Rocket
        self.bodies = lambda : bodies
        self.prop_margin = (M_0-M_dry) * 0.01
        self.turn_rate = self.falcon9_engine_spf["turn_rate"]



    def ThrustSize(self, time):
        current_mass = self.Rocket().mass
        current_pressure = self.Rocket().flight_conditions.pressure
        current_airspeed = self.Rocket().flight_conditions.airspeed
        current_mach = self.Rocket().flight_conditions.mach_number
        if current_mass >= self.Rocket_M_dry + self.prop_margin:
            return self.Thrust_level
        elif current_mass < self.Rocket_M_dry + self.prop_margin:
            return 0.0

    def IspThrust(self, time):
        return self.ISP_level

    def Thrust_Guidance(self, time):
        if time <= Falcon_9_engine["t_burnout"]:
            x = sin(
                0.5 * pi - (time / Falcon_9_engine["t_burnout"]) * 0.5 * pi * self.turn_rate)
            y = cos(
                0.5 * pi - 0.5 * pi * (time / Falcon_9_engine["t_burnout"]) * self.turn_rate)
            y = min(y, 1)
            x = max(0, x)
        elif time > Falcon_9_engine["t_burnout"]:
            x = sin(0.5 * pi - 1 * 0.5 * pi * self.turn_rate)
            y = cos(0.5 * pi - 0.5 * pi * 1 * self.turn_rate)
        return array([[x], [y], [0]])

    def set_turn_rate(self, turn_rate : float):
        self.turn_rate = turn_rate
        return



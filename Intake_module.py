from SetUp import EngineType
from Temporary_scripts import Imported_alltitude as IA
if EngineType == 'HYRBID':
    from SetUp import PratioTurbine
    global PratioTurbine


def compression(T, N, P2, P1, kappa): #todo describe this procedure in overleaf
    Tnew = T * (1 + 1/N*((P2/P1)**((kappa-1)/kappa)-1))
    return Tnew
def turbine(P, T2, T1, N, kappa):
    Pnew = P * ((((T2/T1)-1)/N)+1)**(kappa/(kappa-1))
    return Pnew
def poisson(T1, P1, P2, kappa):
    T2 = T1/((P1/P2)**((kappa - 1)/kappa))
    return T2
def unchokednozzle(Tt1, Pt1, Pa, kappa, N):
    T2 = -Tt1*N_nozzle*(1-(Pa/Pt1)**((kappa-1)/kappa)) + Tt1
    return T2


def IntakeEngine(h, EngineType, CurrentBreathingMode, NumberEngines):
    if CurrentBreathingMode == False:
        m_oxidiser = m_stored_oxidiser*NumberEngines
        p_oxidiser = set_intake_pressure_oxidiser
        T_oxidiser = set_intake_temperature_oxidiser
        return m_oxidiser, p_oxidiser, T_oxidiser
    if EngineType == "HYBRID":
        #todo atmospheric and vehicle conditions
        P_0,T_0,rho_0 = IA.altitude(h)
        Tt0 = T_0 * (1 + (kappa_air - 1) / 2 * M ** 2)
        Pt0 = P_0 * (1 + (kappa_air - 1) / 2 * M ** 2) ** (kappa_air / (kappa_air - 1))
        v_flight = M * sqrt(kappa_air * R * Tambient)
        #ram intake

        #heat transfer system

        #Single stage Compressor
        Pt_3 = PratioTurbine * Pt_25
        Tt_3 = compression(Tt25, N_HPC, Pt3, Pt25, kappa_air)



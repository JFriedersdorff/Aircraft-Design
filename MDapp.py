from dataclasses import dataclass
import matplotlib.pyplot as plt
import numpy as np
import streamlit as st

#ISA calculator up to 20000m, returning P,T,rho

def ISA_calculator(h, dT=0.0): 
    g0 = 9.80665 
    R = 287.05
    T0 = 288.15
    P0 = 101325
    a = -0.0065

    if h<=11000:
        T = T0 + a*h + dT
        P = P0*(T/T0)**(-g0/(a*R))
        rho = P/(R*T)

    else:
        P11, T11, h11 = 22632, 216.65, 11000
        T = T11 + dT
        P = P11*np.exp(-g0/(R*T11)*(h-h11))
        rho = P/(R*T)

    return T, P, rho

@dataclass
class AircraftData:
    AR: float=11.0
    B: float=10.0

    #Aerodynamic Coefficients
    Cl_max_landing: float=2.4
    Cl_max_TO: float=1.9
    Cd_0_cruise: float=0.0162
    e_cruise: float=0.7751
    Cd_0_L_u: float=0.0792  # Landing flap, gear down
    e_L: float=0.8661
    Cd_0_L_d: float=0.0617
    Cd_0_TO_d: float=0.0532  # TO flap, gear down
    e_TO: float=0.8141
    Cd_0_TO_u: float=0.0357  # TO flap, gear up

    #Mass fractions
    Beta_cruise: float=1.0
    Beta_TO: float=1.0
    Beta_Landing: float=0.745

    #Requirments
    v_cr: float=242 #ms^-1
    v_app: float=71.44
    L_LF: float=1856
    C_LF: float=0.45
    ROC_min: float=8.128
    h_ROC_min: float=0.0
    h_cr: float=11887.2 #m
    L_TOF: float=2790
    h_2: float=11.0

    #Climb gradient requirements 25.119
    Beta_119: float=0.745
    f_T_119: float=0.95
    grad_119: float=0.032
    h_119: float=0.0

    #Climb gradient requirements 25.121a
    Beta_121a: float=1.0
    f_T_121a: float=0.47
    grad_121a: float=0.0
    h_121a: float=0.0

    #Climb gradient requirements 25.121b
    Beta_121b: float=1.0
    f_T_121b: float=0.47
    grad_121b: float=0.024
    h_121b: float=0.0

    #Climb gradient requirements 25.121c
    Beta_121c: float=1.0
    f_T_121c: float=0.47
    grad_121c: float=0.012
    h_121c: float=0.0

    #Climb gradient requirements 25.121d
    Beta_121d: float=0.9
    f_T_121d: float=0.47
    grad_121d: float=0.021
    h_121d: float=0.0

    #Take-off field length requirement
    L_TOF: float=2790
    h_TO: float=0.0
    h_2: float=11.0
    Cl_2 = ((1/1.13)**2) * Cl_max_TO

    #Design Point
    W_S: float=6250.0
    T_W: float=0.35


class Aerodynamcis():

    @staticmethod
    def Mach_Number(v,h):
        R = 287.05
        T,P,rho = ISA_calculator(h)
        M = v/np.sqrt(1.4*R*T)
        return M

    @staticmethod
    def calc_alpha_T(v, h, B):
        T,P,rho = ISA_calculator(h)
        P0 = 101325
        M = Aerodynamcis.Mach_Number(v, h)
        P_t = P * (1 + 0.2*(M**2))**(1.4/0.4)
        delta_t = P_t/P0
        alpha_T = delta_t * (1 - (0.43 + 0.014*B) * np.sqrt(M))
        return alpha_T

    @staticmethod
    def calc_v_opt(W_S, h, Cl):
        T,P,rho = ISA_calculator(h)
        v = np.sqrt(W_S*(2/rho)*(1/Cl))
        return v

class MatchingDiagram():

    #Vertical Wing Loading Limits:
    def __init__(self, data=AircraftData, W_S_min=100, W_S_max=20000, pts=200):
        self.d = data
        self.W_S = np.linspace(W_S_min, W_S_max, pts)
        self.g0 = 9.80665

    def calc_landing_limits(self):
        T,P,rho = ISA_calculator(0)
        lim_app = (1/self.d.Beta_Landing)*(rho/2)*((self.d.v_app/1.23)**2)*self.d.Cl_max_landing
        lim_FL = (1/self.d.Beta_Landing)*(self.d.L_LF/self.d.C_LF)*rho*self.d.Cl_max_landing*0.5
        return lim_app, lim_FL

    #Horizontal T/W Constraint Lines
    def calc_cruise_constraint(self):
        T,P,rho = ISA_calculator(self.d.h_cr)
        alpha_T = Aerodynamcis.calc_alpha_T(self.d.v_cr, self.d.h_cr, self.d.B)
        q = 0.5*rho*(self.d.v_cr)**2
        T_W = (self.d.Beta_cruise/alpha_T)*((self.d.Cd_0_cruise*q)/(self.d.Beta_cruise*self.W_S) + ((self.d.Beta_cruise*self.W_S)/(np.pi*self.d.AR*self.d.e_cruise*q)))
        return T_W

    def calc_ROC_constraint(self):
        T,P,rho = ISA_calculator(self.d.h_ROC_min)
        Cl_opt = min(np.sqrt(self.d.Cd_0_TO_u*np.pi*self.d.AR*self.d.e_TO), self.d.Cl_max_TO)
        v_opt = Aerodynamcis.calc_v_opt(self.W_S, self.d.h_ROC_min, Cl_opt)
        alpha_T = Aerodynamcis.calc_alpha_T(v_opt, self.d.h_ROC_min, self.d.B)
        T_W = (self.d.Beta_TO/alpha_T) * (np.sqrt((rho*self.d.ROC_min**2)/(self.d.Beta_TO*self.W_S*2) * Cl_opt) + 2 * np.sqrt(self.d.Cd_0_TO_u/(np.pi*self.d.AR*self.d.e_TO)))
        return T_W

    def calc_gradient_constraint(self, Beta, f_T, grad, Cd_0, e, h, config):
        Cl_opt_initial = np.sqrt(Cd_0*np.pi*self.d.AR*e)
        if config=="l":
            Cl_opt = min(self.d.Cl_max_landing, Cl_opt_initial)

        elif config=="t":
            Cl_opt = min(self.d.Cl_max_TO, Cl_opt_initial)

        else:
            Cl_opt = Cl_opt_initial

        v_opt = Aerodynamcis.calc_v_opt(self.W_S, h, Cl_opt)
        alpha_T = Aerodynamcis.calc_alpha_T(v_opt, self.d.h_ROC_min, self.d.B)
        T_W = (1/f_T)*(Beta/alpha_T)*(grad + 2*np.sqrt(Cd_0/(np.pi*self.d.AR*e)))
        return T_W

    def calc_TO_constraint(self):
        T,P,rho = ISA_calculator(self.d.h_TO)
        Cl_2 = self.d.Cl_max_TO*(1/1.13)**2
        v_2 = Aerodynamcis.calc_v_opt(self.W_S, self.d.h_TO, Cl_2)
        alpha_T = Aerodynamcis.calc_alpha_T(v_2, self.d.h_TO, self.d.B)
        T_W = (1/alpha_T)*(1.15*np.sqrt(self.W_S/(self.d.L_TOF*0.85*rho*9.80665*np.pi*self.d.AR*self.d.e_TO)) + (4*self.d.h_2/self.d.L_TOF))
        return T_W

    #Evaluating and Plotting:
    def generate_plot(self):
        fig, ax = plt.subplots(figsize=(10,6))
        W_S_app, W_S_LFL = self.calc_landing_limits()

        ax.axvline(W_S_app, label="Approach speed")
        ax.axvline(W_S_LFL, label="Landing field length")
        ax.plot(self.W_S, self.calc_TO_constraint(), label='Take-off field length')
        ax.plot(self.W_S, self.calc_gradient_constraint(self.d.Beta_119, self.d.f_T_119, self.d.grad_119, self.d.Cd_0_L_d, self.d.e_L, self.d.h_119, "l"), label='Climb rate gradient 25_119')
        ax.plot(self.W_S, self.calc_gradient_constraint(self.d.Beta_121a, self.d.f_T_121a, self.d.grad_121a, self.d.Cd_0_TO_d, self.d.e_TO, self.d.h_121a, "t"), label='Climb rate gradient 25_121a')
        ax.plot(self.W_S, self.calc_gradient_constraint(self.d.Beta_121b, self.d.f_T_121b, self.d.grad_121b, self.d.Cd_0_TO_u, self.d.e_TO, self.d.h_121b, "t"), label='Climb rate gradient 25_121b')
        ax.plot(self.W_S, self.calc_gradient_constraint(self.d.Beta_121c, self.d.f_T_121c, self.d.grad_121c, self.d.Cd_0_cruise, self.d.e_cruise, self.d.h_121c, "c"), label='Climb rate gradient 25_121c')
        ax.plot(self.W_S, self.calc_gradient_constraint(self.d.Beta_121d, self.d.f_T_121d, self.d.grad_121d, self.d.Cd_0_L_u, self.d.e_L, self.d.h_121d, "l"), label='Climb rate gradient 25_121d')
        ax.plot(self.W_S, self.calc_ROC_constraint(), label='Climb rate')
        ax.plot(self.W_S, self.calc_cruise_constraint(), label='Cruise speed')
        ax.plot(self.d.W_S, self.d.T_W, marker='o', markersize=5, color='blue', label="Design Point")
        ax.set_ylim(0,1.0)
        ax.set_xlabel("Wing loading [N/m^2]")
        ax.set_ylabel("Thrust-to-weight ratio [-]")
        ax.legend(bbox_to_anchor=(1.02, 1), loc="upper left")
        fig.tight_layout()
        return fig

    

st.set_page_config(page_title="Aircraft Sizing Tool", layout="wide")
st.title("Aircraft Matching Diagram")

st.sidebar.header("Wing Geometry and Propulsion")
ar = st.sidebar.slider("Aspect Ratio", 6.0, 16.0, 11.0, 0.5)
b = st.sidebar.slider("Bypass Ratio", 5.0, 15.0, 10.0, 0.5)

st.sidebar.header("Aerodynamic Coefficients")
Cl_max_to = st.sidebar.slider("Take-off Cl_max", 0.8, 2.5, 1.9, 0.1)
Cl_max_l = st.sidebar.slider("Landing Cl_max", 1.0, 3.0, 2.4)

st.sidebar.header("Performance")
v = st.sidebar.slider("Cruise Velocity (ms^-1)", 50.0, 300.0, 240.0, 1.0 )
h = st.sidebar.slider("Cruise Altitude (m)", 0.0, 20000.0, 11887.0, 1.0)

st.sidebar.header("Design Point")
w_s = st.sidebar.slider("Wing Loading", 100.0, 20000.0, 6250.0, 10.0)
t_w = st.sidebar.slider("Thrust to Weight ratio", 0.05, 0.6, 0.35, 0.01)

ac_data = AircraftData(AR=ar, B=b, Cl_max_TO=Cl_max_to, Cl_max_landing=Cl_max_l, v_cr=v, h_cr=h, W_S=w_s, T_W=t_w)
diagram = MatchingDiagram(ac_data)
fig = diagram.generate_plot()

st.pyplot(fig)







    

    

    
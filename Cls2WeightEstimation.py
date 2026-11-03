from dataclasses import dataclass
import numpy as np

@dataclass
class WeightEstimationParameters:
    #Whole Aicraft Parameters
    W_dg: float=392423.0 #des. Weight [lb]
    f_l: float=0.745 #landing mass fraction
    @property
    def W_l(self) -> float: #des. landing weight [lb]
        return self.W_dg * self.f_l
    @property
    def N_l(self) -> float: #landing load factor, N_gear*1.5
        return 1.5*(self.N_nw+self.N_mw)
    @property
    def N_z(self) -> float: #Ult. load Factor
        n_lim = (2.1 + (24000/(self.W_dg+10000)))
        if n_lim < 2.5:
            return 2.5*1.5
        elif n_lim > 3.8:
            return 3.8*1.5
        else:
            return n_lim*1.5
    @property
    def V_stall(self) -> float:
        return 1.944*np.sqrt((self.W_dg*0.453592)/(self.Clmax*0.5*1.225*301.0))

    #Wing Parameters
    S_w: float=3240 #Wing Area [ft^2]
    A: float=7.0 #aspect ratio
    t_c: float=0.1 #root t/c
    taper: float=0.313 #taper ratio
    B_w: float=180.12 #wing span [ft]
    sweep: float=0.4363 #c/4 sweep at MAC [rad]
    c_r: float=27.43 #root chord [ft]
    Clmax: float=2.35
    
    #Horiztonal Tail Parameters
    K_uht: float=1.0
    B_h: float=14.6 #htail span [ft]
    S_ht: float=609.7 #tail area [ft^2]
    L_t: float=73.06 #tail length -> wingMAC to htMAC
    @property
    def K_y(self) -> float: #pitchng radius of gyration = 0.3L_t
        return 0.3*self.L_t 
    Sweep_ht: float=0.5585
    A_h: float=0.35 
    S_e: float=0.0 #elevator area

    #Vertical Tail Parameters
    Ht_Hv: float=0.0 #1.0 for T-tail, 0.0 for conventional
    S_vt: float=399.0 #vertical tail area
    sweep_vt: float=0.6632
    @property
    def K_z(self) -> float: #yawing radius of gyration =L_t
        return self.L_t
    A_v: float=1.3 #vtail AR

    #Fuselage
    F_w: float=19.547 #fuselage width
    K_door: float=1.0 #1.0 if no cargo door, 1.06 if one side door, 1.12 if two side doors, 1.12 if aft clamshell door, 1.25 if aft and two side doors
    K_Lg: float=1.0 #1.12 if fuselage mounted main landing gear, 1.0 otherwise
    L: float=151.67 #fuselage structural length
    S_f: float=9769.0 #fuselage wetted area [ft^2]
    D: float=19.547 #fuselage stuctural depth
    @property
    def K_ws(self) -> float:
        return 0.75*((1+2*self.taper)/(1+self.taper))*(self.B_w*np.tan(self.sweep)/self.L)

    #Main LG
    K_mp: float=1.0 #1.26 for kneeling gear, 1.0 otherwise
    L_m: float=10.105 #length of main gear
    N_mw: float=8.0 #No. wheels
    N_mss: float=2.0 #No. shock struts

    #Nose LG
    K_np: float=1.0 #1.15 for kneeling gear, 1.0 otherwise
    L_n: float=10.105 #length of nose gear
    N_nw: float=2 #No. nose wheels

    #Nacelle group
    K_ng: float=1.017 #1.017 for pylon mounted nacelle, 1.0 otherwise
    N_Lt: float=15.39
    N_w: float=9.26
    @property
    def W_ec(self) -> float: #weight of engine
        return 2.331*(self.W_en**0.901)*self.K_p*self.K_tr            
    N_en: float=2.0
    @property
    def S_n(self) -> float: #nacelle wetted area
        return self.N_Lt*np.pi*(self.N_w)
    K_p: float=1.0 #1.4 for prop, 1.0 otherwise
    K_tr: float=1.18 #1.18 for jet with thrust reverser, 1.0 otherwise

    #Engine Controls
    L_Wing: float=30.48 #dist. from cockpit to wing root LE. [ft]
    f_outboard: float=0.35
    @property
    def L_eng(self) -> float: #dist. from wing LE to engine
        return self.f_outboard*0.5*self.B_w
    @property
    def L_ec(self) -> float: #length frm engine front to cockpit (total if multiple engines)
        return self.N_en * (self.L_Wing + self.L_eng) 

    #Starter
    W_en: float=13552.0 #engine weight (of 1 engine)

    #Fuel System
    V_t: float=20246.0 #total fuel vol. [gal]
    V_i: float=0.0 #integral tank vol.
    V_p: float=0.0 #self-sealing "protected" tank vp;.
    N_t: float=3.0 #No. fuel tanks

    #Flight Controls
    N_f: float=5.0 #No. functions performed by controls (~4-7)
    N_m: float=2.0 #No. mech. function (~0-2)
    y_i: float=0.61
    y_o: float=0.98
    @property
    def S_cs(self) -> float: #total area of cntrl
        return ((self.y_o-self.y_i)*self.B_w*0.5) * (self.c_r*(1-(1-self.taper)*((self.y_o+self.y_i)*0.5)))
    @property
    def I_y(self) -> float: #yawing moment of inertia
        return (self.K_z**2)*(self.W_dg/32.174)
    #APU
    W_unistalled: float=540.1 #[lb] 787-9 APU

    #Instruments
    K_r: float=1.0 #1.133 if rociprocating engine, 1.0 otherwise
    K_tp: float=1.0 #1.0 for not turboprop
    N_c: int=14 #No. crew (4 pilots, 10 cabin crew)
    L_f: float=215.22 #Fuselage length [ft]

    #Electrical
    R_kva: float=50.0 #system electrical rating (assumed), 40-60 for transports
    @property
    def L_a(self) -> float: #electrical routing dist. (generators to avionics to cockpit)
        return self.L_ec
    @property
    def N_gen(self) -> float: #No. Generators
        return self.N_en

    #Avionics
    W_uav: float=1000 #assumed range (800-1400lb)

    #Furnishings
    W_c: float=109001.0 #max cargo weight [lb]
    W_payload: float=61000.0 

    #Air-Conditioning
    N_p: float=313 #No. crew + passengers
    @property
    def V_pr(self) -> float: #Vol. pressurised sect.
        return self.L*np.pi*(self.D**2)*0.25

    #misc.
    N_pilots: int=4



class WeightEstimation:

    def __init__(self, data, convergence_Thrshld):
        self.Convergence_Threshold = convergence_Thrshld
        self.d = data
        self.iterations = 0

    def Wing(self):
        return 0.0051*((self.d.W_dg*self.d.N_z)**0.557)*(self.d.S_w**0.649)*(self.d.A**0.5)*(self.d.t_c**-0.4)*((1+self.d.taper)**0.1)*(np.cos(self.d.sweep)**-1.0)*(self.d.S_cs**0.1)

    def Htail(self):
        return 0.0379*self.d.K_uht*((1+self.d.F_w/self.d.B_h)**-0.25)*(self.d.W_dg**0.639)*(self.d.N_z**0.1)*(self.d.S_ht**0.75)*(self.d.L_t**-1.0)*(self.d.K_y**0.704)*(np.cos(self.d.Sweep_ht)**-1.0)*(self.d.A_h**0.166)*((1+self.d.S_e/self.d.S_ht)**0.1)

    def Vtail(self):
        return 0.0026*((1+self.d.Ht_Hv)**0.225)*(self.d.W_dg**0.556)*(self.d.N_z**0.536)*(self.d.L_t**-0.5)*(self.d.S_vt**0.5)*(self.d.K_z**0.875)*(np.cos(self.d.sweep_vt)**-1.0)*(self.d.A_v**0.35)*(self.d.t_c**-0.5)

    def Fuselage(self):
        return 0.328*self.d.K_door*self.d.K_Lg*((self.d.W_dg*self.d.N_z)**0.5)*(self.d.L**0.25)*(self.d.S_f**0.302)*((1+self.d.K_ws)**0.04)*((self.d.L/self.d.D)**0.1)

    def MainLG(self):
        return 0.0106*self.d.K_mp*(self.d.W_l**0.888)*(self.d.N_l**0.25)*(self.d.L_m**0.4)*(self.d.N_mw**0.321)*(self.d.N_mss**-0.5)*(self.d.V_stall**0.1)

    def NoseLG(self):
        return 0.032*self.d.K_np*(self.d.W_l**0.646)*(self.d.N_l**0.2)*(self.d.L_n**0.5*self.d.N_nw**0.45)

    def NacelleGrp(self):
        return 0.6724*self.d.K_ng*(self.d.N_Lt**0.1)*(self.d.N_z**0.119)*(self.d.W_ec**0.611)*(self.d.N_en**0.984)*(self.d.S_n**0.224)

    def EngCtrls(self):
        return 5.0*self.d.N_en + 0.80*self.d.L_ec

    def starter(self):
        return 49.19*(self.d.N_en*self.d.W_en/1000)**0.541

    def FuelSystm(self):
        return 2.405*(self.d.V_t**0.606)*((1+self.d.V_i/self.d.V_t)**-1.0)*(1+self.d.V_p/self.d.V_t)*(self.d.N_t**0.5)
    
    def FlightCntrls(self):
        return 145.9*(self.d.N_f**0.554)*((1+self.d.N_m/self.d.N_f)**-1.0)*(self.d.S_cs**0.2)*((self.d.I_y*(10**-6.0))**0.07)

    def APU(self):
        return 2.2*self.d.W_unistalled    

    def Instruments(self):
        return 4.509*self.d.K_r*self.d.K_tp*(self.d.N_c**0.541)*self.d.N_en*((self.d.L_f+self.d.B_w)**0.5)

    def Hydraulics(self):
        return 0.2673*self.d.N_f*((self.d.L_f+self.d.B_w)**0.937)

    def Electrical(self):
        return 7.291*(self.d.R_kva**0.782)*(self.d.L_a**0.346)*(self.d.N_gen**0.1)

    def Avionics(self):
        return 1.73*(self.d.W_uav**0.983)

    def Furnishings(self):
        return 0.0577*(self.d.N_c**0.1)*(self.d.W_c**0.393)*(self.d.S_f**0.75)

    def AirConditioning(self):
        return 62.36*(self.d.N_p**0.25)*((self.d.V_pr/1000)**0.604)*(self.d.W_uav**0.1)

    def AntiIce(self):
        return 0.002*self.d.W_dg

    def HandlingGear(self):
        return 3*(10**-4)*self.d.W_dg

    def Engines(self):
        return self.d.W_en*self.d.N_en

    def Seats(self):
        return 32.0*(self.d.N_p-self.d.N_pilots) + 60.0*self.d.N_pilots

    def Loos(self):
        return 1.11*((self.d.N_p-self.d.N_c)**1.33)
        
    def Estimate(self):
        return (self.Wing() + self.Htail() + self.Vtail() + self.Fuselage() + self.MainLG() + self.NoseLG() + self.NacelleGrp() + self.EngCtrls() + self.starter() + self.FuelSystm() + self.FlightCntrls() + self.APU() + self.Instruments() + self.Hydraulics() + self.Electrical() + self.Avionics() + self.Furnishings() + self.AirConditioning() + self.AntiIce() + self.HandlingGear() + self.Engines() + self.Seats() + self.Loos())

    def IterateWeight(self):
        converged = False
        while not converged:
            self.iterations += 1
            MTOM0 = self.d.W_dg
            OEM1 = self.Estimate()
            MTOM1 = (OEM1 + self.d.W_payload)/0.645
            print(f"f_OEM: {OEM1/MTOM0}")
            print(f"f_payload: {self.d.W_payload/MTOM0}")
            print(f"total mass frac: {(OEM1+self.d.W_payload)/MTOM0+0.355}")
            if abs((MTOM1-MTOM0)/MTOM0) <= self.Convergence_Threshold:
                converged = True

            self.d.W_dg = MTOM1
            print(f"weight after {self.iterations} is {self.d.W_dg}")

        return self.d.W_dg, self.iterations


Data = WeightEstimationParameters()
Estimation = WeightEstimation(Data, 0.001)
Converged_Weight, N_it = Estimation.IterateWeight()

print(F"The final comverged weight is {Converged_Weight}lb, after {N_it} iterations")


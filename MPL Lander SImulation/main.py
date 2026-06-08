import dynamics 
import animationfunc as plot

import math
import numpy as np

#Vehicle Physical Definitions

InertiaTensorCalcs = dynamics.CalculateMomentofInertia(mass=1, radius=0.1, height=1)

VehicleMass_kg = 1 # mass of the rocket 

InertiaTensorCalcs = dynamics.CalculateMomentofInertia(mass=VehicleMass_kg, radius=0.073, height=1)

MomentInertia_xx = InertiaTensorCalcs[0]  
MomentInertia_yy = InertiaTensorCalcs[1] 
MomentInertia_zz = InertiaTensorCalcs[2] 
ProductOfInertia_xz = InertiaTensorCalcs[3]
ProductOfInertia_zx = InertiaTensorCalcs[4]


vehicle = {
    'mass_kg': VehicleMass_kg,
    'Jxz_kgm2': ProductOfInertia_xz,
    'Jzx_kgm2': ProductOfInertia_zx,
    'Jxx_kgm2': MomentInertia_xx,
    'Jyy_kgm2': MomentInertia_yy,
    'Jzz_kgm2': MomentInertia_zz,
}

#Initialization
u_mps = 0.001
v_mps = 0
w_mps = 0

p_rps = 0.001
q_rps = 0  
r_rps = 0

phi_rad = 0*math.pi/180
theta_rad = 87*math.pi/180 
psi_rad = 0*math.pi/180

Xworld_m = 0
Yworld_m = 0
Zworld_m = 0

InitalStateVector = np.array([
    u_mps, # x-axis velocity in body frame
    v_mps, # y-axis velocity in body frame
    w_mps, # z-axis velocity in body frame
    p_rps, #roll rate in body frame
    q_rps, #pitch rate in body frame
    r_rps, #yaw rate in body frame
    phi_rad, #roll angle in world frame
    theta_rad, #pitch angle in world frame
    psi_rad, #yaw angle in world frame
    Xworld_m, # x-axis position in world frame
    Yworld_m, # y-axis position in world frame
    Zworld_m  # z-axis position in world frame
])

def EulerForwardIntegration(simDuration_s, timestep_s, InitialStateVector, vehicle):

    t_s = np.arange(0, simDuration_s + timestep_s, timestep_s)

    nStates = len(InitialStateVector)

    StateHistory = np.zeros((nStates, len(t_s)))

    # Initial condition
    StateHistory[:, 0] = InitialStateVector

    # Time Increment Loop
    for i in range(len(t_s)-1):

        CurrentState = StateHistory[:, i]
        
        if CurrentState[11] > 2: #Check if we hit the ground
            print(f"Rocket has landed at time {t_s[i]:.2f} seconds.")
            StateHistory = StateHistory[:, :i+1]  # Trim the history to the landing time
            t_s = t_s[:i+1]
            break
        
        # Compute state derivatives
        StateDot = dynamics.SixDOFDynamics(
            t_s[i],
            CurrentState,
            vehicle
        )

        # Euler step
        StateHistory[:, i+1] = (
            CurrentState +
            timestep_s * StateDot
        )

    return t_s, StateHistory



def run():

    simDuration_s = 20
    timestep_s = 0.01

    t_s, StateHistory = EulerForwardIntegration(simDuration_s,timestep_s,InitalStateVector,vehicle)

    #Animate2D(t_s, StateHistory)
    #PlotVerticalVelocity(t_s, StateHistory)
    #plot.PlotFlightPath(t_s, StateHistory)
    plot.PlotAllStates(t_s, StateHistory)

run()
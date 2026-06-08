import math
import numpy as np


def ThrustCurve(t, duration, thrust):
    ramp_time = 0.05 * duration
    steady_start = ramp_time

    if t < ramp_time:
        return (thrust / ramp_time) * t  # ramp up
    elif t <= duration:
        return thrust  # steady
    else:
        return 0
    
def CalculateMomentofInertia(mass,radius,height):
    #simple cylindrical assumption for now, will update later

    jxx = ((mass*height**2)/12)+((mass*radius**2)/4)
    jyy = ((mass*height**2)/12)+((mass*radius**2)/4)
    jzz = (mass*radius**2)/4
    jxz = 0
    jzx = 0

    return jxx, jyy, jzz, jxz, jzx

def SixDOFDynamics(t_s, StateVector, vehicle):

    StateVectordot = np.zeros((12))

    #Unpack State Vector
    u_mps = StateVector[0]
    v_mps = StateVector[1]
    w_mps = StateVector[2]

    p_rps = StateVector[3]
    q_rps = StateVector[4]   
    r_rps = StateVector[5]

    phi_rad = StateVector[6]
    theta_rad = StateVector[7]
    psi_rad = StateVector[8]

    Xworld_m = StateVector[9]
    Yworld_m = StateVector[10]
    Zworld_m = StateVector[11]
    
    #Unpack Vehicle Definitions

    m_kg = vehicle['mass_kg']
    Jxz_kgm2 = vehicle['Jxz_kgm2']
    Jzx_kgm2 = vehicle['Jzx_kgm2']
    Jxx_kgm2 = vehicle['Jxx_kgm2']
    Jyy_kgm2 = vehicle['Jyy_kgm2']
    Jzz_kgm2 = vehicle['Jzz_kgm2']

    #----Calculate Dynamics----#

    #Trig Functions
    sin_phi = math.sin(phi_rad)
    cos_phi = math.cos(phi_rad)
    sin_theta = math.sin(theta_rad)
    cos_theta = math.cos(theta_rad)
    sin_psi = math.sin(psi_rad)
    cos_psi = math.cos(psi_rad)


    #Translational Dynamics

    #Thrust and Aerodynamic Forces in Body Frame
    Fxbody_N = ThrustCurve(t_s, 4, 15) 
    Fybody_N = 0
    Fzbody_N = 0

    ForceVector = np.array([[Fxbody_N],
                            [Fybody_N],
                            [Fzbody_N]])
    
    GravityRotationMatrix = np.array([[-sin_theta],
                                      [sin_phi*cos_theta],
                                      [cos_phi*cos_theta]])
    
    CoriolisVector = np.array([[(r_rps*v_mps)-(q_rps*w_mps)],
                               [(p_rps*w_mps)-(r_rps*u_mps)],
                               [(q_rps*u_mps)-(p_rps*v_mps)]])

    TranslationalAccelerations = (1/m_kg)*ForceVector + 9.81*GravityRotationMatrix + CoriolisVector

    udot_mps2 = TranslationalAccelerations[0,0] #0
    vdot_mps2 = TranslationalAccelerations[1,0] #1
    wdot_mps2 = TranslationalAccelerations[2,0] #2

    #Rotational Dynamics

    # Moments in Body Frame
    Mxbody_Nm = 0
    Mybody_Nm = 0
    Mzbody_Nm = 0

    IntertiaTensor = np.array([[Jxx_kgm2, 0, -Jxz_kgm2], 
                               [0, Jyy_kgm2, 0], 
                               [-Jzx_kgm2, 0, Jzz_kgm2]]) #<---Ts is fried
    
    InverseInertiaTensor = np.linalg.inv(IntertiaTensor)

    MomentVector = np.array([[Mxbody_Nm+((Jyy_kgm2-Jzz_kgm2)*q_rps*r_rps)+(Jxz_kgm2*p_rps*q_rps)],
                             [Mybody_Nm+((Jzz_kgm2-Jxx_kgm2)*p_rps*r_rps)+(Jxz_kgm2*((r_rps**2)-(p_rps**2)))],
                             [Mzbody_Nm+((Jxx_kgm2-Jyy_kgm2)*p_rps*q_rps)-(Jxz_kgm2*q_rps*r_rps)]])
    
    RotationalAccelerations = InverseInertiaTensor @ MomentVector

    pdot_rps2 = RotationalAccelerations[0,0] #3
    qdot_rps2 = RotationalAccelerations[1,0] #4
    rdot_rps2 = RotationalAccelerations[2,0] #5

    #Kinematic Transformation Equationse

    """
    (Note from Ayan)
    The kinematic transformation equations are essienalyl just body rates or measurements transformed into  
    world frame. The translational kinematics transform body frame velocities into world frame velocities, and the rotational kinematics transform body frame angular rates into world frame angular rates.

    So here the TranslationalKinematicsMatrix transforms the body frame velocities (u, v, w) into world frame velocities (xdot, ydot, zdot) that one would observe from the ground (pretty sure)
    The world frame is the earth fixed coordinate system where 
    x points north
    y points east
    z points down

    The RotationalKinematicsMatrix transforms the body frame angular rates (p, q, r) into world frame angular rates (phidot, thetadot, psidot) that one would observe from the ground (pretty sure)
    it gets kinda tricky here because idrk how the order of what we are rotating first second third is but ill get to it later and update this comment    
    """

    #Translational Kinematics

    TranslationalKinematicsMatrix = np.array([[(cos_theta*cos_psi),((sin_phi*sin_theta*cos_psi)-(cos_phi*sin_psi)),((cos_phi*sin_theta*cos_psi)+(sin_psi*sin_phi))],
                                           [(cos_theta*sin_psi),((sin_phi*sin_theta*sin_psi)+(cos_phi*cos_psi)),((cos_phi*sin_theta*sin_psi)-(sin_psi*cos_phi))],
                                           [(-sin_theta),(sin_phi*cos_theta),(cos_phi*cos_theta)]]) #transforms body rates to world rates
    
    BodyTranslationalRatesVector = np.array([[u_mps],
                                             [v_mps],
                                             [w_mps]])
    
    WorldTranslationalVelocities = TranslationalKinematicsMatrix @ BodyTranslationalRatesVector 

    xdot_m = WorldTranslationalVelocities[0,0] #6
    ydot_m = WorldTranslationalVelocities[1,0] #7
    zdot_m = WorldTranslationalVelocities[2,0] #8

    #Rotational Kinematics

    RotationalKinematicsMatrix = np.array([[(1), ((sin_phi*sin_theta)/cos_theta), ((cos_phi*sin_theta)/cos_theta)],
                                           [(0), (cos_psi), (-sin_psi)],
                                           [(0), (sin_psi/cos_theta), (cos_psi/cos_theta)]]) #transforms body rates to world rates
    
    BodyRotationalRatesVector = np.array([[p_rps],
                                           [q_rps],
                                           [r_rps]])
    
    EulerAngleRates = RotationalKinematicsMatrix @ BodyRotationalRatesVector #Same as world frame angular rates
    
    phidot_rps = EulerAngleRates[0,0] #9
    thetadot_rps = EulerAngleRates[1,0] #10
    psidot_rps = EulerAngleRates[2,0] #11

    #Construct StateVectordot

    StateVectordot[0] = udot_mps2
    StateVectordot[1] = vdot_mps2
    StateVectordot[2] = wdot_mps2
    StateVectordot[3] = pdot_rps2
    StateVectordot[4] = qdot_rps2
    StateVectordot[5] = rdot_rps2
    StateVectordot[6] = phidot_rps
    StateVectordot[7] = thetadot_rps
    StateVectordot[8] = psidot_rps
    StateVectordot[9] = xdot_m
    StateVectordot[10] = ydot_m
    StateVectordot[11] = zdot_m
    
    return StateVectordot
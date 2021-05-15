''' Aidan Wallace, University of Maryland, September 2020

This is a preliminary draft for a conflict avoidance algorithm outlined in
Erzberger & Heere's 2010 paper: "Algorithm and operational concept for resolving
short range conflicts" to be implemented into the open source air traffic control
software 'BlueSky' as a plugin.
'''

import numpy as np 
import matplotlib.pyplot as plt 


g = 9.81 #m/s^2
airspeed_a = 110 #airspeed of reference aircraft
b0 = [10000, 10000] #initial coordinates of non-reference aircraft
heading0 = -np.pi/2 #initial heading of non-reference aircraft
airspeed_b = 110 #initial airspeed of non-reference aircraft
d_req = 5*1852 #5NM in meters
bankangle = [15, 30]
turnangles = np.linspace(-180,180,181)

####
# Functions for straight line segment

def get_t_smin(p_at,p_bt,heading_a,heading_b): 
# (18)
    dx1 = p_bt[0] - p_at[0]
    dy1 = p_bt[0] - p_at[1]
    Vrx = airspeed_b*np.sin(heading_b) - airspeed_a*np.sin(heading_a)
    Vrx = airspeed_b*np.cos(heading_b) - airspeed_a*np.cos(heading_a)
    t_smin = -(dx1*Vrx + dy1*Vry)/(Vrx**2 + Vry**2)
    return t_smin


def get_d_smin(p_at,p_bt,heading_a,heading_b):
# (19) 
    dx1 = p_bt[0] - p_at[0]
    dy1 = p_bt[0] - p_at[1]
    Vrx = airspeed_b*np.sin(heading_b) - airspeed_a*np.sin(heading_a)
    Vrx = airspeed_b*np.cos(heading_b) - airspeed_a*np.cos(heading_a)
    d_smin = sqrt( (dx1 - Vrx*((dx1*Vrx + dy1*Vry)/(Vrx**2 + Vry**2)))**2 
                    + (dy1 - Vry*((dx1*Vrx + dy1*Vry)/(Vrx**2 + Vry**2)))**2 )
    return d_smin

####
# Functions for separation during turn


def get_da(t, b0, bankangle): 
# turn separation if only A turns (10)
    R = get_turnradius(bankangle)
    dphi = get_turnangle(t,bankangle)
    da = sqrt( (b0[0] + t*airspeed_b*np.sin(heading0) - R*np.sign(dphi) * (1 - np.cos(dphi))**2)
                + (b0[1] + t*airspeed_b*np.cos(heading0) - R*np.sign(dphi)*np.sin(dphi))**2)
    return da

def get_db(t, b0, bankangle): 
# turn separation if only B turns (11)
    R = get_turnradius(bankangle)
    dphi = get_turnangle(t, bankangle)
    db = sqrt( (b0[0] + R*np.sign(dphi)*np.cos(heading0) - R*np.sign(dphi) * np.cos(heading0 +dphi))**2
                + (b0[1] - t*airspeed_b - R*np.sign(dphi) * np.sin(heading0) + R*np.sign(dphi)*np.sin(heading0 +dphi))**2)
    return db

def get_dab(t, b0, bankangle):
# turn separation in cooperative maneuver (13)
    R_b = get_turnradius(bankangle)
    dphi_b = get_turnangle(t, bankangle)
    R_a = get_turnradius(bankangle)
    dphi_a = dphi_b*airspeed_b/airspeed_a #(4)
    dab = sqrt( (b0[0] + R_b*np.sign(dphi_b)*np.cos(dphi_b) - R_b*np.sign(dphi_b) * np.cos(heading0 + dphi_b) - R_a*np.sign(dphi_a) * (1-np.cos(dphi_a)))**2
                + (b0[1] - R_b*np.sign(dphi_b) * np.sin(heading0) + R_b*np.sign(dphi_b)*np.sin(heading0 + dphi_b) - R_a*np.sign(dphi_a)*np.sin(dphi_a))**2)
    return dab 

####
# Functions for position vectors of aircraft while turning

def get_turnposition_a(t, bankangle): 
# position vector in semicircle for reference aircraft (6)
    R = get_turnradius(bankangle, airspeed_a)
    dphi = get_turnangle(t, bankangle, airspeed_a)
    p_at = R_a*np.sign(dphi)*[1-np.cos(dphi),np.sin(dphi)]
    return p_at

def get_turnposition_b(t, bankangle): 
# position vector in turn of non-reference aircraft (8)
    R = get_turnradius(bankangle, airspeed_b)
    dphi = get_turnangle(t, bankangle, airspeed_b)
    p_bt = [b0[0] + R*np.sign(dphi)*np.cos(headin0) - R*np.sign(dphi)*np.cos(heading0 + dphi),
            b0[1] - R*np.sign(dphi)*np.sin(heading0) + R*np.sign(dphi)*np.sin(heading0 + dphi)]
    return p_bt

####
# Aircraft maneuver constraints

def get_timetoturn(turnangle, bankangle, velocity): 
# returns time to turn to specified turn angle
    t = turnangle*velocity/g/np.tan(bankangle)
    return t

def get_turnangle(t, bankangle,airspeed):
 # (3)
    turnangle = t*g*np.tan(bankangle)/airspeed
    return turnangle

def get_turnradius(bankangle,airspeed):
 # (2)
    turnradius = airspeed**2/g/np.tan(bankangle)
    return turnradius 

#Test aircraft maneuvers (simplifying assumption that coordinates are at the start of maneuver)

    #Calculate d_tmin and corresponding turn angle from eqns. 10,11, or 13

    #If d_tmin >= d_req, a lesser turn angle exists that fulfills separation constraints
        #Minimum separation will occur in line after turn up to turnangle_req
        #turnangle_req is obtained by setting left side of (19) to dreq and solving

        #evaluate d_smin in small increments of turnangle to locate extrema before  it merges with turnangle_tmin

        #if min separation for null maneuver is not zero, first extremum is max or min. min if d_smin is initially decreasing.
        #IF MINIMUM ANGLES RANGING FROM ZERO TO THE MINIMUM ARE ELIMINATED
        #IF TURN ANGLE FOR FIRST MINIMUM OCCURS AT TURNANGLE_TMIN, MANEUVER TYPE IS ABANDONED

    #Fist max can be found at less than turnangle_tmin or when equal to turnangle_tmin
    #IF former
        #compare maximum d_smin with d_req
        #IF d_reeq <= max(d_smin)
            #an angle exists such that d_smin = d_req. unknown angle can be determined by
            #standard iterative procedures for solving nonlinear eqns. 
        #IF d_req > max(d_smin)
            #no solution exists in angle range between min and max of d_smin
        #IF d_req <= d_tmin
            #acceptable solution lies at turnangle = turnangle_tmin
        #ELSE
            #No solution yields sufficient separation

    #IF time to reach d_smin at turnangle_req exceed time to  reach turnangle_tmin by 20%,
    #then
        #IF turnangle_tmin for d_req > d_smin


#sort all 12 maneuvers by order of preference. maneuvers that fail are included and labeled
#as failures

#last resort is to select solution that maximizes minimum separation when both craft are turning
#an end of turn heading is then dound corresponing to d_req

#If this fails, i.e. no turn angle corresponds to d_req, then turns are terminated at max separation


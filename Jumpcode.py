# -*- coding: utf-8 -*-
"""
Created on Mon Nov 25 19:23:28 2019

@author: TANG VE
"""

import odrive
import math

def get_gamma(pos_0,pos_1,cpr):
    rad_0=(pos_0/cpr)*2*math.pi
    rad_1=(pos_1/cpr)*2*math.pi
    alpha = -rad_0 + math.pi/2
    beta = rad_1 - math.pi/2
    gamma=alpha/2- beta/2
    return gamma
    
def get_extension(L1,L2,gamma):
    L=L1*math.cos(gamma)+math.sqrt(L1**2*math.cos(gamma)**2-L1**2+L2**2)
    return L

def extension_to_gamma(L,L1,L2):
    cos_param = (L1**2 + L**2 - L2**2) / (2*L1*L)
    if cos_param<-1:
        gamma=math.pi
    elif cos_param>1:
        gamma=0
    else:
        gamma=math.acos(cos_param)
    return gamma
    

def command_legs(state):
    if state=='soft':
        odrv0.axis0.controller.set_coupled_gains(10,1,10,1)
        odrv0.axis1.controller.set_coupled_gains(10,1,10,1)
        L=neutralHopExt
        gamma=extension_to_gamma(L,L1,L2)
        odrv0.axis0.controller.set_coupled_setpoints(0,gamma)
        odrv0.axis1.controller.set_coupled_setpoints(0,gamma)
    if state=='hard':
        odrv0.axis0.controller.set_coupled_gains(220,0.5,220,0.2)
        odrv0.axis1.controller.set_coupled_gains(220,0.5,220,0.2)
        L=neutralHopExt
        gamma=extension_to_gamma(L,L1,L2)
        odrv0.axis0.controller.set_coupled_setpoints(0,gamma)
        odrv0.axis1.controller.set_coupled_setpoints(0,gamma)

def startstate():
    odrv0.axis0.controller.set_coupled_gains(10,1,10,1)
    odrv0.axis1.controller.set_coupled_gains(10,1,10,1)
    L=neutralHopExt
    gamma=extension_to_gamma(L,L1,L2)
    odrv0.axis0.controller.set_coupled_setpoints(0,gamma)
    odrv0.axis1.controller.set_coupled_setpoints(0,gamma)
        

if __name__ == "__main__":
    cpr=2000
    L1=0.1
    L2=0.2
    minSoftToHardVel = 0.001
    minExtPosErr = 0.01
    minSoftToHardExtErr = 0.05
    neutralHopExt = 0.27
    
    print("finding an odrive...")
    odrv0 = odrive.find_any()
    print("found")
    startstate()
    L_pre=0.27
    
    while(True):
        pos_0=odrv0.axis0.encoder.pos_estimate
        pos_1=odrv0.axis1.encoder.pos_estimate
        gamma=get_gamma(pos_0,pos_1,cpr)
        L=get_extension(L1,L2,gamma)
        V_direction=L-L_pre
        
        if (neutralHopExt - L) < minExtPosErr:
            state='soft'
            command_legs(state)
            print("change to soft")
            L_pre=L
            continue
        elif abs(V_direction) < minSoftToHardVel and (neutralHopExt - L) > minSoftToHardExtErr:
            state='hard'
            command_legs(state)
            print("change to hard")
            L_pre=L
            continue
        else:
            L_pre=L
            continue


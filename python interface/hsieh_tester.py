# -*- coding: utf-8 -*-
"""
Created on Mon Jul 11 14:15:42 2022

@author: Molly
"""
#implementing Hsieh et al. 2019 (check name)

import numpy as np

def calculateFieldValues(OB, H, Positions):
    #calculates 3(HdotX)X/(X^5) - H/X^3; X = P - OB
    #for each row of Positions (P = Positions[j,:])
    
    B= 0*Positions
    for j in range(0, Positions.shape[0]):
        P= Positions[j,:]
        B[j, :]= ((3*(np.dot(H, P-OB))*(P-OB))/(P-OB)^5)-(H/(P-OB)^3)
    
    return B

def findR(B,P):
    #returns the R matrix [B' (BxP)'] for set of measurements B at position P
    #gets us to equation 7
    R= np.hstack([B, np.cross(B, P)])
    
    return R
    

def getV(R):
    #equation 10
    (w,v) = np.linalg.eig(np.transpose(R) @ R)
    ind = np.argmin(np.abs(w))
    v = v[:,ind]
    v = v/np.sqrt(np.dot(v[3:],v[3:]))
    return v

def getrAndH(v):
    #splits v into two vectors, r and H according to eqns 11-12
    H= v[3:]
    r= v[0:3]
    return (r,H)

def calculateGs (r,H,Positions):
    #calculates G2,G1,G0 according to equation 19
    
    G2 = 0*Positions
    G1 = 0*Positions
    G0 = 0*Positions
    
    for j in range(0,Positions.shape[0]):
        P = Positions[j,:]    
        G2[j,:] = 2*H
        G1[j,:] = (-3*P)+(3*(np.cross(r, H)))-((np.cross(H, P))*H)
        G0[j,:] = (3*(np.dot(H, P))*P)-(3*(np.dot(H,P))*(np.cross(r, H)))- ((np.dot(P, P))+ (np.dot(r,r)) - (2*(np.dot(np.cross(r,H) , P))*H))
    return (G2,G1,G0)

def findt (G2,G1,G0,B):
    #find that minimizes square of (G2xB)t^2 + (G1xB)t + G0xB
    
    
    return t

def getPositionAndOrientation(B,P):
    
    
    return B, P
    
    

    


# -*- coding: utf-8 -*-
"""
Created on Tue Jul 12 10:43:39 2022

@author: Molly
"""
import numpy as np


PBoard  = np.array([[0,0,0],
                    [0, 75, 0],
                    [64.951,-37.5,0],
                    [-64.951,-37.5,0],
                    [25,0,0],
                    [-50,0,0],
                    [0,-25,0],
                    [0,50,0]])

def calculateFieldValues(OB, H, Positions = PBoard, Scaling = 7e6):
    #calculates 3(HdotX)X/(X^5) - H/X^3; X = P - OB
    #for each row of Positions (P = Positions[j,:])
    
    B= 0*Positions
    for j in range(0, Positions.shape[0]):
        P= Positions[j,:]
        B[j, :]= ((3*(np.dot(H, P-OB))*(P-OB))/np.linalg.norm(P-OB)**5)-(H/np.linalg.norm(P-OB)**3)
    return B*Scaling

def findR(B,P=PBoard):
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

def calculateGs (r,H,Positions=PBoard):
    #calculates G2,G1,G0 according to equation 19
    
    G2 = 0*Positions
    G1 = 0*Positions
    G0 = 0*Positions
    
    
    for j in range(0,Positions.shape[0]):
        P = Positions[j,:]    
        G2[j,:] = 2*H
        G1[j,:] = (-3*P)+(3*(np.cross(r, H)))-((np.cross(H, P))*H)
        G0[j,:] = (3*(np.dot(H, P))*P)-(3*(np.dot(H,P))*(np.cross(r, H)))- (((np.dot(P, P))+(np.dot(r, r))-((2*(np.dot(np.cross(r,H), P)))))*H)
        
    return (G2,G1,G0)





#XXtest against  findR(B,Pboard):, getV(R):, getrAndH(v): --> H should agree with simulation input; if not, stop and think
    
#if that works, try to finish calculateGs and findt
#test getPositionAndOrientation(B,Pboard) against simulation input
    
#if you get stuck or if you finish, then try adding noise to B,
#e.g. B = B + np.random.random_sample(B.shape)*sigma #sigma is the noise level
#then test to see effect
    
#ultimately what we want is error vs sigma
#pick a sigma, make many noisy fields, measure H, average of Htrue dot Hmeas vs. sigma (0 -->1, 0.1-->0.98?)
#if you had a position average of |Ob - OBtrue| vs. sigma (maybe sqrt(avg(|Ob-OBtrue|^2))) 
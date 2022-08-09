# -*- coding: utf-8 -*-
"""
Created on Mon Jul 11 14:15:42 2022

@author: Molly
"""
#implementing Hsieh et al. 2019 (check name)

import numpy as np
import scipy.optimize
from simulator import plotx
from simulator import makePlot
import matplotlib.pyplot as plt
from itertools import combinations 

def calculateFieldValues(OB, H, Positions, Scaling =1):
    #calculates 3(HdotX)X/(X^5) - H/X^3; X = P - OB
    #for each row of Positions (P = Positions[j,:])
    
    B= 0*Positions
    for j in range(0, Positions.shape[0]):
        P= Positions[j,:]
        B[j, :]= ((3*(np.dot(H, P-OB))*(P-OB))/np.linalg.norm(P-OB)**5)-(H/np.linalg.norm(P-OB)**3)
    return B*Scaling

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




def getPolynomialCoefficients(G2,G1,G0,B):

    g2 = np.cross(G2,B)
    g1 = np.cross(G1,B)
    g0 = np.cross(G0,B)

    g22 = np.dot(g2.flatten(),g2.flatten())
    g21 = np.dot(g2.flatten(),g1.flatten())
    g20 = np.dot(g2.flatten(),g0.flatten())
    
    g11 = np.dot(g1.flatten(),g1.flatten())
    g10 = np.dot(g1.flatten(),g0.flatten())
    
    g00 = np.dot(g0.flatten(),g0.flatten())
    
    c1 = 2*g22 * 1e4
    c2 = 3*g21 * 1e3
    c3 = (2*g20+g11) * 1e2 
    c4 = g10 * 10
    
    # print([c1,c2,c3,c4])

    # c1= 2*(np.dot(g2.flatten(),g2.flatten()))
    # c2= 3*(np.dot(g2.flatten(),g1.flatten()))
    # c3= (2*(np.dot(g2.flatten(),g0.flatten()))) +(np.dot(g1.flatten(),g1.flatten()))
    # c4= np.dot(g0.flatten(), g1.flatten())
    # print([c1,c2,c3,c4])

   
    vectorOfCoefficients = np.array([c1, c2, c3, c4])
    origvector = np.array([g22*1e4, 2*g21*1e3, (2*g20+g11)*1e2, 2*g10*10, g00])
    return  (vectorOfCoefficients, origvector, (g2,g1,g0))
# def findt (G2,G1,G0,B):
#     #find that minimizes square of (G2xB)t^2 + (G1xB)t + G0xB
#     temp = getPolynomialCoefficients(G2,G1,G0,B)
#     (g2,g1,g0) = temp[2]
    
#     g2 = g2.flatten()
#     g1 = g1.flatten()
#     g0 = g0.flatten()
    
#     t=np.zeros(g2.shape[0])
    
#     for j in range(0,g2.shape[0]):
        
#         g22 = np.dot(g2[j,:].flatten(),g2[j,:].flatten())
#         g21 = np.dot(g2[j,:].flatten(),g1[j,:].flatten())
#         g20 = np.dot(g2[j,:].flatten(),g0[j,:].flatten())
        
#         g11 = np.dot(g1[j,:].flatten(),g1[j,:].flatten())
#         g10 = np.dot(g1[j,:].flatten(),g0[j,:].flatten())
        
#         g00 = np.dot(g0[j,:].flatten(),g0[j,:].flatten())
#         polyvec = np.array([g22, 2*g21, (2*g20+g11), 2*g10, g00])
#         roots = np.roots(np.poly1d(polyvec).deriv())
#         t[j] = roots[np.argmin(np.polyval(polyvec,roots))]
    
    
#     return t
def findt (G2,G1,G0,B):
    #find that minimizes square of (G2xB)t^2 + (G1xB)t + G0xB
    temp = getPolynomialCoefficients(G2,G1,G0,B)
    (g2,g1,g0) = temp[2]
    
    g2 = g2.flatten()
    g1 = g1.flatten()
    g0 = g0.flatten()
    
    t=np.zeros((g2.shape[0],2))
    
    for j in range(0,g2.shape[0]):        
        t[j,:] = np.roots((g2[j],g1[j],g0[j]))
        
         
    return t

def fitBTandResiduals (OB,H,B,P):
    Bfit = calculateFieldValues(OB, H, P)
    BT0 = np.nanmedian(B.flatten()/Bfit.flatten())
    BT = scipy.optimize.leastsq(lambda b : b*Bfit.flatten()-B.flatten(), BT0)[0]
    r = BT*Bfit.flatten()-B.flatten()
    return (BT,np.dot(r,r))


def getPositionAndOrientationLeastSquares(B,P,Hinit,OBinit):
    obj = lambda x: (B-calculateFieldValues(x[3:], x[:3], P)).flatten()
    w0 = np.hstack((Hinit,OBinit))
    result = scipy.optimize.least_squares(obj, w0)
    return (result.x[:3], result.x[3:])


def getPositionAndOrientation(B,P):
    R = findR(B,P)
    (r,H) = getrAndH(getV(R))
    (G2,G1,G0) = calculateGs(r,H,P)
    t = findt(G2,G1,G0,B)
    t = t.flatten()
    bt = 0*t
    resid = 0*t
    for j in range(0,len(t)):
        (bt[j],resid[j]) = fitBTandResiduals(np.cross(r,H) +t[j]*H, H, B, P)
    
    ind = np.argmin(resid)
    Hinit = H*bt[ind]
    OBinit = np.cross(r,H) +t[ind]*H

    return(getPositionAndOrientationLeastSquares(B, P, Hinit, OBinit))    
    
def getPositionAndOrientationFile(filename):
    Breading= np.loadtxt('e:\\magreadings726\\' + filename + '.txt')
    Bnomag= np.loadtxt('e:\\magreadings720\\nomagnet.txt')
    B = (Breading-Bnomag)[:,3:]
    P = Bnomag[:,:3]
    (H,OB) = getPositionAndOrientation(B, P)
    M = np.linalg.norm(H)
    H = H/M
    return (OB, H, M)



def calculateResiduals (xcoordinates, sensorinds, filenames):
    #Find positions
    positions= []
    for x in filenames:
        Breading= np.loadtxt('e:\magreadings726_backup\ '+ x+ '.txt')
        Bnomag= np.loadtxt('e:\magreadings720\ nomagnet.txt')
        B = (Breading-Bnomag)[sensorinds,3:]
        P = Bnomag[sensorinds,:3]
        (H,OB) = getPositionAndOrientation(B, P)
        M = np.linalg.norm(H)
        H = H/M
        positions.append(OB)
    OBx0= positions[0]
    OBx025= positions[1]
    OBx05= positions[2]
    OBx075= positions[3]
    OBx1= positions[4]
    OBx125= positions[5]
    OBx15= positions[6]
    OBx175= positions[7]
    OBx2= positions[8]
    OBx225= positions[9]
    OBx25= positions[10]
    OBx275= positions[11]
    OBx3= positions[12]
    OBx325= positions[13]
    OBx35= positions[14]
    OBx375= positions[15]
    OBx4= positions[16]  
    ycoordinates= np.array([OBx0[0], OBx025[0], OBx05[0], OBx075[0], OBx1[0], OBx125[0], OBx15[0], OBx175[0], OBx2[0], OBx225[0], OBx25[0], OBx275[0], OBx3[0], OBx325[0], OBx35[0], OBx375[0], OBx4[0]])
    #plot points and line of best fit
    (fig,ax)= makePlot()
    plotx(OBx0,OBx025,OBx05,OBx075,OBx1,OBx125,OBx15,OBx175,OBx2,OBx225,OBx25,OBx275,OBx3,OBx325,OBx35,OBx375,OBx4)
    a,b= np.polyfit(xcoordinates, ycoordinates, 1)
    plt.plot(xcoordinates, (a*xcoordinates)+b, color='black')
    #calculate and plot residuals
    residuals= ycoordinates-(a*xcoordinates+b)
    (fig,ax)= makePlot()
    plt.plot(xcoordinates, residuals)
    error= np.sqrt(np.sum(residuals**2))/(17-2)
    return error
    

def getPermutations (totalNumSensors, nSensorsToChoose):
    #(sensorSelections, inclusionMatrix) = getPermutations (totalNumSensors, nSensorsToChoose)
    #sensorSelections is list of combinations (e.g. (0,1,3))
    #inclusionMatrix[i,j] is true iff sensor j is included in combination i
    sensorSelections = list(combinations(range(totalNumSensors), nSensorsToChoose))
    inclusionMatrix = np.zeros((len(sensorSelections), totalNumSensors), bool)
    for i in range(len(sensorSelections)):
        inclusionMatrix[i,sensorSelections[i]] = True
    return (sensorSelections, inclusionMatrix)

def filenamesAndPositionsMolly():

        center = ['x0','x025','x05','x075','x1','x125','x15','x175','x2','x225','x25','x275','x3','x325','x35','x375','x4']
        edge = ['edgex0','edgex025','edgex05','edgex075','edgex1','edgex125','edgex15','edgex175','edgex2','edgex225','edgex25','edgex275','edgex3','edgex325','edgex35','edgex375','edgex4']
        offboard = ['offx0','offx025','offx05','offx075','offx1','offx125','offx15','offx175','offx2','offx225','offx25','offx275','offx3','offx325','offx35','offx375','offx4']
        xcoordinates= np.array([0*25.4, 0.025*25.4, 0.05*25.4, 0.075*25.4, 0.1*25.4, 0.125*25.4, 0.15*25.4, 0.175*25.4, 0.2*25.4, 0.225*25.4, 0.25*25.4, 0.275*25.4, 0.3*25.4, 0.325*25.4, 0.35*25.4, 0.375*25.4, 0.4*25.4])

        return(center, edge, offboard, xcoordinates)

def loadAndProcessFullDataSet(filenames):
    Bnomag= np.loadtxt('e:\\magreadings720\\nomagnet.txt')
    Breading = [];
    for filename in filenames:
        Breading.append(np.loadtxt('e:\\magreadings726\\'+ filename + '.txt'))
     
    fullSensorSetH = np.zeros((len(Breading),3))
    fullSensorSetOB = np.zeros((len(Breading),3))
    for j in range(len(Breading)):
        Breading[j][:,3:] = (Breading[j]-Bnomag)[:,3:];
        B = Breading[j][:,3:]
        P = Breading[j][:,:3]
        (H7,OB7) = getPositionAndOrientation(B, P)
        fullSensorSetH[j,:] = H7
        fullSensorSetOB[j,:] = OB7
    
    return (fullSensorSetH, fullSensorSetOB, Breading)


def calculateMSErrorSet (xcoordinates, filenames, nsensors):
    (fullSensorSetH, fullSensorSetOB, Breading) = loadAndProcessFullDataSet(filenames)
    OBf = fullSensorSetOB*0
    p = np.polyfit(xcoordinates, fullSensorSetOB,1)
    fullMS = np.zeros((3,))
    for i in range(3):
        OBf[:,i] = np.polyval(p[:,i],xcoordinates)
        fullMS[i] = np.mean((fullSensorSetOB[:,i]-OBf[:,i])**2)    
    (sensorSelections, inclusionMatrix) = getPermutations (7, nsensors) #hard code number of sensors
    MS_diff_meas = np.zeros((len(sensorSelections),3))
    MS_diff_fullfit = np.zeros((len(sensorSelections),3))
    MS_diff_fit = np.zeros((len(sensorSelections),3))
    for i in range(len(sensorSelections)):
        OB = 0*fullSensorSetOB
        for j in range(len(Breading)):
            B = Breading[j][sensorSelections[i],3:]
            P = Breading[j][sensorSelections[i],:3]
            (__,OB[j,:]) = getPositionAndOrientationLeastSquares(B,P,fullSensorSetH[j,:], fullSensorSetOB[j,:])
        MS_diff_meas[i,:] = np.mean((OB-fullSensorSetOB)**2,0)
        MS_diff_fullfit[i,:] = np.mean((OB-OBf)**2,0)
        
        p = np.polyfit(xcoordinates, OB,1)
        for j in range(3):
            obf_local = np.polyval(p[:,j],xcoordinates)
            MS_diff_fit[i,j] = np.mean((OB[:,j]-obf_local)**2)
    
    
        
    return (inclusionMatrix, MS_diff_meas, MS_diff_fullfit, MS_diff_fit)
    
    

def calculateMSErrorBySensor(inclusionMatrix, MS_error):
    MS_error_sensor = np.zeros((7,3))
    for i in range(7):
        MS_error_sensor[i,:] = np.mean(MS_error[inclusionMatrix[:,i],:],0)
    
    return MS_error_sensor

def calculateMSErrorBySensorExclusion(inclusionMatrix, MS_error):
    MS_error_sensor = np.zeros((7,3))
    for i in range(7):
        MS_error_sensor[i,:] = np.mean(MS_error[np.logical_not(inclusionMatrix[:,i]),:],0)
    
    return MS_error_sensor


def calculateResiduals1 (xcoordinates, sensorinds, filenames):
    #Find positions
    positions= []
    for x in filenames:
        Breading= np.loadtxt('e:\magreadings726\ '+ x+ '.txt')
        Bnomag= np.loadtxt('e:\magreadings720\ nomagnet.txt')
        B = (Breading-Bnomag)[:,3:]
        P = Bnomag[:,:3]
        (H7,OB7) = getPositionAndOrientation(B, P)
    for x in filenames:
        Breading= np.loadtxt('e:\magreadings726\ '+ x+ '.txt')
        Bnomag= np.loadtxt('e:\magreadings720\ nomagnet.txt')
        B = (Breading-Bnomag)[sensorinds,3:]
        P = Bnomag[sensorinds,:3]
        (H,OB) = getPositionAndOrientationLeastSquares(B, P,H7,OB7)
        M = np.linalg.norm(H)
        H = H/M
        positions.append(OB)
    OBx0= positions[0]
    OBx025= positions[1]
    OBx05= positions[2]
    OBx075= positions[3]
    OBx1= positions[4]
    OBx125= positions[5]
    OBx15= positions[6]
    OBx175= positions[7]
    OBx2= positions[8]
    OBx225= positions[9]
    OBx25= positions[10]
    OBx275= positions[11]
    OBx3= positions[12]
    OBx325= positions[13]
    OBx35= positions[14]
    OBx375= positions[15]
    OBx4= positions[16]  
    ycoordinates= np.array([OBx0[0], OBx025[0], OBx05[0], OBx075[0], OBx1[0], OBx125[0], OBx15[0], OBx175[0], OBx2[0], OBx225[0], OBx25[0], OBx275[0], OBx3[0], OBx325[0], OBx35[0], OBx375[0], OBx4[0]])
    #plot points and line of best fit
    (fig,ax)= makePlot()
    plotx(OBx0,OBx025,OBx05,OBx075,OBx1,OBx125,OBx15,OBx175,OBx2,OBx225,OBx25,OBx275,OBx3,OBx325,OBx35,OBx375,OBx4)
    a,b= np.polyfit(xcoordinates, ycoordinates, 1)
    plt.plot(xcoordinates, (a*xcoordinates)+b, color='black')
    #calculate and plot residuals
    residuals= ycoordinates-(a*xcoordinates+b)
    (fig,ax)= makePlot()
    plt.plot(xcoordinates, residuals)
    error= np.sqrt(np.sum(residuals**2))/(17-2)
    return error   

    #(fig,ax)= makePlot()
   # plotx(OBx0, OBx025, OBx05, OBx075, OBx1, OBx125, OBx15, OBx175, OBx2, OBx225, OBx25, OBx275, OBx3, OBx325, OBx35, OBx375, OBx4)
   # a,b= np.polyfit(xcoordinates, ycoordinates, 1)
   # return (a,b)
    #plt.plot(xcoordinates, a*xcoordinates+b)
    #calculate and plot residuals
   # residuals= ycoordinates-(a*xcoordinates+b)
   # (fig, ax)= makePlot()
   # plt.plot(xcoordinates, residuals)
   # error= np.sqrt(np.sum(residuals**2))/(17-2)
   # return error


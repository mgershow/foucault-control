# -*- coding: utf-8 -*-
"""
Created on Tue Jul 12 10:43:39 2022

@author: Molly
"""
import numpy as np
import scipy.optimize
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt

PBoard  = np.array([[0,0,0],
                    [0, 75, 0],
                    [64.951,-37.5,0],
                    [-64.951,-37.5,0],
                    [25,0,0],
                    [-50,0,0],
                    [0,-25,0],
                    [0,50,0]])

def calculateFieldValues(OB, H, Positions = PBoard, Scaling = 1):
    #calculates 3(HdotX)X/(X^5) - H/X^3; X = P - OB
    #for each row of Positions (P = Positions[j,:])
    
    B= 0*Positions
    for j in range(0, Positions.shape[0]):
        P= Positions[j,:]
        B[j, :]= ((3*(np.dot(H, P-OB))*(P-OB))/np.linalg.norm(P-OB)**5)-(H/np.linalg.norm(P-OB)**3)
    return B*Scaling

def obj(x,B,P):
    
    H= x[:3]
    OB= x[3:]
    BSim= calculateFieldValues(OB, H, P)    
    
    return ((B- BSim).flatten())

def getPositionAndOrientationLeastSquares(B,P,Hinit,OBinit):
    w0 = np.hstack((Hinit,OBinit))
    result = scipy.optimize.least_squares(obj, w0, args = (B,P))
    return (result.x[:3], result.x[3:])

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



def findt (G2,G1,G0,B):
    #find that minimizes square of (G2xB)t^2 + (G1xB)t + G0xB
    
    
    return t

def getPositionAndOrientation(B,P):
    
    
    return B, P
#data= np.loadtxt('e:\magreadings\-rotating_100.txt')
#B= data[:,4:]


def getBs(B):
    B0= B[0::7,:]
    B1= B[1::7,:]
    B2= B[2::7,:]
    B3= B[3::7,:]
    B4= B[4::7,:]
    B5= B[5::7,:]
    B6= B[6::7,:]
           
    
    return(B0, B1, B2, B3, B4, B5, B6)
 

def getNewB(B0, B1, B2, B3, B4, B5, B6, magCenters):
    B0_new= B0*0
    B1_new= B0*0
    B2_new= B0*0
    B3_new= B0*0
    B4_new= B0*0
    B5_new= B0*0
    B6_new= B0*0    
    for j in range(0,100):
        B0_new[j,:]= B0[j,:]- magCenters[0]
    for j in range(0,100):
        B1_new[j,:]= B1[j,:]- magCenters[1]
    for j in range(0,100):
        B2_new[j,:]= B2[j,:]- magCenters[2]
    for j in range(0,100):
        B3_new[j,:]= B3[j,:]- magCenters[3]
    for j in range(0,100):
        B4_new[j,:]= B4[j,:]- magCenters[4]
    for j in range(0,100):
        B5_new[j,:]= B5[j,:]- magCenters[5]
    for j in range(0,100):
        B6_new[j,:]= B6[j,:]- magCenters[6]
        
    return (B0_new, B1_new, B2_new, B3_new, B4_new, B5_new, B6_new)
       

def getUnitB(B0_new, B1_new, B2_new, B3_new, B4_new, B5_new, B6_new):
    B0_unit= B0_new*0
    B1_unit= B0_new*0
    B2_unit= B0_new*0
    B3_unit= B0_new*0
    B4_unit= B0_new*0
    B5_unit= B0_new*0
    B6_unit= B0_new*0
  
    for j in range(0,100):
        B0_unit[j,:]= B0_new[j,:]/(np.sqrt(np.dot(B0_new[j,:], B0_new[j,:])))
    for j in range(0,100):
        B1_unit[j,:]= B1_new[j,:]/(np.sqrt(np.dot(B1_new[j,:], B1_new[j,:])))
    for j in range(0,100):
        B2_unit[j,:]= B2_new[j,:]/(np.sqrt(np.dot(B2_new[j,:], B2_new[j,:])))
    for j in range(0,100):
        B3_unit[j,:]= B3_new[j,:]/(np.sqrt(np.dot(B3_new[j,:], B3_new[j,:])))
    for j in range(0,100):
        B4_unit[j,:]= B4_new[j,:]/(np.sqrt(np.dot(B4_new[j,:], B4_new[j,:])))
    for j in range(0,100):
        B5_unit[j,:]= B5_new[j,:]/(np.sqrt(np.dot(B5_new[j,:], B5_new[j,:])))
    for j in range(0,100):
        B6_unit[j,:]= B6_new[j,:]/(np.sqrt(np.dot(B6_new[j,:], B6_new[j,:])))
    
    return (B0_unit, B1_unit, B2_unit, B3_unit, B4_unit, B5_unit, B6_unit)
      

 

def getR(B0_unit, B1_unit, B2_unit, B3_unit, B4_unit, B5_unit, B6_unit):  
    R0= scipy.spatial.transform.Rotation.align_vectors(B0_unit, B0_unit)
    R1= scipy.spatial.transform.Rotation.align_vectors(B0_unit, B1_unit)
    R2= scipy.spatial.transform.Rotation.align_vectors(B0_unit, B2_unit)
    R3= scipy.spatial.transform.Rotation.align_vectors(B0_unit, B3_unit)
    R4= scipy.spatial.transform.Rotation.align_vectors(B0_unit, B4_unit)
    R5= scipy.spatial.transform.Rotation.align_vectors(B0_unit, B5_unit)
    R6= scipy.spatial.transform.Rotation.align_vectors(B0_unit, B6_unit)
    
    return(R0, R1, R2, R3, R4, R5, R6)
    


def makePlot():
    fig= plt.figure()
    ax= plt.axes()
    return (fig, ax)

def testSpiral(ax):
    z= np.linspace(0,1,100)
    x= z*np.sin(25*z)
    y=z*np.cos(25*z)
    ax.scatter(x,y,z)
    
def B0B1Plot(ax, B0_unit, B1_unit):
    xx=B0_unit[:,0]
    yy= B0_unit[:,1]
    zz= B0_unit[:,2]
    ax.cla()
    ax.scatter(xx,yy,zz, c='blue')
    
    xx=B1_unit[:,0]
    yy= B1_unit[:,1]
    zz= B1_unit[:,2]
    ax.scatter(xx,yy,zz, c= 'red')
       
        
def getDotResult(B0, B):
    dotresult = 0*B0
    for j in range(0,100):
        dotresult[j,:]= np.dot(B0[j,:], B[j,:])
        
    return dotresult
        
def fitLocationLSQ(filename, Hinit = np.array([0, 0, 1e6]), OBinit = np.array([0,0,100])):
    Breading= np.loadtxt('e:\magreadings720\ ' + filename +'.txt')
    Bnomag= np.loadtxt('e:\magreadings720\ nomagnet.txt')
    B_mag= Breading-Bnomag
    
    return getPositionAndOrientationLeastSquares(B_mag[:,3:],Breading[:,:3],Hinit,OBinit)
    

def quiverPlot(filename):
    Breading= np.loadtxt('e:\magreadings726\ ' + filename +'.txt')
    Bnomag= np.loadtxt('e:\magreadings720\ nomagnet.txt')
    B_mag= Breading-Bnomag
    
    (fig, ax)= makePlot()
    
    plt.quiver(Breading[:,0], Breading[:,1], B_mag[:, 3], B_mag[:,4])
 



def plotx (OBx0, OBx025, OBx05, OBx075, OBx1, OBx125, OBx15, OBx175, OBx2, OBx225, OBx25, OBx275, OBx3, OBx325, OBx35, OBx375, OBx4):
     
    
     plt.plot(0*25.4, OBx0[0], marker= 'o', markersize= 3, markeredgecolor= 'red', markerfacecolor= 'red')
     plt.plot(0.025*25.4, OBx025[0], marker= 'o', markersize= 3, markeredgecolor= 'red', markerfacecolor= 'red')
     plt.plot(0.05*25.4, OBx05[0], marker= 'o', markersize= 3, markeredgecolor= 'red', markerfacecolor= 'red')
     plt.plot(0.075*25.4, OBx075[0], marker= 'o', markersize= 3, markeredgecolor= 'red', markerfacecolor= 'red')
     plt.plot(0.1*25.4, OBx1[0], marker= 'o', markersize= 3, markeredgecolor= 'red', markerfacecolor= 'red')
     plt.plot(0.125*25.4, OBx125[0], marker= 'o', markersize= 3, markeredgecolor= 'red', markerfacecolor= 'red')
     plt.plot(0.15*25.4, OBx15[0], marker= 'o', markersize= 3, markeredgecolor= 'red', markerfacecolor= 'red')
     plt.plot(0.175*25.4, OBx175[0], marker= 'o', markersize= 3, markeredgecolor= 'red', markerfacecolor= 'red')
     plt.plot(0.2*25.4, OBx2[0], marker= 'o', markersize= 3, markeredgecolor= 'red', markerfacecolor= 'red')
     plt.plot(0.225*25.4, OBx225[0], marker= 'o', markersize= 3, markeredgecolor= 'red', markerfacecolor= 'red')
     plt.plot(0.25*25.4, OBx25[0], marker= 'o', markersize= 3, markeredgecolor= 'red', markerfacecolor= 'red')         
     plt.plot(0.275*25.4, OBx275[0], marker= 'o', markersize= 3, markeredgecolor= 'red', markerfacecolor= 'red')
     plt.plot(0.3*25.4, OBx3[0], marker= 'o', markersize= 3, markeredgecolor= 'red', markerfacecolor= 'red')
     plt.plot(0.325*25.4, OBx325[0], marker= 'o', markersize= 3, markeredgecolor= 'red', markerfacecolor= 'red')
     plt.plot(0.35*25.4, OBx35[0], marker= 'o', markersize= 3, markeredgecolor= 'red', markerfacecolor= 'red')
     plt.plot(0.375*25.4, OBx375[0], marker= 'o', markersize= 3, markeredgecolor= 'red', markerfacecolor= 'red')
     plt.plot(0.4*25.4, OBx4[0], marker= 'o', markersize= 3, markeredgecolor= 'red', markerfacecolor= 'red')
    # plt.plot(0.425*25.4, OBx425[0], marker= 'o', markersize= 3, markeredgecolor= 'red', markerfacecolor= 'red')
   #  plt.plot(0.45*25.4, OBx45[0], marker= 'o', markersize= 3, markeredgecolor= 'red', markerfacecolor= 'red')                                                                             
    # plt.plot(0.475*25.4, OBx475[0], marker= 'o', markersize= 3, markeredgecolor= 'red', markerfacecolor= 'red')                      
    # plt.plot(0.5*25.4, OBx5[0], marker= 'o', markersize= 3, markeredgecolor= 'red', markerfacecolor= 'red') 







#fig = plt.figure()
 
# syntax for 3-D plotting
#ax = plt.axes(projection ='3d')
 
# syntax for plotting
#ax.plot_surface(x, y, z, cmap ='viridis', edgecolor ='green')
#ax.set_title('Surface plot geeks for geeks')
#plt.show()



#for j in range(0, B0.shape[0]):
    #B0_new[j,:]= B0[j, :]- listCenters[0]




    



#Xtest against  findR(B,Pboard):, getV(R):, getrAndH(v): --> H should agree with simulation input; if not, stop and think
    
#if that works, try to finish XcalculateGs and findt
#test getPositionAndOrientation(B,Pboard) against simulation input
    
#Xif you get stuck or if you finish, then try adding noise to B,
#Xe.g. B = B + np.random.random_sample(B.shape)*sigma #sigma is the noise level
#Xthen test to see effect
    
#ultimately what we want is error vs sigma
#pick a sigma, make many noisy fields, measure H, average of Htrue dot Hmeas vs. sigma (0 -->1, 0.1-->0.98?)
#if you had a position average of |Ob - OBtrue| vs. sigma (maybe sqrt(avg(|Ob-OBtrue|^2))) 
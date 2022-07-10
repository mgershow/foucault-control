#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Jul  7 15:55:13 2022

@author: gershow
"""
import serial
import serial.tools.list_ports
import time
import struct
from enum import Enum
import numpy as np
from scipy import linalg
import matplotlib.pyplot as plt


TARGET_VERSION = 10

class ReadingType(Enum):
    INVALID= -1
    SYNC = 0
    DETVAL = 1
    COILI = 2
    MAGVEC = 3
    ACCVEC = 4
    CROSSING = 5
    MAGVEC0 = 6
    MAGVEC1 = 7
    MAGVEC2 = 8
    MAGVEC3 = 9
    MAGVEC4 = 10
    MAGVEC5 = 11
    MAGVEC6 = 12
    MAGVEC7 = 13
    
class Reading:
    
    readingsize = 20
    
    def __init__(self, b):
        self.readingType = -1
        self.readingTime = 0
        self.x = 0
        self.y = 0
        self.z = 0
        self.valid = False
        try:
            self.readingType = b[0]
            self.readingTime = int.from_bytes(b[1:7], byteorder = 'little', signed=False)
            [self.x,self.y,self.z] = struct.unpack('fff', b[7:19])
            xs = b[0]^b[0]
            for bb in b:
                xs = xs ^ bb
            self.valid = xs == 0  
        except:
            self.valid = False
            
    
    @staticmethod
    def fromSerial(ser):
        readings = []
        valid = True
        while (ser.in_waiting >= Reading.readingsize):
            r = Reading(ser.read(Reading.readingsize))
            valid = valid and r.valid
            readings.append(r)
        if valid:
            return (readings, len(readings))
        else:
            return (readings, -len(readings))

class MagReading:
    sensorPositions = np.array([[0,0,0],
                                [0, 75, 0],
                                [64.951,-37.5,0],
                                [-64.951,-37.5,0],
                                [25,0,0],
                                [-50,0,0],
                                [0,-25,0],
                                [0,50,0]])
    
    offsetFields = 0*sensorPositions
    
    def __init__(self, readings):
        self.readingTime = 0
        self.mag = np.zeros([8,3])
        self.valid = [False]*8
        for r in readings:
            if (r.valid and r.readingType >= ReadingType.MAGVEC0.value and r.readingType <= ReadingType.MAGVEC7.value):
                self.readingTime = np.maximum(self.readingTime, r.readingTime)
                ind = r.readingType-ReadingType.MAGVEC0.value
                self.mag[ind,:] = [r.x,r.y,r.z];
                self.valid[ind] = True
    
    
    def setAsOffset(self):
        MagReading.offsetFields[self.valid,:] = self.mag[self.valid,:]
    
    def getMag(self):
        return self.mag[self.valid,:] - self.offsetFields[self.valid,:]

    def getPositions(self):
        return self.sensorPositions[self.valid,:]
    
    def plotXY(self):
        u = self.getMag()[:,0]
        v = self.getMag()[:,1]
        w = self.getMag()[:,2]
        
        n = np.sqrt(u**2 + v**2 + w**2);
        
    
        x = self.getPositions()[:,0]
        y = self.getPositions()[:,1]
       
        plt.quiver(x,y,u/n,v/n)
        plt.show()

    
    def estimateLocation(self):
        #hsieh et al 10.1109/AIM.2019.8868443
        B = self.getMag()
        P = self.getPositions()
        cp = np.cross(B, P)
        #P = self.sensorPositions
        R = np.hstack((B,cp)) #eq 7
        
        M = np.matmul(np.transpose(R), R)
        (w,v) = linalg.eig(M)
        ind = np.argmin(np.abs(w))
        v = v[:,ind]
        print(linalg.norm(v[3:]))
        v = v / linalg.norm(v[3:]) #eq 10
        H = v[3:] #eq 11
        r = v[0:3] #eq 12
        
        c2 = np.zeros(P.shape)
        c1 = np.copy(c2)
        c0 = np.copy(c2)
        for j in range(P.shape[0]):
           # P = self.sensorPositions[j,:]
            HdP = np.dot(H,P[j,:])
            rcH= np.cross(r,H)
            #eq 19
            g2 = 2*H
            g1 = -3*P[j,:] + 3*rcH-HdP*H
            g0 = 3*HdP*(P[j,:]-rcH) -(np.dot(P[j,:],P[j,:])+np.dot(r,r)-2*rcH)*H
            #eq 25
            c2[j,:] = np.cross(g2,B[j,:])
            c1[j,:] = np.cross(g1,B[j,:])
            c0[j,:] = np.cross(g0,B[j,:])
        
        #c2t^2 + c1t+c0 = all 0s  if perfect
        #find t that minimizes |c2t^2 + c1t + c0|^2
        a = c2.flatten()
        b = c1.flatten()
        c = c0.flatten()
        
        coeff_decrease = [np.dot(a,a), 2*np.dot(a,b), 2*np.dot(a,c)+np.dot(b,b), 2*np.dot(b,c), np.dot(c,c)];
        #H has magnitude 1; t*H is a position in units of sensorPositions (mm)
        #so t is reasonably between -1000 and 1000 (1 meter)
        sqerr = np.polynomial.Polynomial(coeff_decrease[::-1], domain=[-1e3, 1e3])
        dv0 = sqerr.deriv().roots() #solve for gradient is 0
        dv0 = dv0[np.isreal(dv0)]
        t = dv0[np.argmin(sqerr(dv0))]
        x = rcH + t*H #eq 13
        return(x,H)
                                

def sendCommand(ser, cmd, timestamp = 0, data = []):
    dd = [0,0,0,0]
    for j in range(0,len(data)):
        dd[j] = data[j]
    cmd = '{} {} {} {} {} {}\n'.format(cmd, timestamp, dd[0], dd[1], dd[2], dd[3])
    ser.write(bytes(cmd,'utf-8'))
    

def synchronize(ser):
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    ser.reset_input_buffer()
    sendCommand(ser, 'X')
    nzeros = 0
    while (nzeros < 20):
        if ser.read(1) == 0:
            nzeros = nzeros + 1
        else:
            nzeros = 0
    bb = ser.read(1)
    while (bb == 0):
        bb = ser.read(1)
    b = ser.read(19)
    b.insert(0,bb)
    return Reading(b)
    

    

    

# def readone():
#     print("available")
#     print(arduino.in_waiting)
#     b = arduino.read(20)
#     print(len(b))
#     if (len(b) < 20):
#         return dict (type=0,time=0,x=0,y=0,z=0,valid=False)
#     readingType = b[0]
#     readingTime = int.from_bytes(b[1:7], byteorder = 'little', signed=False)
#     [x,y,z] = struct.unpack('fff', b[7:19])
#     xs = b[0]^b[0]
#     for bb in b:
#         xs = xs ^ bb
#     return dict (type=readingType,time=readingTime,x=x,y=y,z=z,valid= xs == 0)


def findPort():
    port_list = serial.tools.list_ports.comports();
    for p in port_list:
        try:
            arduino = serial.Serial(port=p.device, timeout = .1)
            arduino.reset_input_buffer()         
            sendCommand(arduino, 'T',0,[0 ,0, 0, 0])
            time.sleep(0.1)
            arduino.reset_input_buffer()
            time.sleep(0.1)
            arduino.reset_input_buffer()                
            sendCommand(arduino, 'R')
            time.sleep(0.1)
            lineread = arduino.readline().decode();
            print(lineread)
            version = int(lineread.strip())
            print ("version = {}".format(version))
            if (version == TARGET_VERSION):
                return arduino
            if (version > TARGET_VERSION):
                print ("hardware version {} is greater than software version {}. Update this software".format(version, TARGET_VERSION))
                return arduino
        except Exception as e:
            print(e)
        try:
            arduino.close()
        except:
            pass    
    print ("no valid devices found")
    return 0

def clearBuffer(arduino):
    sendCommand(arduino, 'T',0,[0 ,0, 0, 0])
    time.sleep(0.05) 
    arduino.reset_input_buffer()
    time.sleep(0.05) 
    arduino.reset_input_buffer()

def startup():
    
    print("opening serial port")
    arduino = findPort()

    sendCommand(arduino, 'T',0,[2 ,0, 0, 0])#enable magnetometer
    time.sleep(0.1)
    
    while(arduino.in_waiting < 160):
        time.sleep(0.01)
    
    (rr,nread) = Reading.fromSerial(arduino)
    print(nread)
    
    MagReading(rr).setAsOffset()
    global mrtest
    mrtest = MagReading(rr)
    
    sendCommand(arduino, 'T',0,[0 ,0, 0, 0])
    time.sleep(0.01)
    
    arduino.reset_input_buffer()
    print("magnetometer zeroed - ready to continue")
    return arduino

def grabReadings(arduino):
    sendCommand(arduino, 'T',0,[2 ,0, 0, 0])#enable magnetometer
    time.sleep(0.1)
    
    while(arduino.in_waiting < 160):
        time.sleep(0.01)
    
    
    (rr,nread) = Reading.fromSerial(arduino)
    
    sendCommand(arduino, 'T',0,[0 ,0, 0, 0])
    time.sleep(0.01)
    
    arduino.reset_input_buffer()
    
    return (rr,nread)
    

def repeatMeasurements(arduino):
    clearBuffer(arduino)
    
    sendCommand(arduino, 'T',0,[0 ,0, 0, 0])
    time.sleep(0.01)
    
    arduino.reset_input_buffer()
    sendCommand(arduino, 'T',0,[2 ,0, 0, 0])
    while True:
        while(arduino.in_waiting < 160):
            time.sleep(0.01)
        (rr,nread) = Reading.fromSerial(arduino) 
        
        mr = MagReading(rr[-12:])
        plt.clf()
        mr.plotXY()
        #(x,H) = mr.estimateLocation()
        #print('location = {}, h = {}'.format(x, H))
        plt.pause(0.1)

# time.sleep(1)

# sendCommand(arduino, 'T',0,[2 ,0, 0, 0])#enable magnetometer
# time.sleep(0.5)
# (rrr,nnread) = Reading.fromSerial(arduino)

# for j in range(0,10):
#     time.sleep(0.5)
#     (rr,nread) = Reading.fromSerial(arduino)
#     plt.figure(1)
#     MagReading(rr).plotXY()
#     plt.draw()
    
#  #   print(nread)
#     # for r in rr:
#     #     if (r.readingType == ReadingType.MAGVEC0.value):
#     #         print(r.__dict__)
#     #         break

    
# sendCommand(arduino, 'T',0,[0 ,0, 0, 0])
# arduino.close()

# mr = MagReading(rr)
# (x,H) = mr.estimateLocation()
# print('location = {}, h = {}'.format(x, H))

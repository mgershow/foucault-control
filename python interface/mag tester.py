#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Jul  7 15:55:13 2022

@author: gershow
"""
import serial
import io

arduino = serial.Serial(port='/dev/cu.usbmodem1441', timeout = .1)


arduino.write("T 0 0 0 0 0\n")
arduino.flush()
arduino.reset_input_buffer()

arduino.write("R 0 0 0 0 0\n")
print(arduino.readLine())

arduino.close()
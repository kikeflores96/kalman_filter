#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Feb 27 18:02:11 2021

@author: kike
"""


from datetime import datetime
import serial
import struct
import numpy as np
import copy
import time


class serialPlot:
    # def __init__(self, serialPort='/dev/ttyUSB0', serialBaud=38400, dataNumBytes=2, numVariables=1):
    def __init__(self, serialPort='/dev/ttyACM0', serialBaud=115200, dataNumBytes=2, numVariables=9):
        self.port = serialPort
        self.baud = serialBaud
        self.dataNumBytes = dataNumBytes
        self.numVariables = numVariables
        self.rawData = bytearray(numVariables * dataNumBytes)
        self.dataType = None
        if dataNumBytes == 2:
            self.dataType = 'h'     # 2 byte integer
        elif dataNumBytes == 4:
            self.dataType = 'f'     # 4 byte float
        self.dataBlock = []
        self.data = []
        for i in range(numVariables):
            self.data.append([])

        print('Trying to connect to: ' + str(serialPort) + ' at ' + str(serialBaud) + ' BAUD.')
        try:
            self.serialConnection = serial.Serial(serialPort, serialBaud, timeout=4)
            print('Connected to ' + str(serialPort) + ' at ' + str(serialBaud) + ' BAUD.')
            print(self.serialConnection)
        except:
            print("Failed to connect with " + str(serialPort) + ' at ' + str(serialBaud) + ' BAUD.')

    def getSerialData(self, t):
        self.serialConnection.reset_input_buffer()
        startTime = datetime.now()

        while (datetime.now() - startTime).total_seconds() < t:
            self.serialConnection.readinto(self.rawData)
            self.dataBlock.append(self.rawData[:])

        print("Captured data for %d seconds" % t)
        self.close()
        print("Processing data ...")

        for i in range(len(self.dataBlock)):
            for j in range(self.numVariables):
                byteData = self.dataBlock[i][(j * self.dataNumBytes):((j + 1) * self.dataNumBytes)]
                value, = struct.unpack(self.dataType, byteData)
                self.data[j].append(copy.copy(value))
        # print(self.data)
        print("Exporting data ...")
        csvData = np.flip(np.array(self.data), 1).transpose()
        # np.savetxt('magnetometer_data.csv', csvData, delimiter=',', fmt='%f')
        print("Done")
        return csvData

    def close(self):
        self.serialConnection.close()
        print('Disconnected...')


# def main():
#     # portName = 'COM6'
#     portName = '/dev/ttyACM0'
#     baudRate = 115200
#     dataNumBytes = 4        # number of bytes of 1 data point
#     numVariables = 9        # number of plots in 1 graph
#     s = serialPlot(portName, baudRate, dataNumBytes, numVariables)   # initializes all required variables
#     time.sleep(2)
#     s.getSerialData(10)


# if __name__ == '__main__':
#     main()


# portName = 'COM6'
# portName = '/dev/ttyACM1'
portName = '/dev/ttyACM0'
baudRate = 115200
dataNumBytes = 2        # number of bytes of 1 data point
numVariables = 9        # number of plots in 1 graph
s = serialPlot(portName, baudRate, dataNumBytes, numVariables)   # initializes all required variables
time.sleep(2)
Data = s.getSerialData(30)
np.savetxt('24_02_2022_2.csv', Data, delimiter=',', fmt='%f')

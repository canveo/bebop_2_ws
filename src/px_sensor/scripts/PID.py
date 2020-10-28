#!/usr/bin/env python
#
# title           :PID.py
# description     :PID controller for X, Y, Z, roll, pitch, yaw
# author          :Carlos Hansen
# date            :28-11-2019
# pythonVersion   :2.7.15
# ==============================================================================
import numpy as np  # for matrix manipulations
import time         # for the deltas of time


class PID:
    """PID controller for a 3 element vector
    
    Attributes
    ----------
    @ivar Kp: Set of constants for the Proportional part of the controller
    @type Kp: numpy.ndarray(1_6)
    @ivar Kp: Set of constants for the Integral part of the controller
    @type Kp: numpy.ndarray(1_6)
    @ivar Kp: Set of constants for the Derivative part of the controller
    @type Kp: numpy.ndarray(1_6)
    @ivar setPoint: Desired point to reach for the PID controller
    @type setPoint: numpy.ndarray(1_6)
    Methods
    -------
    TODO: CREATE the description of the methods
    """

    def __init__(self, P=np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]), I=np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]), D=np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]), samplingTime=0):
        """Initialize the controller
        @param P: Proportional constant XYZrpy (default: {[1.0,1.0,1.0, 1.0, 1.0, 1.0]})
        @type P: numpy.ndarray(1_6)
        @param I: Integral constant XYZrpy (default: {[0.0, 0.0, 0.0 ,0.0, 0.0, 0.0]})
        @type I: numpy.ndarray(1_6)
        @param D: Derivative constant XYZrpy (default: {0.0, 0.0, 0.0, 0.0, 0.0, 0.0})
        @type D: numpy.ndarray(1_6)
        """
        self.Kp = P
        self.Ki = I
        self.Kd = D

        self.setSamplingTime(samplingTime)

        self.setSetPoint(np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]))
        self.clear()    # Set all the values to 0
        self.setLastTime(0) # Initialize time

    def controll(self, currentVector, currentTime):
        """Get the controlled output depending on the currentVector
        @param currentVector: Current vector to move to the setPoint
        @type currentVector: numpy.ndarray(1_6)
        @param currentTime: Current time
        @type currentTime: Time_obj 
        @returns: Controlled Output
        @rtype: numpy.ndarray(1_6)
        """

        deltaTime = currentTime - self.getLastTime()

        if (deltaTime > 2* self.getSamplingTime()):
            deltaTime = 2* self.getSamplingTime()

        error = self.setPoint - currentVector
        deltaError = error - self.getLastError()

        if (deltaTime >= self.getSamplingTime()):
            # Proportional part
            self.pComponent = np.multiply(self.Kp, error)

            # Integral part
            self.integral += error * deltaTime 
            self.iComponent = np.multiply(self.Ki, self.integral)


            # Derivative part
            self.dComponent = 0
            if (deltaTime > 0):
                self.dComponent = np.multiply(self.Kd, (deltaError/deltaTime))          
    

            PID = self.pComponent + self.iComponent + self.dComponent 

            # Update times and errors
            self.setLastTime(currentTime)
            self.setLastError(error)

            # print ("Command: {0}".format(PID))

            return PID

    # ----------- Setters and Getters

    def clear(self):
        """Clears PID computations and coefficients"""
        self.setIntegral(np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]))

        self.setLastError(np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]))

    def setSetPoint(self, setPoint):
        """ Initilize the setPoint of PID
        @param currentVector: Current vector to move to the setPoint
        @type currentVector: numpy.ndarray(1_6)
        """
        self.setPoint = setPoint
        
        self.clear()

    def getSetPoint(self):
        """ Get the current value of the Setpoint
        @returns: Current value of the Setpoint
        @rtype: numpy.ndarray(1_6)
		"""
        return self.setPoint

    def setIntegral(self, integral):
        """ Set the Integer to the desired value
        @param integral: New value for the Integer
        @type integral: numpy.ndarray(1_6)
        """
        self.integral = integral

    def getIntegral(self):
        """ Get the current value of the Integer
        @returns: Current value for the Integer
        @rtype: numpy.ndarray(1_6)
		"""
        return self.integral

    def setKp(self, P):
        """ Set the Proportional Constants to the desired value
        @param P: New value for the Proportional constants
        @type P: numpy.ndarray(1_6)
        """
        self.Kp = P

    def getKp(self):
        """ Get the Proportional Constants
        @returns: Proportional constants
        @rtype: numpy.ndarray(1_6)
        """
        return self.Kp

    def setKi(self, I):
        """ Set the Integral Constants to the desired value
        @param I: New value for the Integral constants
        @type I: numpy.ndarray(1_6)
        """
        self.Ki = I

    def getKi(self):
        """ Get the Integral Constants
        @returns: Integral constants
        @rtype: numpy.ndarray(1_6)
        """
        return self.Ki

    def setKd(self, D):
        """ Set the Derivative Constants to the desired value
        @param D: New value for the Derivative constants
        @type D: numpy.ndarray(1_6)
        """
        self.Kd = D

    def getKd(self):
        """ Get the Derivative Constants
        @returns: Derivative constants
        @rtype: numpy.ndarray(1_6)
        """
        return self.Kd

    def setLastError(self, err):
        """ Set the new Error value as the last
        @param err: New value for the Error deviation
        @type err: numpy.ndarray(1_6)
        """
        self.error = err

    def getLastError(self):
        """ Get the Last error from the controller
        @returns: error
        @rtype: numpy.ndarray(1_6)
        """
        return self.error

    def setLastTime(self, time):
        """ Set the new Time value as the last
        @param time: New value for the Time deviation
        @type time: Time_obj 
        """
        self.lastTime = time

    def getLastTime(self):
        """ Get the Last time from the controller
        @returns: time
        @rtype: Time_obj 
        """
        return self.lastTime

    def setSamplingTime(self, time):
        """ Set the sampling time for the controller
        @param time: New value for thesampling time
        @type time: Time_obj 
        """
        self.samplingTime = time

    def getSamplingTime(self):
        """ Get the sampling time from the controller
        @returns: time
        @rtype: Time_obj 
        """
        return self.samplingTime
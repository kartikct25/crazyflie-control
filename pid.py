#import numpy as np
import time
from collections import deque
import itertools

class PID:
    def __init__(self, kP = 1.0, kI = 0.1, kD = -0.3, minSat = -1e8, maxSat = 1e8):
        self.dFilterState = 0.0
        self.kP = kP
        self.kI = kI
        self.kD = kD
        if minSat<=maxSat:
            self.minSat = minSat
            self.maxSat = maxSat
        else:
            self.minSat = maxSat
            self.maxSat = minSat

        self.output = 0.0

        self.outputP = 0.0
        self.outputI = 0.0
        self.outputD = 0.0

        self._histSize = None
        self.outputHist = deque()
        self.errorHist  = deque()
        self.timeHist   = deque()
        self.setHistorySize(2)

        self.integralState = 0.0

        self.lastUpdate = 0.0

        self.isRunning = False

        self.timeHist[0] = time.time()

    def setHistorySize(self, size):
        if (size < 0):
            return
        size_old = len(self.outputHist)
        if size_old < size:
            tmp = (size - size_old) * [0]
            self.outputHist.extend(tmp)
            self.errorHist.extend(tmp)
            self.timeHist.extend(tmp)
        elif size_old > size:
            self.outputHist = deque( itertools.slice(self.outputHist, 0, size) )
            self.errorHist  = deque( itertools.slice(self.outputHist, 0, size) )
            self.timetHist  = deque( itertools.slice(self.outputHist, 0, size) )
        self._histSize = size

    def getHistorySize(self):
        return self._histSize

    def updateControl(self, plantOutput = 0.0, reference = 0.0):
        # Compute Error
        error =  reference - plantOutput

        # Compute how long ago the controller was updated
        if self.isRunning == False:
            lastDeltaT = 0.0
            self.isRunning = True
        else:
            lastDeltaT = time.time() - self.lastUpdate
        self.lastUpdate = time.time()

        # Shift history and insert data (output will be added later)
        self.errorHist.rotate(1)
        self.outputHist.rotate(1)
        self.timeHist.rotate(1)
        self.timeHist[0] = self.lastUpdate
        self.errorHist[0] = error

        # Compute Control Terms
        self.outputP = self.kP * error
        self.outputD = self.kD * self.derivative()
        self.integralState += self.integrate()
        self.outputI = self.kI * self.integralState

        # Compute Output
        self.output = self.outputP + self.outputI +  self.outputD

        # Add output to history
        self.outputHist[0] = self.output

    def saturatedControl(self):

        if (self.output < self.maxSat and self.output > self.minSat): # If the output is within bounds, no saturation needed
            return self.output
            
        else: # Otherwise, saturate
            if (self.output < self.minSat): # If it is below the minimum saturation, use the minimum saturation
                return self.minSat    
            else: # If it is above the maximum saturation, use the maximum saturation
                return self.maxSat

    def integrate(self):
        if (self.output < self.maxSat and self.output > self.minSat) and self.timeHist[1] != 0.0: # Only integrate if unsaturated.
            return ((self.errorHist[0] + self.errorHist[1]) / 2 * (self.timeHist[0] - self.timeHist[1]))
        else:
            return 0.0

    def derivative(self):
        if (self.timeHist[0] != self.timeHist[1]): # Case taken into consideration since it can produce division by zero
            # return ((self.errorHist[0] - self.errorHist[1]) / (self.timeHist[0] - self.timeHist[1]))
            self.dFilterState = 0.4724 * self.dFilterState + 2.0 * self.errorHist[0] # Discrete Filter implementation for h = 0.05 and N = 15
            return (-3.9573 * self.dFilterState + 15.0 * self.errorHist[0])
        else:
            return 0.0  # Should only happen in the first iteration of the controller. Unlikely to cause trouble.

    def setIntegralState(self, value):
        self.integralState = value
        
"""
  __  __       _       
 |  \/  |     (_)      
 | \  / | __ _ _ _ __  
 | |\/| |/ _` | | '_ \ 
 | |  | | (_| | | | | |
 |_|  |_|\__,_|_|_| |_|

"""
  
if __name__ == '__main__':
    controller = PID(1.0, 0.1, 0.3, 0, 60000)
    reference = 1.0
    y = 0.1
    for i in range(0,10):
        y += controller.output/5
        controller.updateControl(y, reference)
    
        print('The control for the reference {:1.2f} when the output is {:1.2f} is {:1.2f} (P: {:1.2f}, I: {:1.2f}, D: {:1.2f})'.format(reference,y, controller.output, controller.outputP, controller.outputI, controller.outputD))

        time.sleep(0.5)
    
"""
   _____          _        ______           _ 
  / ____|        | |      |  ____|         | |
 | |     ___   __| | ___  | |__   _ __   __| |
 | |    / _ \ / _` |/ _ \ |  __| | '_ \ / _` |
 | |___| (_) | (_| |  __/ | |____| | | | (_| |
  \_____\___/ \__,_|\___| |______|_| |_|\__,_|
                                              
"""       
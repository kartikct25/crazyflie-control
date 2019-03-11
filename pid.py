#import numpy as np
import time

class PID:
    histSize = 2

    # minSat = 0.0
    # maxSat = 1.0

    # kP = 1.0
    # kI = 0.1
    # kD = -0.3

    # output = 0.0
    # outputHist = [0.0 for x in range(histSize)]

    # outputP = 0.0
    # outputI = 0.0
    # outputD = 0.0

    # errorHist = [0.0 for x in range(histSize)]
    # timeHist = [0.0 for x in range(histSize)]

    # integralState = 0.0

    # lastUpdate = 0.0

    # isRunning = False

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

        self.outputHist = [0.0 for x in range(self.histSize)]

        self.outputP = 0.0
        self.outputI = 0.0
        self.outputD = 0.0

        self.errorHist = [0.0 for x in range(self.histSize)]
        self.timeHist = [0.0 for x in range(self.histSize)]

        self.integralState = 0.0

        self.lastUpdate = 0.0

        self.isRunning = False

        self.timeHist[0] = time.time()

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

        # Append it to history
        for i in range(self.histSize-1, 0, -1):
            self.errorHist[i] = self.errorHist[i - 1]
            self.timeHist[i] = self.timeHist[i - 1]
        self.errorHist[0] = error
        self.timeHist[0] = self.lastUpdate

        # Compute Control Terms
        self.outputP = self.kP * error
        self.outputD = self.kD * self.derivative()
        self.integralState += self.integrate()
        self.outputI = self.kI * self.integralState

        # Compute Output
        self.output = self.outputP + self.outputI +  self.outputD

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
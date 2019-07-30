# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2018 Bitcraze AB
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
#  MA  02110-1301, USA.
"""
Script to run landing experiment.
"""

#   _____                            _                 _           
#  |  __ \                          | |               (_)          
#  | |  | | ___ _ __   ___ _ __   __| | ___ _ __   ___ _  ___  ___ 
#  | |  | |/ _ \ '_ \ / _ \ '_ \ / _` |/ _ \ '_ \ / __| |/ _ \/ __|
#  | |__| |  __/ |_) |  __/ | | | (_| |  __/ | | | (__| |  __/\__ \
#  |_____/ \___| .__/ \___|_| |_|\__,_|\___|_| |_|\___|_|\___||___/
#              | |                                                 
#              |_|                                                 

import time

import math

import csv

import numpy as np

from pid import PID
from crazyflight import CrazyFlight

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.mem import MemoryElement
from cflib.crazyflie.mem import Poly4D
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger

#  __      __        _       _     _           
#  \ \    / /       (_)     | |   | |          
#   \ \  / /_ _ _ __ _  __ _| |__ | | ___  ___ 
#    \ \/ / _` | '__| |/ _` | '_ \| |/ _ \/ __|
#     \  / (_| | |  | | (_| | |_) | |  __/\__ \
#      \/ \__,_|_|  |_|\__,_|_.__/|_|\___||___/
                                            
# URI to the Crazyflie to connect to
uri1 = 'radio://0/80/2M'
uri1 = 'radio://0/85/2M'
uri3 = 'radio://0/100/2M'

constPi = math.pi

#   _____        __ _       _ _   _                 
#  |  __ \      / _(_)     (_) | (_)                
#  | |  | | ___| |_ _ _ __  _| |_ _  ___  _ __  ___ 
#  | |  | |/ _ \  _| | '_ \| | __| |/ _ \| '_ \/ __|
#  | |__| |  __/ | | | | | | | |_| | (_) | | | \__ \
#  |_____/ \___|_| |_|_| |_|_|\__|_|\___/|_| |_|___/                         

def reset_estimator(cf):
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.5)
    cf.param.set_value('kalman.resetEstimation', '0')

    cf.param.set_value('kalman.initialX', '{:1.16f}'.format(0.00))
    time.sleep(0.2)
    cf.param.set_value('kalman.initialY', '{:1.16f}'.format(0.00))
    time.sleep(0.2)

    wait_for_position_estimator(cf)

def wait_for_position_estimator(scf):
    print('Waiting for estimator to find position...')

    log_config = LogConfig(name='Kalman Variance', period_in_ms=500)
    log_config.add_variable('kalman.varPX', 'float')
    log_config.add_variable('kalman.varPY', 'float')
    log_config.add_variable('kalman.varPZ', 'float')

    var_y_history = [1000] * 10
    var_x_history = [1000] * 10
    var_z_history = [1000] * 10

    threshold = 1.0e-3

    with SyncLogger(scf, log_config) as logger:
        for log_entry in logger:
            data = log_entry[1]

            var_x_history.append(data['kalman.varPX'])
            var_x_history.pop(0)
            var_y_history.append(data['kalman.varPY'])
            var_y_history.pop(0)
            var_z_history.append(data['kalman.varPZ'])
            var_z_history.pop(0)

            min_x = min(var_x_history)
            max_x = max(var_x_history)
            min_y = min(var_y_history)
            max_y = max(var_y_history)
            min_z = min(var_z_history)
            max_z = max(var_z_history)

            print("{} {} {}".
                  format(max_x - min_x, max_y - min_y, max_z - min_z))

            if (max_x - min_x) < threshold and (
                    max_y - min_y) < threshold and (
                    max_z - min_z) < threshold:
                break
    
    logger.disconnect


def smoothApproach(time):

        a = -1.173333333
        b = 3.348571429
        c = -3.175238095
        d = 1.000571429

        return a * time*time*time + b * time*time + c * time + d

"""
  __  __       _       
 |  \/  |     (_)      
 | \  / | __ _ _ _ __  
 | |\/| |/ _` | | '_ \ 
 | |  | | (_| | | | | |
 |_|  |_|\__,_|_|_| |_|

"""
  
if __name__ == '__main__':

    cflib.crtp.init_drivers(enable_debug_driver=False)

    with SyncCrazyflie(uri1, cf=Crazyflie(rw_cache='./cache')) as scf:
        cf = scf.cf

        # Prepare for Flight
        reset_estimator(cf)

        flight = CrazyFlight(scf)

        # Unlock thrust control
        for _ in range(10):
            cf.commander.send_setpoint(0.0, 0.0, 0.0, 0)

        # Go to experiment location
        print('Taking off')

        # Run experiment
        t = 0.0                     # Initial timestamp
        h = 0.05                    # Sampling/Control Period
        experimentTimeout = 15.0    # Time at which the experiment ends and landing starts

        maxAngle = 5.0      # Maximum roll and pitch angle
        angleFactor = 20.0  # Normalization for roll and pitch PIDs
        maxPos = 0.25       # Maximum horizontal deviation during experiment

        #                      P    I    D
        rollController  = PID(1.2, 0.01, 0.5, -maxAngle, maxAngle)  # PID instantiation for roll
        pitchController = PID(1.2, 0.01, 0.5, -maxAngle, maxAngle)  # PID instantiation for pitch
        # yawController   = PID(1.0, 0.0, 0.3, -1e8     , 1e8)

        thrustFactor = 33.03e3  # Thrust input at which the Crazyflie hovers. Makes the altitude PID zero output a neutral control.

        altiController = PID(0.5e0, 3.5e-1,6.5e-1, -0.5, 65e3/2/thrustFactor)   # PID instantiation for altitude

        amplitude = 0.0
        period = 4
        omega = 1/period * 2*math.pi

        zRef = 0.50

        lastAltiReduc = 0.0
        timeAtAlti = 5.0
        altiReducStep = 0.3

        alreadyChanged = True

        fftHistSize = 100
        downHist = []
        fftFreq = np.fft.fftfreq((np.arange(fftHistSize)*h).shape[-1])

        with open('CSV_landing.txt', mode='w', newline='') as csv_file:

            csv_log = csv.writer(csv_file, delimiter=' ', quotechar='"', quoting=csv.QUOTE_NONE)

            flight.resetFlightStartTime()

            # cf.param.set_value('ground.level', '{:1.16f}'.format(0.0))

            while (t < experimentTimeout):
                
                # Control cycle start
                controlCycleStart = time.time()

                if flight.cfPos[2] > 1.0 or flight.cfPos[2] < -0.5:
                    print('Abnormal altitude ({:1.2f})'.format(flight.cfPos[2]))
                    break

                if (flight.cfPos[0] > maxPos) or (flight.cfPos[0] < -maxPos) or (flight.cfPos[1] > maxPos) or (flight.cfPos[1] < -maxPos):
                    print('Abnormal position')
                    break

                t = time.time() - flight.flightStart

                rollController.updateControl(-flight.cfPos[1], 0.0)
                pitchController.updateControl(flight.cfPos[0], 0.0)

                # zRef = max(min( 0.20*smoothApproach((t-10.0)/10.0), 0.20), 0.0) + 0.1

                zOsc = zRef + amplitude * math.sin(omega * t)
                zOscDot = omega * amplitude * math.cos(omega * t)

                altiController.updateControl(flight.cfPos[2], zOsc)
                
                ffControl = 0.0 * zOsc + 0.0 * zOscDot

                thrustControl = int( \
                    thrustFactor * (1.0 + altiController.saturatedControl()) + ffControl    \
                        )

                cf.commander.send_setpoint(angleFactor*rollController.saturatedControl(), \
                    angleFactor*pitchController.saturatedControl(), \
                        0.0, thrustControl)

                csv_log.writerow([t, flight.cfPos[0], flight.cfPos[1], flight.cfPos[2], flight.cfEuler[0], flight.cfEuler[1], flight.cfEuler[2], \
                    pitchController.saturatedControl(), rollController.saturatedControl(), 0.0, zOsc, altiController.saturatedControl(), flight.cfDownRange])

                # print('zRange {:1.3f}'.format(flight.cfDownRange*1e-3))

                if ((t - lastAltiReduc) >= timeAtAlti) and (alreadyChanged == False):
                    zRef -= altiReducStep
                    lastAltiReduc = math.floor(t)
                    amplitude = 0.0
                    alreadyChanged = True

                # FFT
                if len(downHist) < fftHistSize:
                    downHist.extend([flight.cfDownRange])
                else:
                    downHist = np.roll(downHist, -1)
                    downHist[-1] = flight.cfDownRange

                    fftSpectrum = np.fft.fft(downHist)

                # Control cycle end

                controlCycleEnd = time.time()

                holdPeriod = h - (controlCycleEnd - controlCycleStart)

                if holdPeriod > 0:
                    time.sleep(holdPeriod)
                else:
                    print('WARNING! The control cycle exceeded the sampling period by {}s'.format(-holdPeriod))

        # Get back
        print('Landing')
        t = time.time() - flight.flightStart

        while (flight.cfPos[2] > 0.2) and ((t - flight.flightStart) < experimentTimeout + 2.0):

            rollController.updateControl(-flight.cfPos[1], 0.0)
            pitchController.updateControl(flight.cfPos[0], 0.0)
            altiController.updateControl(flight.cfPos[2], 0.15)

            cf.commander.send_setpoint(angleFactor*rollController.saturatedControl(), \
                angleFactor*pitchController.saturatedControl(), \
                    0.0, \
                        int(thrustFactor*(1.0+altiController.saturatedControl())))

            time.sleep(0.05) # Update delay

            t = time.time() - flight.flightStart

        cf.commander.send_setpoint(0.0, 0.0, 0.0, 0)

        flight.removeLogBlocks(scf)

        time.sleep(0.1)
        flight.__del__
        time.sleep(0.1)

        print(downHist)

        if len(downHist) == fftHistSize:
            import matplotlib.pyplot as plt
            plt.plot(fftFreq, fftSpectrum.real, fftFreq, fftSpectrum.imag)
            plt.show()

"""
   _____          _        ______           _ 
  / ____|        | |      |  ____|         | |
 | |     ___   __| | ___  | |__   _ __   __| |
 | |    / _ \ / _` |/ _ \ |  __| | '_ \ / _` |
 | |___| (_) | (_| |  __/ | |____| | | | (_| |
  \_____\___/ \__,_|\___| |______|_| |_|\__,_|
                                              
"""       
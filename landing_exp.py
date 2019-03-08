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
uri2 = 'radio://0/85/2M'
uri3 = 'radio://0/100/2M'

constPi = math.pi

#  _____          _        _           _        
#  |_   _| _ __ _ (_)___ __| |_ ___ _ _(_)___ ___
#    | || '_/ _` || / -_) _|  _/ _ \ '_| / -_|_-<
#    |_||_| \__,_|/ \___\__|\__\___/_| |_\___/__/
#               |__/                             

# The trajectory to fly
# See https://github.com/whoenig/uav_trajectories for a tool to generate
# trajectories

# Duration,x^0,x^1,x^2,x^3,x^4,x^5,x^6,x^7,y^0,y^1,y^2,y^3,y^4,y^5,y^6,y^7,z^0,z^1,z^2,z^3,z^4,z^5,z^6,z^7,yaw^0,yaw^1,yaw^2,yaw^3,yaw^4,yaw^5,yaw^6,yaw^7
figure8 = [
    [1.050000, 0.000000, -0.000000, 0.000000, -0.000000, 0.830443, -0.276140, -0.384219, 0.180493, -0.000000, 0.000000, -0.000000, 0.000000, -1.356107, 0.688430, 0.587426, -0.329106, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa 33 elementos
    [0.710000, 0.396058, 0.918033, 0.128965, -0.773546, 0.339704, 0.034310, -0.026417, -0.030049, -0.445604, -0.684403, 0.888433, 1.493630, -1.361618, -0.139316, 0.158875, 0.095799, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
    [0.620000, 0.922409, 0.405715, -0.582968, -0.092188, -0.114670, 0.101046, 0.075834, -0.037926, -0.291165, 0.967514, 0.421451, -1.086348, 0.545211, 0.030109, -0.050046, -0.068177, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
    [0.700000, 0.923174, -0.431533, -0.682975, 0.177173, 0.319468, -0.043852, -0.111269, 0.023166, 0.289869, 0.724722, -0.512011, -0.209623, -0.218710, 0.108797, 0.128756, -0.055461, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
    [0.560000, 0.405364, -0.834716, 0.158939, 0.288175, -0.373738, -0.054995, 0.036090, 0.078627, 0.450742, -0.385534, -0.954089, 0.128288, 0.442620, 0.055630, -0.060142, -0.076163, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
    [0.560000, 0.001062, -0.646270, -0.012560, -0.324065, 0.125327, 0.119738, 0.034567, -0.063130, 0.001593, -1.031457, 0.015159, 0.820816, -0.152665, -0.130729, -0.045679, 0.080444, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
    [0.700000, -0.402804, -0.820508, -0.132914, 0.236278, 0.235164, -0.053551, -0.088687, 0.031253, -0.449354, -0.411507, 0.902946, 0.185335, -0.239125, -0.041696, 0.016857, 0.016709, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
    [0.620000, -0.921641, -0.464596, 0.661875, 0.286582, -0.228921, -0.051987, 0.004669, 0.038463, -0.292459, 0.777682, 0.565788, -0.432472, -0.060568, -0.082048, -0.009439, 0.041158, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
    [0.710000, -0.923935, 0.447832, 0.627381, -0.259808, -0.042325, -0.032258, 0.001420, 0.005294, 0.288570, 0.873350, -0.515586, -0.730207, -0.026023, 0.288755, 0.215678, -0.148061, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
    [1.053185, -0.398611, 0.850510, -0.144007, -0.485368, -0.079781, 0.176330, 0.234482, -0.153567, 0.447039, -0.532729, -0.855023, 0.878509, 0.775168, -0.391051, -0.713519, 0.391628, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
]

reta = [
    [1.8815,0,0,0,0,2.76531,-2.92775,1.08427,-0.140453,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
    [1.8815,2,1.12711e-12,-1.99691,-2.91512e-12,0.0551595,1.12888,-0.765572,0.140453,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
]

voltaLEAD = [
[0.903641,0,0,0,0,16.2131,-32.3605,24.0914,-6.46218,0,0,0,0,-3.5376,9.63853,-8.83593,2.76868,0,0,0,0,3.7226,-10.166,9.33071,-2.92096,0,0,0,0,0,0,0,0],
[1.07395,1.25,2.43196,-0.160153,-0.919801,0.535753,-3.18303,3.87985,-1.30837,0,0.301344,0.446916,0.134018,0.450873,-0.44826,-0.117606,0.131069,0,-0.305563,-0.410953,-0.0247011,-3.56048,9.52217,-7.89416,2.17102,0,0,0,0,0,0,0,0],
[1.05136,2.5,-0.405999,-0.677591,-0.130451,1.40599,-3.86201,3.61057,-1.09559,1,1.37697,0.0319538,0.0897663,-1.39976,0.902388,0.175442,-0.192415,-0.5,0.069693,0.431668,0.023002,6.06406,-14.4551,11.4455,-3.0815,0,0,0,0,0,0,0,0],
[1.05136,1.25,-1.85029,-0.0286544,0.160863,1.16218,-0.112328,-1.04045,0.471534,2,0.28094,-0.456617,0.0271276,-3.64296,4.71741,-2.14938,0.332311,0,0.01863,-0.6265,-0.00822939,-3.70118,9.20026,-7.34439,1.97562,0,0,0,0,0,0,0,0],
[0.84102,0,-0.222247,0.532664,-0.211391,0.946794,-3.87498,4.4664,-1.63924,1,-2.12906,-0.137002,1.10488,-1.69555,12.9032,-18.9745,7.94264,-0.5,-0.254957,0.411528,-0.0954076,3.20752,-9.906,10.1171,-3.48337,0,0,0,0,0,0,0,0],
]

#   _____        __ _       _ _   _                 
#  |  __ \      / _(_)     (_) | (_)                
#  | |  | | ___| |_ _ _ __  _| |_ _  ___  _ __  ___ 
#  | |  | |/ _ \  _| | '_ \| | __| |/ _ \| '_ \/ __|
#  | |__| |  __/ | | | | | | | |_| | (_) | | | \__ \
#  |_____/ \___|_| |_|_| |_|_|\__|_|\___/|_| |_|___/                         

class Uploader:
    def __init__(self):
        self._is_done = False

    def upload(self, trajectory_mem):
        print('Uploading data')
        trajectory_mem.write_data(self._upload_done)

        while not self._is_done:
            time.sleep(0.2)

    def _upload_done(self, mem, addr):
        print('Data uploaded')
        self._is_done = True

def reset_estimator(cf):
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.2)
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

def activate_high_level_commander(cf):
    cf.param.set_value('commander.enHighLevel', '1')
    cf.param.set_value('stabilizer.controller', '2')        

def upload_trajectory(cf, trajectory_id, trajectory):
    trajectory_mem = cf.mem.get_mems(MemoryElement.TYPE_TRAJ)[0]

    total_duration = 0
    for row in trajectory:
        duration = row[0]
        x = Poly4D.Poly(row[1:9])
        y = Poly4D.Poly(row[9:17])
        z = Poly4D.Poly(row[17:25])
        yaw = Poly4D.Poly(row[25:33])
        trajectory_mem.poly4Ds.append(Poly4D(duration, x, y, z, yaw))
        total_duration += duration

    Uploader().upload(trajectory_mem)
    cf.high_level_commander.define_trajectory(trajectory_id, 0,
                                              len(trajectory_mem.poly4Ds))
    return total_duration

def run_sequence(cf, trajectory_id, duration):
    commander = cf.high_level_commander

    print('Holding Position')
    time.sleep(1.0)

    #relative = True
    #commander.start_trajectory(trajectory_id, 1.0, relative)
    #time.sleep(duration)

    commander.go_to(0.0, 0.0, 1.0, constPi, 2.0)
    time.sleep(2.0)





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
        # cf.high_level_commander.takeoff(0.25, 0.5)
        # time.sleep(1.0)

        # Run experiment
        t = 0.0
        h = 0.05
        lastRun = -h
        experimentTimeout =4.0
        timeIsUp = False

        maxAngle = 5.0
        angleFactor = 10.0
        maxPos = 0.25
        #                      P    I    D
        rollController  = PID(1.0, 0.01, 0.2, -maxAngle, maxAngle)
        pitchController = PID(1.0, 0.01, 0.2, -maxAngle, maxAngle)
        # yawController   = PID(1.0, 0.0, 0.3, -1e8     , 1e8)

        thrustFactor = 33.03e3

        altiController = PID(1.1e0, 7.0e-1, 3.9e-1, -0.7, 65e3/2/thrustFactor)

        zRef = 0.3

        print('Battery status: {}'.format(flight.cfBattery))
        with open('CSV_landing.txt', mode='w', newline='') as csv_file:
            csv_log = csv.writer(csv_file, delimiter=' ', quotechar='"', quoting=csv.QUOTE_NONE)

            while (t < experimentTimeout):

                if flight.cfPos[2] > 0.6 or flight.cfPos[2] < -0.05:
                    print('Abnormal altitude ({:1.2f})'.format(flight.cfPos[2]))
                    break

                if (flight.cfPos[0] > maxPos) or (flight.cfPos[0] < -maxPos) or (flight.cfPos[1] > maxPos) or (flight.cfPos[1] < -maxPos):
                    print('Abnormal position')
                    break

                t = time.time() - flight.flightStart

                while ((t - lastRun) < h):
                    time.sleep(0.005)
                    t = time.time() - flight.flightStart
                
                lastRun = t

                rollController.updateControl(-flight.cfPos[1], 0.0)
                pitchController.updateControl(flight.cfPos[0], 0.0)

                altiController.updateControl(flight.cfPos[2], 0.3)

                cf.commander.send_setpoint(angleFactor*rollController.saturatedControl(), \
                    angleFactor*pitchController.saturatedControl(), \
                        0.0, \
                            int(thrustFactor*(1.0+altiController.saturatedControl())))

                csv_log.writerow([t, flight.cfPos[0], flight.cfPos[1], flight.cfPos[2], flight.cfEuler[0], flight.cfEuler[1], flight.cfEuler[2], \
                    pitchController.saturatedControl(), rollController.saturatedControl(), 0.0, zRef, altiController.saturatedControl()])

                # time.sleep(0.05) # Update delay

                if t > experimentTimeout:
                    timeIsUp = True

        # Get back
        print('Landing')
        t = time.time() - flight.flightStart

        # while (flight.cfPos[2] > 0.1) and ((t - flight.flightStart) < experimentTimeout + 2.0):

        #     rollController.updateControl(-flight.cfPos[1], 0.0)
        #     pitchController.updateControl(flight.cfPos[0], 0.0)
        #     altiController.updateControl(flight.cfPos[2], 0.1)

        #     cf.commander.send_setpoint(angleFactor*rollController.saturatedControl(), \
        #         angleFactor*pitchController.saturatedControl(), \
        #             0.0, \
        #                 int(thrustFactor*(1.0+altiController.saturatedControl())))

        #     time.sleep(0.05) # Update delay

        #     t = time.time() - flight.flightStart

        cf.commander.send_setpoint(0.0, 0.0, 0.0, 0)

        time.sleep(0.1)
        flight.__del__
        time.sleep(0.1)

"""
   _____          _        ______           _ 
  / ____|        | |      |  ____|         | |
 | |     ___   __| | ___  | |__   _ __   __| |
 | |    / _ \ / _` |/ _ \ |  __| | '_ \ / _` |
 | |___| (_) | (_| |  __/ | |____| | | | (_| |
  \_____\___/ \__,_|\___| |______|_| |_|\__,_|
                                              
"""       
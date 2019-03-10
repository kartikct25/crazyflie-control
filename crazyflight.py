import time
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
# from cflib.crazyflie.mem import MemoryElement
# from cflib.crazyflie.mem import Poly4D
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger

class CrazyFlight:

    flightStart = None
    flightEnd = None

    cfPos = [0.0, 0.0, 0.0]
    cfEuler = [0.0, 0.0, 0.0]
    cfHeading = 0.0

    cfBattery = 0

    cfInRange = []

    activeCFs = None

    def __init__(self, scf):
        self.flightStart = time.time()
        print('Flight started on', time.ctime(self.flightStart))
        cfInRange = cflib.crtp.scan_interfaces()

        scf.cf.param.set_value('kalman.initialX', '{:1.16f}'.format(0.00))
        scf.cf.param.set_value('kalman.initialY', '{:1.16f}'.format(0.00))

        self.start_position_read(scf)
        # self.start_battery_read(scf)
        

    def __del__(self):
        self.flightEnd = time.time()
        print('Flight ended on', time.ctime(self.flightEnd))
        print('This flight lasted {:1.3f}s'.format(self.flightEnd - self.flightStart))

    def isAirborne(self):
        return True

    def printPosition(self):
        print('Crazyflie is at X = {:1.2f}m, Y = {:1.2f}m, Z = {:1.2f}m, Psi = {:1.2f}deg'.format(self.cfPos[0], self.cfPos[1], self.cfPos[2], self.cfHeading))

    def start_position_read(self, scf):
        log_conf = LogConfig(name='Pose', period_in_ms=50)
        log_conf.add_variable('stateEstimate.x', 'float')
        log_conf.add_variable('stateEstimate.y', 'float')
        log_conf.add_variable('stateEstimate.z', 'float')
        log_conf.add_variable('stabilizer.roll', 'float')
        log_conf.add_variable('stabilizer.pitch', 'float')
        log_conf.add_variable('stabilizer.yaw', 'float')

        scf.cf.log.add_config(log_conf)
        log_conf.data_received_cb.add_callback(self.position_callback)
        log_conf.start()

    def position_callback(self, timestamp, data, logconf):
        self.cfPos[0] = data['stateEstimate.x']
        self.cfPos[1] = data['stateEstimate.y']
        self.cfPos[2] = data['stateEstimate.z']
        self.cfEuler[0] = data['stabilizer.roll']
        self.cfEuler[1] = data['stabilizer.pitch']
        self.cfEuler[2] = data['stabilizer.yaw']
        self.cfHeading = data['stabilizer.yaw']


    def start_battery_read(self, scf):
        log_conf = LogConfig(name='Battery', period_in_ms=5000)
        log_conf.add_variable('pm.vbat', 'float')
        scf.cf.log.add_config(log_conf)
        log_conf.data_received_cb.add_callback(self.battery_callback)
        log_conf.start()

    def battery_callback(self, timestamp, data, longconf):
        self.cfBattery = data['batteryLevel']
        self.cfBattery = data['pm.vbat']

    def findCFs(self):
        available = cflib.crtp.scan_interfaces()
        time.sleep(2.0)

        if len(available) > 0:
            print('Crazyflies found:')
            for i in available:
                print(i[0])
        else:
            print('No Crazyflies found, cannot run example')

    def resetFlightStartTime(self):
        self.flightStart = time.time()
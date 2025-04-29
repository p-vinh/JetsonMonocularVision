#!/usr/bin/python3.10

import atexit
import sys
import time
from threading import Thread
from MavLink import MavLink
from Networking import Network
from Detector import Detector
from Load_Config import Config
import logging

LOGFILE = os.path.join(os.path.dirname(__file__), "Detector_last.log")

logging.basicConfig(
    filename=LOGFILE,
    filemode='w',
    level=logging.INFO,
    format='%(asctime)s %(levelname)s:%(name)s: %(message)s',
    force=True                
)

logger = logging.getLogger("Main")
class Main:
    def __init__(self, conf_dict):
        self.GPS = MavLink(conf_dict['MAVLINK']['directory'], int(conf_dict['MAVLINK']['baud']))
        self.Detection_and_Location = Detector(left_input=conf_dict['DETECTION']['left'],
                                               left_save_name=conf_dict['DETECTION']['left_savename'],
                                               dir_name=conf_dict['DETECTION']['directory'],
                                               camera_pitch=float(conf_dict['DETECTION']['camera_pitch']),
                                               GPS_node=self.GPS,
                                               spread=float(conf_dict['DETECTION']['camera_sep']),
                                               sensor_width=float(conf_dict['DETECTION']['sensor_width']),
                                               fov=float(conf_dict['DETECTION']['camera_fov']))
        self.Network = Network((conf_dict['NETWORKING']['GROUND_STATION_IP'],
                                conf_dict['NETWORKING']['GROUND_STATION_PORT']),
                               (conf_dict['NETWORKING']['HUSKY_IP'],
                                conf_dict['NETWORKING']['HUSKY_PORT']),
                               self.GPS, self.Detection_and_Location,
                               conf_dict['NETWORKING']['logging_interval'],
                               conf_dict['NETWORKING']['mission_cooldown'])

        self.isRunning = True
        self.thread = Thread(target=self.loop, args=())
        self.thread.deamon = True
        self.thread.start()

    def loop(self):
        last_log = 0
        LOG_INTERVAL = 3
        while self.isRunning:
            if last_log + LOG_INTERVAL <= time.time():
                last_log = time.time()
                print("UAV_MAIN:")
                print("    Detector thread is ", end='')
                if self.Detection_and_Location.is_thread_alive():
                    print("ALIVE")
                else:
                    print("NOT ALIVE")

                print("    Networking thread is ", end='')
                if self.Network.is_thread_alive():
                    print("ALIVE")
                else:
                    print("NOT ALIVE")

                print("    Mavlink thread is ", end='')
                if self.GPS.is_thread_alive():
                    print("ALIVE")
                else:
                    print("NOT ALIVE")

    def kill(self):
        self.isRunning = False
        self.thread.join()
        self.Detection_and_Location.kill()
        self.Network.kill()
        self.GPS.kill()


def kill(target):
    target.kill()


if __name__ == '__main__':
    config_dict = Config(sys.argv[1])
    main_class = Main(config_dict.main_dict)
    atexit.register(kill, main_class)

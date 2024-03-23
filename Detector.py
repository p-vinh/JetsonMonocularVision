import atexit
import time
from threading import Thread
from Jetson_Camera_2 import Jetson_Camera
from MavLink import MavLink
from DetectorThreadRunner import DetectorThreadRunner


class Detector:
    def __init__(self, left_input, right_input, left_save_name, right_save_name, dir_name, GPS_node: MavLink,
                 camera_pitch, spread=0.8636, fov=62.2, flip=False):
        self.detection_timestamp = 0
        self.persons_cords = None
        self.logfile = open("Detector_last_log.log", "w")
        self.t_runner = DetectorThreadRunner(Jetson_Camera(str(left_input), dir_name, left_save_name, flip),
                                             Jetson_Camera(str(right_input), dir_name, right_save_name, flip),
                                             pitch=camera_pitch, spread=spread, fov=fov, GPS_node=GPS_node,
                                             log_file=self.logfile)
        if flip:
            self.t_runner.swap_cameras()

        self.isRunning = True
        self.allow_read = True
        self.thread = Thread(target=self.loop, args=())
        self.t_lock = Thread.Lock()
        self.thread.deamon = True

        time.sleep(2.0)
        self.thread.start()

    def loop(self):
        while self.isRunning:
            self.t_runner.thread_loop()
            self.t_lock.acquire()
            self.persons_cords = self.t_runner.person_loc
            self.detection_timestamp = self.t_runner.detection_time
            self.t_lock.release()

    def is_thread_alive(self):
        return self.thread.is_alive()

    def get_person_loc(self):
        temp = None
        self.t_lock.acquire()
        temp = self.persons_cords
        self.t_lock.release()
        return temp

    def get_detection_time(self):
        temp = None
        self.t_lock.acquire()
        temp = self.detection_timestamp
        self.t_lock.release()
        return temp

    def get_data(self):
        temp1 = None
        temp2 = None
        self.t_lock.acquire()
        temp1 = self.persons_cords
        temp2 = self.detection_timestamp
        self.t_lock.release()
        return temp1, temp2

    def kill(self):
        self.isRunning = False
        self.thread.join()
        self.t_runner.kill()
        self.logfile.write("Thread killed, closing the file.")
        self.logfile.close()


def kill(core):
    core.kill()


if __name__ == '__main__':
    GPS = MavLink("/dev/ttyACM0", 57600)
    detector = Detector("0", "1", "test_left", "test_left", "./Recordings", GPS, 0, flip=True)
    atexit.register(kill, detector)

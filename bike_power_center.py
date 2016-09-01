
"""
Rickie Kerndt <rkerndt@cs.uoregon.edu
bike_power_center.py
Provides services for monitoring and managing bicycle electrical systems
"""

from __future__ import print_function
import sys
sys.path.append('/home/pi/lib/python')

import signal
import ina219
import power_meter as pm
import rgb
from threading import Event
from threading import Timer
from datetime import datetime

class PowerCenter():
    """
    Methods for monitoring power generating, consumption, and modifying on bicycle electronics to control
    power consumption.
    """

    def __init__(self):
        """
        Instantiates power center which for now just monitors battery and external power ina219 devices.
        """
        self.trigger = Event().clear()
        self.battery_power = pm.PowerMeter(pm.INA219_Monitor(0.1, 0x40, 1, ina219.INA219_CALIB_32V_1A, 128),
                                           self.trigger)
        self.dyno_power = pm.PowerMeter(pm.INA219_Monitor(0.1, 0x41, 1, ina219.INA219_CALIB_32V_2A, 128),
                                        self.trigger)
        self.log_file = '/var/log/power_center.log'
        self.log_fh = None
        self.battery_status = rgb.RGB_led(21, 20, 16)


    def run(self):


    def close(self):
        """
        Shuts down power_center
        """
        self.battery_power.close()
        self.dyno_power.close()
        self.battery_status.close()
        if self.log_fh:
            self._log_message('Shutting down Power Center')
            self.log_fh.flush()
            self.log_fh.close()
        exit()

    def _log_power_data(self):
        """
        writes the battery and dyno power values to the log file
        """
        if self.log_fh:
            battery_data = self.battery_power.get()
            dyno_data = self.dyno_power.get()
            self._log_message(battery_data.csv() + ',' + dyno_data.csv())

    def _log_message(self, message):
        """
        writes message to log file inserting a timestamp
        :param message: string
        """
        if self.log_fh:
            print('%s message' % datetime.now(), file=self.log_fh)

    def _sig_handler(self, signum, frame):
        """
        Handles system signals. SIGINT, SIGTERM to gracefully terminate operation. SIGUSR1 will toggle
        data logging.
        :param signum:
        :param frame:
        """
        if signum in [signal.SIGTERM, signal.SIGINT]:
            self.close()

        if signum == signal.SIGUSR1:
            if self.log_fh:
                self._log_message('Logging disabled')
                self.log_fh.flush()
                self.log_fh.close()
                self.log_fh = None
            else:
                try:
                    self.log_fh = open(self.log_file, 'a')
                except:
                    print('Failed to open %s for logging: %s' % (self.log_file, sys.exc_info()[0]), sys.stderr)
                    self.log_fh = None
                    return
                self._log_message('Logging enabled')



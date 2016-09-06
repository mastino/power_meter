
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
from threading import Event, Timer
from datetime import datetime

class PowerCenter():
    """
    Methods for monitoring power generating, consumption, and modifying on bicycle electronics to control
    power consumption.
    """

    CHARGING    = 0
    FULL_CHARGE = 1
    ON_BATTERY  = 2
    LOW_BATTERY = 3
    LOW_BATTERY_VOLTS = 3.2

    LOG_FILE = "/var/log/power_center.log"
    RUNNING = 0
    SHUTDOWN = 1
    TERMINATE = 2

    # timer intervals
    CHECK_BATTERY_INTERVAL = 1.0
    LOG_DATA_INTERVAL = 1.0
    POWER_MONITOR_INTERVAL = 0.08

    DEBUG = False

    def __init__(self):
        """
        Instantiates power center which for now just monitors battery and external power ina219 devices.
        """
        self._trigger = Event()
        self._state = None
        self.battery_power = pm.PowerMeter(pm.INA219_Monitor(0.1, 0x40, 1, ina219.INA219_CALIB_32V_1A, 128),
                                           self._trigger)
        self.dyno_power = pm.PowerMeter(pm.INA219_Monitor(0.1, 0x41, 1, ina219.INA219_CALIB_32V_2A, 128),
                                        self._trigger)
        self.log_file = PowerCenter.LOG_FILE
        self.log_fh = None
        self.battery_status = None
        self.battery_status_led = rgb.RGB_led(21, 20, 16)
        self._battery_timer = Timer(PowerCenter.CHECK_BATTERY_INTERVAL, self._check_battery)
        self._log_timer = None
        self._power_monitor_timer = Timer(PowerCenter.POWER_MONITOR_INTERVAL, self._power_monitor_sync)
        self._debug = PowerCenter.DEBUG
        signal.signal(signal.SIGTERM, self._sig_handler)
        signal.signal(signal.SIGINT, self._sig_handler)
        signal.signal(signal.SIGUSR1, self._sig_handler)

    def run(self):
        """
        Starts things up and launches timers
        """
        self._state = PowerCenter.RUNNING
        self.battery_power.open()
        self.dyno_power.open()
        self._power_monitor_timer.start()
        self._battery_timer.start()
        if self.log_fh:
            self._log_timer = Timer(PowerCenter.LOG_DATA_INTERVAL, self._output_log)
            self._log_timer.start()

        while self._state != PowerCenter.TERMINATE:
            result = signal.pause()
            if self._debug:
                self._log_message('Caught Interrupt')


    def close(self):
        """
        Shuts things down and cancels timers. Set status flag to TERMINATE
        """
        # set state
        if self._debug:
            self._log_message('Setting state to SHUTDOWN')
        self._status = PowerCenter.SHUTDOWN

        # shutdown timers
        if self._power_monitor_timer:
            if self._debug:
                self._log_message('Canceling power monitor timer')
            self._power_monitor_timer.cancel()
        if self._log_timer:
            if self._debug:
                self._log_message('Canceling log timer')
            self._log_timer.cancel()
        if self._battery_timer:
            if self._debug:
                self._log_message('Canceling battery timer')
            self._battery_timer.cancel()

        # shutdown power monitors
        if self._debug:
            self._log_message('Stopping battery power meter')
        self.battery_power.stop()
        if self._debug:
            self._log_message('Stopping dyno power meter')
        self.dyno_power.stop()
        self._trigger.set()
        self.battery_power.close()
        if self._debug:
            self._log_message('Battery power meter closed')
        self.dyno_power.close()
        if self._debug:
            self._log_message('Dyno power meter closed')

        # shutdown LED(s)
        if self.battery_status_led:
            self.battery_status_led.close()

        # shutdown logging
        if self.log_fh:
            self._log_message('Shutting down Power Center')
            self.log_fh.flush()
            self.log_fh.close()
            self.log_fh = None

        # set state
        if self._debug:
            self._log_message('Setting state to TERMINATE')
        self._state = PowerCenter.TERMINATE

    def _power_monitor_sync(self):
        """
        Sets event controlling synchronization of power monitors
        """
        self._trigger.set()
        if self._state == PowerCenter.RUNNING:
            self._power_monitor_timer = Timer(PowerCenter.POWER_MONITOR_INTERVAL, self._power_monitor_sync)
            self._power_monitor_timer.start()

    def _check_battery(self):
        """
        Runs battery check and then relaunches timer
        """
        self._update_battery_status()
        if self._state == PowerCenter.RUNNING:
            self._battery_timer = Timer(PowerCenter.CHECK_BATTERY_INTERVAL, self._check_battery)
            self._battery_timer.start()

    def _update_battery_status(self):
        """
        Checks battery voltage and amperage updating LED indication if there is a change
        CHARGING := negative battery amperage
        FULL_CHARGE := positive battery amperage with greater external amperage
        ON_BATTERY := positive battery amperage
        LOW_BATTERY := battery voltage <= LOW_BATTERY_VOLTS
        """
        amperage = self.battery_power.amp
        voltage = self.battery_power.volt
        ext_amperage = self.dyno_power.amp

        if voltage <= PowerCenter.LOW_BATTERY_VOLTS:
            if self.battery_status != PowerCenter.LOW_BATTERY:
                self.battery_status = PowerCenter.LOW_BATTERY
                self.battery_status_led.red()
        elif amperage < 0:
            if self.battery_status != PowerCenter.CHARGING:
                self.battery_status = PowerCenter.CHARGING
                self.battery_status_led.green(True)
        elif ext_amperage > amperage:
            if self.battery_status != PowerCenter.FULL_CHARGE:
                self.battery_status = PowerCenter.FULL_CHARGE
                self.battery_status_led.green()
        else:
            if self.battery_status != PowerCenter.ON_BATTERY:
                self.battery_status = PowerCenter.ON_BATTERY
                self.battery_status_led.blue()

    def _output_log(self):
        """
        Updates logs and relaunches timer
        """
        self._log_power_data()
        if self._state == PowerCenter.RUNNING:
            self._log_timer = Timer(PowerCenter.LOG_DATA_INTERVAL, self._output_log)
            self._log_timer.start()

    def _log_power_data(self):
        """
        writes the battery and dyno power values to the log file
        """
        if self.log_fh:
            battery_data = self.battery_power.get()
            battery_watt_seconds = self.battery_power.watt_seconds
            dyno_data = self.dyno_power.get()
            dyno_watt_seconds = self.dyno_power.watt_seconds
            message = ',%s,%f,%s,%f' % (battery_data.csv(), battery_watt_seconds, dyno_data.csv(), dyno_watt_seconds)
            self._log_message(message)

    def _log_message(self, message):
        """
        writes message to log file inserting a timestamp
        :param message: string
        """
        if self.log_fh:
            print('%s %s' % (datetime.now(), message), file=self.log_fh)
        if self._debug:
            print('%s %s' % (datetime.now(), message), file=sys.stderr)

    def _sig_handler(self, signum, frame):
        """
        Handles system signals. SIGINT, SIGTERM to gracefully terminate operation. SIGUSR1 will toggle
        data logging.
        :param signum:
        :param frame:
        """
        if signum in [signal.SIGTERM, signal.SIGINT]:
            if self._debug:
                self._log_message('Received signal %d' % signum)
            self.close()

        elif signum == signal.SIGUSR1:
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
                self._log_timer = Timer(PowerCenter.LOG_DATA_INTERVAL, self._output_log)
                self._log_timer.start()
                self._log_message('Logging enabled')


def main():
    """
    Instantiates PowerCenter and waits for termination
    """
    pc = PowerCenter()
    pc.run()

if __name__ == "__main__":
    main()
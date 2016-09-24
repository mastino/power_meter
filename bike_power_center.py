
"""
Rickie Kerndt <rkerndt@cs.uoregon.edu
bike_power_center.py
Provides services for monitoring and managing bicycle electrical systems
"""

from __future__ import print_function
import sys
import os
sys.path.append('/home/pi/lib/python')

import signal
import ina219
import power_meter as pm
import rgb
from threading import Event, Timer
from datetime import datetime
import smbus

class PowerCenter():
    """
    Methods for monitoring power generating, consumption, and modifying on bicycle electronics to control
    power consumption.
    """

    # battery states
    CHARGING    = 0
    FULL_CHARGE = 1
    ON_BATTERY  = 2
    LOW_BATTERY = 3
    LOW_BATTERY_VOLTS = 3.2
    LOW_BATTERY_HYSTERESIS = 0.02   # once low battery state is reached voltage must rise this additional
                                    # value to obtain on_battery state
    FULL_CHARGE_AMPS_POS = 0.003    # maximum batt amp draw on full charge with external power
    FULL_CHARGE_AMPS_NEG = 0.000    # maximum batt charge amp considered at full charge

    LOG_FILE = "/var/log/power_center.log"
    DATA_FILE_PATH = "/var/log"
    DATA_FILE_PREFIX = "power_center"
    DATA_FILE_SUFFIX = ".csv"
    PID_FILE = "/var/run/power_center.pid"

    # power center states
    RUNNING = 0
    SHUTDOWN = 1
    TERMINATE = 2

    # timer intervals
    CHECK_BATTERY_INTERVAL = 1.0
    LOG_DATA_INTERVAL = 1.0
    POWER_MONITOR_INTERVAL = 0.08

    DEBUG = False

    # avr constants
    AVR_I2C_BUS = 1
    AVR_I2C_ADDRESS = 0x21
    AVR_BATT_CHRG_REG = 23  # sets the battery charging rate (1/3, 2/3, 1 amp)

    # ina219 defaults
    INA219_I2C_BUS = 1
    BATT_I2C_ADDRESS = 0x40
    EXT_I2C_ADDRESS = 0x41
    INA219_POLL_SECS = 0.1
    INA219_SAMPLES = 128

    # LED BCM pin numbers
    RED_PIN = 21
    GREEN_PIN = 20
    BLUE_PIN = 16

    def __init__(self, i2c_bus=INA219_I2C_BUS, batt_addr=BATT_I2C_ADDRESS, ext_addr=EXT_I2C_ADDRESS):
        """
        Instantiates power center which for now just monitors battery and external power ina219 devices.
        """
        self._trigger = Event()
        self._state = None
        self.battery_power = pm.PowerMeter(pm.INA219_Monitor(PowerCenter.INA219_POLL_SECS, PowerCenter.BATT_I2C_ADDRESS,
                                                             PowerCenter.INA219_I2C_BUS, ina219.INA219_CALIB_32V_1A,
                                                             PowerCenter.INA219_SAMPLES),
                                           self._trigger)
        self.dyno_power = pm.PowerMeter(pm.INA219_Monitor(PowerCenter.INA219_POLL_SECS, PowerCenter.EXT_I2C_ADDRESS,
                                                          PowerCenter.INA219_I2C_BUS, ina219.INA219_CALIB_32V_2A,
                                                          PowerCenter.INA219_SAMPLES),
                                        self._trigger)
        self.log_fh = None
        self.data_fh = None
        self.battery_status = None
        self.battery_status_led = rgb.RGB_led(PowerCenter.RED_PIN, PowerCenter.GREEN_PIN, PowerCenter.BLUE_PIN)
        self._battery_timer = Timer(PowerCenter.CHECK_BATTERY_INTERVAL, self._check_battery)
        self._log_timer = None
        self._power_monitor_timer = Timer(PowerCenter.POWER_MONITOR_INTERVAL, self._power_monitor_sync)
        self._debug = PowerCenter.DEBUG
        self._i2c_bus = smbus.SMBus(PowerCenter.AVR_I2C_BUS)

        signal.signal(signal.SIGTERM, self._sig_handler)
        signal.signal(signal.SIGINT, self._sig_handler)
        signal.signal(signal.SIGUSR1, self._sig_handler)
        signal.signal(signal.SIGHUP, self._sig_handler)

        try:
            fh = open(PowerCenter.PID_FILE, 'w')
            print('%d' % os.getpid(), file=fh)
            fh.close()
        except:
            print('Failed to open %s for pid file: %s' % (PowerCenter.PID_FILE, sys.exc_info()[0]), file=sys.stderr)

    def run(self):
        """
        Starts things up and launches timers
        """
        self._state = PowerCenter.RUNNING
        self.battery_power.open()
        self.dyno_power.open()
        self._power_monitor_timer.start()
        self._battery_timer.start()

        try:
            self.log_fh = open(PowerCenter.LOG_FILE, 'a')
            self._log_message("Power Center starting")
        except:
            print('Failed to open %s for logging: %s' % (PowerCenter.LOG_FILE, sys.exc_info()[0]), file=sys.stderr)
            self.log_fh = None

        while self._state != PowerCenter.TERMINATE:
            signal.pause()
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

    def set_charge_rate(self, rate):
        """
        Sets the charge rate on the AndiceLabs PowerPi
        :param rate: one of {0,1,2,3}
                     0 := charging disabled
                     1 := 1/3 amp
                     2 := 2/3 amp
                     3 := 1 amp
        """
        if rate in [0, 1, 2, 3]:
            if self._debug:
                value = self._i2c_bus.read_byte_data(PowerCenter.AVR_I2C_ADDRESS, PowerCenter.AVR_BATT_CHRG_REG)
                self._log_message('Charge rate was %03f amps' % (value/3))
            self._i2c_bus.write_byte_data(PowerCenter.AVR_I2C_ADDRESS, PowerCenter.AVR_BATT_CHRG_REG, rate)
            if self._debug:
                self._log_message('Charge rate set to %03f amps' % (rate/3))
        else:
            raise ValueError

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
        FULL_CHARGE := positive battery amperage below FULL_CHARGE_AMPS
        ON_BATTERY := positive battery amperage
        LOW_BATTERY := battery voltage <= LOW_BATTERY_VOLTS
        """
        amperage = self.battery_power.amp
        voltage = self.battery_power.volt

        if voltage <= PowerCenter.LOW_BATTERY_VOLTS:
            if self.battery_status != PowerCenter.LOW_BATTERY:
                self._log_message("Low battery after %f wattseconds" % self.battery_power.watt_seconds)
                self.battery_status = PowerCenter.LOW_BATTERY
                self.battery_status_led.red()
        elif amperage < PowerCenter.FULL_CHARGE_AMPS_NEG:
            if self.battery_status != PowerCenter.CHARGING:
                self._log_message("Battery is charging")
                self.battery_status = PowerCenter.CHARGING
                self.battery_status_led.green(True)
        elif amperage < PowerCenter.FULL_CHARGE_AMPS_POS:
            if self.battery_status != PowerCenter.FULL_CHARGE:
                # when transitioning from charging to full charge zero wattseconds
                # so we can improve tracking battery capacity
                if self.battery_status == PowerCenter.CHARGING:
                    self._log_message("Battery has reached full charge")
                    self.battery_power._watt_seconds = 0.0
                self.battery_status = PowerCenter.FULL_CHARGE
                self.battery_status_led.green()
        else:
            if ((self.battery_status != PowerCenter.ON_BATTERY) and
               not ((self.battery_status == PowerCenter.LOW_BATTERY) and
                    (voltage < PowerCenter.LOW_BATTERY_VOLTS + PowerCenter.LOW_BATTERY_HYSTERESIS))):
                self._log_message("On battery")
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
        if self.data_fh:
            battery_data = self.battery_power.get()
            battery_watt_seconds = self.battery_power.watt_seconds
            dyno_data = self.dyno_power.get()
            dyno_watt_seconds = self.dyno_power.watt_seconds
            message = '"%s",%s,%f,%s,%f' % (datetime.now(), battery_data.csv(), battery_watt_seconds, dyno_data.csv(), dyno_watt_seconds)
            print(message, file=self.data_fh)

    def _log_message(self, message):
        """
        writes message to log file inserting a timestamp
        :param message: string
        """
        if self.log_fh:
            print('%s %s' % (datetime.now(), message), file=self.log_fh)
            self.log_fh.flush()
        if self._debug:
            print('%s %s' % (datetime.now(), message), file=sys.stderr)

    def _sig_handler(self, signum, frame):
        """
        Handles system signals. SIGINT, SIGTERM to gracefully terminate operation. SIGUSR1 will toggle
        data logging. SIGHUP restarts log file (send after rotating logs)
        :param signum:
        :param frame:
        """
        if signum in [signal.SIGTERM, signal.SIGINT]:
            if self._debug:
                self._log_message('Received signal %d' % signum)
            self.close()

        elif signum == signal.SIGUSR1:
            if self.data_fh:
                self._log_message('Data logging disabled')
                self.data_fh.flush()
                self.data_fh.close()
                self.data_fh = None
            else:
                mark = datetime.now()
                time_string = mark.strftime('%y%m%d_%H%M%S')
                data_file = PowerCenter.DATA_FILE_PATH + '/' + PowerCenter.DATA_FILE_PREFIX + time_string + PowerCenter.DATA_FILE_SUFFIX
                try:
                    self.data_fh = open(data_file, 'a')
                except:
                    print('Failed to open %s for logging: %s' % (data_file, sys.exc_info()[0]), file=sys.stderr)
                    self.data_fh = None
                    return
                self._log_timer = Timer(PowerCenter.LOG_DATA_INTERVAL, self._output_log)
                self._log_timer.start()
                self._log_message('Data logging enabled')

        elif signum == signal.SIGHUP:
            if self.log_fh:
                self.log_fh.close()
            try:
                self.log_fh = open(PowerCenter.LOG_FILE, 'a')
                self._log_message("Log file restart")
            except:
                print('Failed to open %s for logging: %s' % (PowerCenter.LOG_FILE, sys.exc_info()[0]), file=sys.stderr)
                self.log_fh = None



def main():
    """
    Instantiates PowerCenter and waits for termination
    """
    if len(sys.argv) == 4:
        i2c_bus = int(sys.argv[1])
        batt_addr = int(sys.argv[2])
        ext_addr = int(sys.argv[3])
    else:
        i2c_bus = PowerCenter.INA219_I2C_BUS
        batt_addr = PowerCenter.BATT_I2C_ADDRESS
        ext_addr = PowerCenter.EXT_I2C_ADDRESS

    pc = PowerCenter(i2c_bus, batt_addr, ext_addr)
    pc.run()

if __name__ == "__main__":
    main()
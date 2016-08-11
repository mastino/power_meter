"""
PowerMeter
Rickie Kerndt <rkerndt@cs.uoregon.edu>
"""

from __future__ import print_function
import serial
import datetime
import Queue
from threading import Thread
import copy
from sys import stderr
from time import sleep
import ina219

def output_error_message(msg):
    print(msg, file=stderr)

class PowerData:
    """

    """
    NUM_FIELDS = 7
    VOLT_IDX = 1
    AMP_IDX = 3
    WATT_IDX = 6
    DEFAULT_VOLT_CALIB = 1.0
    DEFAULT_AMP_CALIB = 1.0

    def __init__(self, volts, amps, watts, time_stamp, period, volt_calib = DEFAULT_VOLT_CALIB,
                 amp_calib = DEFAULT_AMP_CALIB):

        self._time_stamp = time_stamp
        self._voltage = volts
        self._ampere = amps
        self._wattage = watts
        self._period = period
        self._volt_calib = volt_calib
        self._amp_calib = amp_calib


    @classmethod
    def from_PowerGauge(cls, data_str, time_stamp, period, volt_calib = DEFAULT_VOLT_CALIB,
                 amp_calib = DEFAULT_AMP_CALIB):
        """

        :param data_str:
        :param time_stamp:
        :param period:
        :param volt_calib:
        :param amp_calib:
        :return:
        """
        try:
            voltage, ampere, wattage = PowerData.parse(data_str)
        except ValueError as err:
            msg = 'value error converting power data %s' % str(err)
            output_error_message(msg)
            raise err
        except TypeError as err:
            msg = 'type error converting power data %s' % str(err)
            output_error_message(msg)
            raise err
        return cls(PowerData(voltage, ampere, wattage, time_stamp, period, volt_calib,
                             amp_calib))

    @property
    def timestamp(self):
        return self._time_stamp

    @staticmethod
    def parse(data_str):
        """

        :param data_str: expects a string in the format of 'v: n.n I: nnn mA Watts: n.n'
        :param timestamp
        """
        fields = data_str.split()
        if len(fields) == PowerData.NUM_FIELDS:
            voltage = float(fields[PowerData.VOLT_IDX])
            ampere = float(fields[PowerData.AMP_IDX]) / 1000
            wattage = float(fields[PowerData.WATT_IDX])
        else:
            raise ValueError

        return voltage, ampere, wattage

    def volt(self, calib=True):
        """
        :param calib
        :return:
        """
        voltage = self._voltage
        if calib:
            voltage *= self._volt_calib
        return voltage


    def amp(self, calib=True):
        """
        :param calib
        :return:
        """
        ampere = self._ampere
        if calib:
            ampere *= self._amp_calib
        return ampere


    def watt(self, calib=True):
        """

        :param calib:
        :return:
        """
        wattage = self._wattage
        if calib:
            wattage = self.volt(True) * self.amp(True)
        return wattage

    def __repr__(self):
        """
        :return: string suitable for input argument of PowerData class
        """
        return 'V: %f I: %f Watts: %f' % (self._voltage, self._ampere, self._wattage)

    def __str__(self):
        """
        :return: string representation which include the time data was aquired
        """
        return '%s %s' % (str(self._time_stamp), self.__repr__())

    @staticmethod
    def csv_header():
        return "datetime,period,volt,amp,watt"

    def csv(self, calib=True):
        return "\"%s\",%s,%s,%s,%s" % (self._time_stamp, self._period.total_seconds() ,self.volt(), self.amp(), self.watt())


class PowerMonitor (Thread):
    """
    Abstract class for power device monitor
    """

    def __init__(self):
        """
        Function called with new powerdata object
        :param callback:
        :return:
        """
        Thread.__init__(self)
        self.callback = None

    def close(self):
        """
        Take actions necessary to shutdown the montior and force run() to terminate
        """
        pass

class PowerGage_Monitor (PowerMonitor):
    """
    Monitors Adafruit PowerGuage on serial port
    """

    def __init__(self, port='/dev/ttyAMA0', baud=9600, timeout=2.0):
        PowerMonitor.__init__(self)
        self._port = port
        self._baud = baud
        self._timeout = timeout        # serial read timeout in seconds (float)
        self._ser = None
        self._serial_timeout_count = 0
        self._monitor = False

    def close(self):
        """
        Sets flag to exit monitoring loop, close serial connection and terminate
        """
        self._monitor = False

    def run(self):
        """
        Opens serial port and starts monitoring of PowerGuage
        """
        try:
            self._ser = serial.Serial(port=self._port, baudrate=self._baud, timeout=self._timeout)
        except ValueError as err:
            msg = 'Invalid Serial argument: %s' % str(err)
            output_error_message(msg)
            self._ser = None
        except serial.SerialException as err:
            msg = 'Error opening serial port %s' % self._port
            output_error_message(msg)
            self._ser = None

        if self._ser:
            self._monitor = True

        while self._monitor:
            data_str = None
            try:
                data_str = self._ser.readline()
            except ValueError as err:
                # shouldn't happen
                msg = 'ValueError in monitor: %s' % str(err)
                output_error_message(msg)
            except serial.SerialTimeoutException as err:
                self._serial_timeout_count += 1
                msg = 'Serial timeout: (%d)' % self._serial_timeout_count
                output_error_message(msg)
            except serial.SerialException as err:
                # shouldn't happen
                msg = 'SerialException in monitor: %s' % str(err)
                output_error_message(msg)

            if data_str:
                try:
                    power_data = PowerData.from_PowerGauge(data_str, None, None, None, None)
                except:
                    power_data = None

                if power_data:
                    self.callback(power_data)

        if self._ser:
            self._ser.close()


class INA219_Monitor (PowerMonitor):
    """
    Monitors any ina219 device on smbus
    """

    def __init__(self, interval, addr, i2c_device_num, calibrator=ina219.INA219_CALIB_32V_2A):
        """
        Runs as a concurrent thread to read from an ina219 device attached to local i2c (SMBus).
        :param callback: method to handle latest power data object
        :param interval: float in seconds. Read interval for ina219
        :param addr: int - i2c bus address of ina219 device
        :param i2c_device_num: int - bus number of i2c device ina219 is attached (e.g /dev/i2c-n where n
                                     is i2c_device_num
        """
        PowerMonitor.__init__(self)
        self._meter = ina219.INA219(addr, i2c_device_num)
        self._meter.setCalibration(calibrator)
        self._monitor = False
        self._interval = interval

    def close(self):
        """
        Sets flag to stop monitoring
        """
        self._monitor = False

    def run(self):
        """
        Sets calibration of ina219 and starts monitoring
        :return:
        """

        self._meter.begin()
        self._monitor = True

        while self._monitor:

            voltage = self._meter.getBusVoltage_V()
            ampere = self._meter.getCurrent_mA() / 1000
            power = self._meter.getPower_mW() / 1000

            power_data = PowerData(voltage, ampere, power, None, None, None, None)
            self.callback(power_data)
            sleep(self._interval)

        self._meter.close()

class PowerMeter:
    """

    """
    def __init__(self, monitor):
        """
        :param monitor
        """
        self._debug = False

        self._monitor = monitor
        self._monitor.callback = self._update

        self._volt_calib = 1.0       # vector of calibration values for voltage
        self._amp_calib = 1.0        # vector of calibration values for amperage

        self._epoch = None             # timestamp (time.datetime) for start of monitoring
        self._last = None              # most recent power values received on ttl
        self._avg_period = None        # period (time span in time.deltatime) of power values
        self._queue = Queue.Queue(maxsize=10)

        self._watt_seconds = 0.0
        self._calib_watt_seconds = 0.0


    def volt(self, calib=True):
        """
        :param calib
        :return:
        """
        return self._last.volt(calib)


    def amp(self, calib=True):
        """
        :param calib
        :return:
        """
        return self._last.amp(calib)


    def watt(self, calib=True):
        """

        :param calib:
        :return:
        """
        # TODO add ability to override calibration for ina219 devices
        # TODO which provide their own calibration mechanism
        return self._last.watt(False)


    def next(self, block=True):
        """

        :param block
        :return:
        """
        return self._queue.get(block)


    def get_current(self):
        """

        :return:
        """
        return copy.deepcopy(self._last)

    def calibrate(self, volt_calib, amp_calib):
        """
        Set calibration vectors for voltage and amperage.
        :param volt_calib: tuple containing single constant
        :param amp_calib: tuple containing single constant
        """

        msg = 'oops'
        error = False
        if not (isinstance(volt_calib, tuple) and isinstance(amp_calib, tuple)):
            msg = 'expected tuple'
            error = True
        if (not error) and len(volt_calib) != 1:
            msg = 'expected single constant for volt calibrator'
            error = True
        if (not error) and len(amp_calib) != 1:
            msg = 'expected single constant for amp calibrator'
            error = True
        if (not error) and 0 < volt_calib[0]:
            msg = 'voltage const %d is zero or negative' % volt_calib[0]
            error = True
        if (not error) and 0 < amp_calib[0]:
            msg = 'amperage const %d is zero or negative' % amp_calib[0]
            error = True
        if not error:
            self._volt_calib = volt_calib
            self._amp_calib = amp_calib
        else:
            output_error_message(msg)
            raise ValueError

    def open(self):
        """


        """
        self._epoch = datetime.datetime.now()
        self._monitor.start()

    def close(self):
        """


        """
        try:
            if self._monitor:
                self._monitor.close()
        except:
            pass

        self._monitor = None

    def _update(self, power_data):
        """
        Given as callback method to monitor. Takes PowerData object, computes the period, and updates
        data structures.
        """

        if power_data:
            timestamp = datetime.datetime.now()
            if self._last:
                period = timestamp - self._last.timestamp
                self._avg_period = (0.8 * self._avg_period) + (0.2 * period.total_seconds())
            else:
                period = timestamp - self._epoch
                self._avg_period = period.total_seconds()

            # update with time and calibration information
            power_data._timestamp = timestamp
            power_data._period = period
            power_data._volt_calib = self._volt_calib
            power_data._amp_calib = self._amp_calib

            # update data structures
            self._last = copy.deepcopy(power_data)
            self._watt_seconds += self._last.watt(False) * period.total_seconds()
            self._calib_watt_seconds += self._last.watt(True) * period.total_seconds()

            if self._queue.full():
                self._queue.get(False)
            self._queue.put(power_data)



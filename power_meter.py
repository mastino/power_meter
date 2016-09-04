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
    Data structure for holding basic power data volts, amps, and watts. Provides a timestamp (datetime.datetime object)
    and period (datetime.timedelta object). Period is the length of time over which the values apply for the purpose of
    calculating watt hours.
    """

    def __init__(self, volts, amps, watts, timestamp, period):
        """

        :param volts: float
        :param amps: float
        :param watts: float
        :param time_stamp: datetime.datetime
        :param period: datetime.timedelta
        """
        self._timestamp = timestamp
        self._voltage = volts
        self._ampere = amps
        self._wattage = watts
        self._period = period

    @property
    def timestamp(self):
        """
        :return: datetime.datetime
        """
        return self._timestamp

    @property
    def volt(self):
        """
        :return: float
        """
        return self._voltage

    @property
    def milli_volt(self):
        """
        :return: float
        """
        return self._voltage * 1000

    @property
    def amp(self):
        """
        :return: float
        """
        return self._ampere

    @property
    def milli_amp(self):
        """
        :return: float
        """
        return self._ampere * 1000

    @property
    def watt(self):
        """
        :return: float
        """
        return self._wattage

    @property
    def milli_watt(self):
        """
        :return: float
        """
        return self._wattage * 1000

    @property
    def watt_seconds(self):
        """
        :return: float
        """
        return self._wattage * self._period.total_seconds()

    @property
    def watt_hours(self):
        """
        :return: float
        """
        return self.watt_seconds / 3600


    def __repr__(self):
        """
        :return: string unsuitable for input arguments of PowerData class
        """
        return 'PowerData(%f, %f, %f, %s, %f)' % (self._voltage, self._ampere, self._wattage, self._timestamp,
                                                  self._period.total_seconds())

    def __str__(self):
        """
        :return: string representation which include the time data was aquired
        """
        return '%s volt:%f amp:%f watt:%f period:%f' % (self._timestamp, self._voltage, self._ampere, self._wattage,
                                                        self._period.total_seconds())

    @staticmethod
    def csv_header():
        return "datetime,period,volt,amp,watt"

    def csv(self):
        return "\"%s\",%s,%s,%s,%s" % (self._timestamp, self._period.total_seconds() ,self.volt, self.amp,
                                       self.watt)


class PowerMonitor (Thread):
    """
    Abstract class for power device monitor
    """

    def __init__(self):
        """
        :param callback function called with a new PowerData object
        :param trigger threading.Event object. Use is optional, if present the monitor must wait on this event object
               to synchronize power reads.
        """
        Thread.__init__(self)
        self.callback = None
        self.trigger = None

    def close(self):
        """
        Take actions necessary to shutdown the monitor and force run() to terminate
        """
        pass

class PowerGage_Monitor (PowerMonitor):
    """
    Monitors Adafruit PowerGuage on serial port. Since the PowerGage generates data at roughly 1 second intervals
    without any mechanism to control this, the trigger Event object is ignored when present. Callback is made
    with a PowerData object at each receipt of valid data from the PowerGage.
    """

    # Constants for parsing adafruit PowerGauge serial data
    NUM_FIELDS = 7
    VOLT_IDX = 1
    AMP_IDX = 3
    WATT_IDX = 6

    def __init__(self, port='/dev/ttyAMA0', baud=9600, timeout=2.0):
        PowerMonitor.__init__(self)
        self._port = port
        self._baud = baud
        self._timeout = timeout        # serial read timeout in seconds (float)
        self._ser = None
        self._serial_timeout_count = 0
        self._monitor = False
        self._last_timestamp = None

    def close(self):
        """
        Sets flag to exit monitoring loop, close serial connection and terminate
        """
        self._monitor = False

    @staticmethod
    def parse(data_str):
        """
        Parses one line of serial stream from the PowerGage.
        :param data_str: expects a string in the format of 'v: n.n I: nnn mA Watts: n.n'
        :param timestamp
        """
        fields = data_str.split()
        if len(fields) == PowerGage_Monitor.NUM_FIELDS:
            voltage = float(fields[PowerGage_Monitor.VOLT_IDX])
            ampere = float(fields[PowerGage_Monitor.AMP_IDX]) / 1000
            wattage = float(fields[PowerGage_Monitor.WATT_IDX])
        else:
            raise ValueError

        return voltage, ampere, wattage


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
                    voltage, amperage, wattage = PowerGage_Monitor.parse(data_str)
                    power_data = PowerData(voltage, amperage, wattage, None, None)
                except:
                    power_data = None

                if power_data:
                    timestamp = datetime.datetime.now()
                    if self._last_timestamp:
                        period = timestamp - self._last_timestamp
                        self._last_timestamp = timestamp
                    else:
                        period = datetime.timedelta()
                        self._last_timestamp = timestamp
                    power_data._time_stamp = timestamp
                    power_data._period = period
                    self._last_timestamp = timestamp
                    self.callback(power_data)

        if self._ser:
            self._ser.close()


class INA219_Monitor (PowerMonitor):
    """
    Monitors any ina219 device on smbus
    """

    def __init__(self, interval, addr, i2c_device_num, calibrator=ina219.INA219_CALIB_32V_2A, sample=1):
        """
        Runs as a concurrent thread to read from an ina219 device attached to local i2c (SMBus).
        :param interval: float in seconds. Read interval for ina219
        :param addr: int - i2c bus address of ina219 device
        :param i2c_device_num: int - bus number of i2c device ina219 is attached (e.g /dev/i2c-n where n
                                     is i2c_device_num
        :param number of samples to use for current measurement. See ina219.INA219 for details
        """
        PowerMonitor.__init__(self)
        self._meter = ina219.INA219(addr, i2c_device_num)
        self._meter.setCalibration(calibrator,sample)
        self._monitor = False
        self._interval = interval
        self._last_timestamp = None

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

            # use trigger or interval but not both
            if self.trigger:
                self.trigger.clear()
                self.trigger.wait()
            else:
                sleep(self._interval)

            voltage = self._meter.getBusVoltage_V()
            ampere = self._meter.getCurrent_mA() / 1000
            power = self._meter.getPower_mW() / 1000

            timestamp = datetime.datetime.now()
            if self._last_timestamp:
                period = timestamp - self._last_timestamp
                self._last_timestamp = timestamp
            else:
                period = datetime.timedelta()
                self._last_timestamp = timestamp
            power_data = PowerData(voltage, ampere, power, timestamp, period)
            self.callback(power_data)

        self._meter.close()

class PowerMeter:
    """

    """
    def __init__(self, monitor, trigger=None):
        """
        :param monitor PowerMonitor derived object
        :param trigger threading.Event object providing external synchronization of PowerMonitor object(s)
        """
        self._debug = False

        self._monitor = monitor
        self._monitor.callback = self._update
        self._monitor.trigger = trigger
        self._epoch = None             # timestamp (datetime.datetime) for start of monitoring
        self._last = None              # most recent power values received on ttl
        self._avg_period = None        # period (time span in datetime.timedelta) of power values
        self._maxsize = 10             # maximum size for PowerData queue
        self._queue = Queue.Queue(maxsize=self._maxsize)

        self._watt_seconds = 0.0

    @property
    def volt(self):
        """
        :return: float
        """
        return self._last.volt

    @property
    def amp(self):
        """
        :return: float
        """
        return self._last.amp

    @property
    def watt(self):
        """
        :return: float
        """
        return self._last.watt

    @property
    def watt_seconds(self):
        """
        Watt seconds of power consumption since epoch
        :return: float
        """
        return self._watt_seconds

    @property
    def watt_hours(self):
        """
        Watt hours of power consumption since epoch
        :return: float
        """
        return self._watt_seconds / 3600

    def next(self, block=True):
        """
        Returns the next PowerData object from the queue.
        :param block True/False blocks if PowerData queue is empty
        :return: PowerData object
        """
        return self._queue.get(block)


    def get(self):
        """
        The most recent PowerData object
        :return: PowerData object
        """
        return copy.deepcopy(self._last)


    def open(self):
        """
        Start power monitoring
        """
        self._epoch = datetime.datetime.now()
        self._monitor.start()

    def close(self):
        """
        Stop power monitoring and close resources
        """

        if self._monitor:
            self._monitor.close()
            self._monitor.join()
            self._monitor = None

    def _update(self, power_data):
        """
        Given as callback method to monitor. Takes PowerData object, computes the period, and updates
        data structures.
        """

        if power_data:
            if self._last:
                self._avg_period = (0.8 * self._avg_period) + (0.2 * power_data._period.total_seconds())
            else:
                period = power_data.timestamp - self._epoch
                self._avg_period = period.total_seconds()

            # update data structures
            self._last = copy.deepcopy(power_data)
            self._watt_seconds += self._last.watt * self._last._period.total_seconds()

            if self._queue.full():
                self._queue.get(False)
            self._queue.put(power_data)



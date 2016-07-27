"""
PowerMeter
Rickie Kerndt <rkerndt@cs.uoregon.edu>
"""

from __future__ import print_function
import serial
import datetime
import Queue
import thread
import copy
from sys import stderr, exc_info


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

    def __init__(self, data_str, time_stamp, period, volt_calib = DEFAULT_VOLT_CALIB,
                 amp_calib = DEFAULT_AMP_CALIB):

        self._time_stamp = time_stamp
        self._voltage = None
        self._ampere = None
        self._wattage = None
        self._period = period
        self._volt_calib = volt_calib
        self._amp_calib = amp_calib

        try:
            self._voltage, self._ampere, self._wattage = PowerData.parse(data_str)
        except ValueError as err:
            msg = 'value error converting power data %s' % str(err)
            output_error_message(msg)
            raise err
        except TypeError as err:
            msg = 'type error converting power data %s' % str(err)
            output_error_message(msg)
            raise err

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

class PowerMeter:
    """

    """
    def __init__(self, port='/dev/ttyAMA0', baud=9600, timeout=2.0):
        """
        :param port:     (string) device path for serial port
        :param baud:     (int) baud rate for ttl communications
        :param timeout: (float) serial read timeout in seconds
        """
        self._debug = True
        self._port = port
        self._baud = baud
        self._timeout = timeout        # serial read timeout in seconds (float)

        self._ser = None
        self._monitor = None
        self._serial_timeout_count = 0

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
        return self._last.watt(calib)


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
        self.close()
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
            try:
                self._epoch = datetime.datetime.now()
                if self._debug:
                    print('Starting thread ...',end='',file=stderr)
                self._monitor = thread.start_new_thread(self._monitor_thread, ())
                if self._debug:
                    print(' started',file=stderr)
            except:
                msg = 'Error starting monitor thread: %s' % exc_info()[0]
                output_error_message(msg)
                self._monitor = None
                self._epoch = None


    def close(self):
        """


        """
        try:
            if self._monitor:
                self._monitor.exit()
            if self._ser:
                self._ser.close()
        except:
            pass

        self._monitor = None
        self._ser = None


    def _monitor_thread(self):
        """
        Runs as a concurrent thread to read from Power Gauge updating power values. This funtion
        is also responsible for handling serial read and connectivity errors attempting to maintain
        connection to the Power Guage.
        """
        while True:
            data_str = None
            try:
                if self._debug:
                    print('Waiting for data ...',end='',file=stderr)
                data_str = self._ser.readline()
                if self._debug:
                    print('got %s' % data_str, file=stderr)
            except ValueError as err:
                # shouldn't happen
                msg = 'ValueError in monitor: %s' % str(err)
                output_error_message(msg)
            except serial.SerialException as err:
                # shouldn't happen
                msg = 'SerialException in monitor: %s' % str(err)
                output_error_message(msg)
            except serial.SerialTimeoutException as err:
                self._serial_timeout_count += 1
                msg = 'Serial timeout: (%d)' % self._serial_timeout_count
                output_error_message(msg)

            if data_str:
                timestamp = datetime.datetime.now()
                if self._last:
                    period = timestamp - self._last.timestamp
                    self._avg_period = (0.8 * self._avg_period) + (0.2 * period.total_seconds())
                else:
                    period = timestamp - self._epoch
                    self._avg_period = period.total_seconds()

                try:
                    power_data = PowerData(data_str, timestamp, period, self._volt_calib, self._amp_calib)
                    if self._debug:
                        print(power_data)
                except:
                    power_data = None

                if power_data:
                    self._last = copy.deepcopy(power_data)
                    self._watt_seconds += self._last.watt(False) * period.total_seconds()
                    self._calib_watt_seconds += self._last.watt(True) * period.total_seconds()

                    if self._queue.full():
                        self._queue.get(False)
                    self._queue.put(power_data)



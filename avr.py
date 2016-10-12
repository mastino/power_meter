"""
Rickie Kerndt <rkerndt@cs.uoregon.edu>
Module for controlling Andice Labs PowerCape avr
"""

from __future__ import print_function
import smbus
import sys

class AVR:
    """
    Monitor and controll operation of Andice Labs PowerCape avr
    """

    # AVR constants
    I2C_BUS = 1
    I2C_ADDRESS = 0x21
    STATUS_INIT = 0
    STATUS_OK = 1
    STATUS_FAILED = 2
    STATUS_ERROR = 3

    # AVR registers
    REG_MCUSR = 0             # AVR register
    REG_OSCCAL = 1            # AVR register
    REG_STATUS = 2
    REG_CONTROL = 3
    REG_START_ENABLE = 4
    REG_START_REASON = 5
    REG_RESTART_HOURS = 6     # Countdown hours
    REG_RESTART_MINUTES = 7   # Countdown minutes
    REG_RESTART_SECONDS = 8   # Countdown seconds
    REG_SECONDS_0 = 9         # Uptime counter/clock
    REG_SECONDS_1 = 10        # "
    REG_SECONDS_2 = 11        # "
    REG_SECONDS_3 = 12        # "
    REG_EXTENDED = 13         # Indicator that extended register set follows
    REG_CAPABILITY = 14       # Firmware version/feature "level"
    REG_BOARD_TYPE = 15       # Board type (ie: BeagleBone, Pi, etc.)
    REG_BOARD_REV = 16        # Hardware revision (if known) in ASCII (ie: 'A')
    REG_BOARD_STEP = 17       # Hardware stepping (if known) in ASCII (ie: '1')
    REG_WDT_RESET = 18        # Reset watchdog countdown register (seconds, 0 to disable)
    REG_WDT_POWER = 19        # Power-cycle watchdog countdown register (seconds, 0 to disable)
    REG_WDT_STOP = 20         # Power-off countdown (single-shot seconds, 0 to disable)
    REG_WDT_START = 21        # Start-up activity watchdog countdown (seconds, 0 to disable)
    REG_I2C_ADDRESS = 22      # Slave address to use on I2C interface
    REG_I2C_ICHARGE = 23      # Charge current (0-3)/3 amp
    REG_I2C_TCHARGE = 24      # Charger timer in hours (3-10)

    # STATUS register bits
    STATUS_POWER_GOOD = 0x01  # PG state
    STATUS_BUTTON = 0x02      # Button state
    STATUS_OPTO = 0x04        # Opto state
    
    # CONTROL register bits
    CONTROL_CE = 0x01
    CONTROL_LED0 = 0x02
    CONTROL_LED1 = 0x04
    CONTROL_BOOTLOAD = 0x80
    
    # START enable and reason register bits
    START_BUTTON = 0x01
    START_EXTERNAL = 0x02
    START_PWRGOOD = 0x04
    START_TIMEOUT = 0x08
    START_ALL = 0x0F
    
    # CAPABILITY levels
    CAPABILITY_RTC = 0x00     # The presence of the "extended" register alone indicates RTC
    CAPABILITY_WDT = 0x01     # Board type, revision level, and watchdog functionality
    CAPABILITY_ADDR = 0x02    # Programmable I2C address
    CAPABILITY_CHARGE = 0x03  # Programmable charge current and timer
    CAPABILITY_STATUS = 0x04  # Current button and opto state in status register
    
    # Board types
    BOARD_TYPE_BONE = 0x00
    BOARD_TYPE_PI = 0x01
    BOARD_TYPE_UNKNOWN = 0xFF

    # Charge rates
    CHARGE_RATE_ZERO = 0      # disables battery charging
    CHARGE_RATE_LOW = 1       # 1/3 amp max
    CHARGE_RATE_MED = 2       # 2/3 amp max
    CHARGE_RATE_HIGH = 3      # 1 amp max
    CHARGE_RATE_OK = (CHARGE_RATE_LOW, CHARGE_RATE_MED, CHARGE_RATE_HIGH)

    def __init__(self, address=I2C_ADDRESS, bus_num=I2C_BUS):
        """

        :param address:
        :param bus:
        """
        self._status = AVR.STATUS_INIT
        self._i2c_address = address
        self._i2c_bus_num = bus_num
        self._i2c_bus = smbus.SMBus(self._i2c_bus_num)

    def __repr__(self):
        """
        String representation of self
        """
        return("AVR(0x%x, %d" % (self._i2c_address, self._i2c_bus_num))

    def close(self):
        """
        Release resources
        """
        self._i2c_bus.close()

    def get_charge_rate(self):
        """
        Returns the current charge rate setting from the avr
        :return: int
        """
        value = None
        try:
            value = self._i2c_bus.read_byte_data(self._i2c_address, AVR.REG_I2C_ICHARGE)
            self._status = AVR.STATUS_OK
        except IOError:
            print("AVR i2c read failed for charge rate", file=sys.stderr)
            self._status = AVR.STATUS_ERROR

        return value

    def set_charge_rate(self, rate):
        """
        Sets the battery charge rate on the AVR
        :param rate: int in CHARGE_RATE_OK
        :return:
        """
        if rate in AVR.CHARGE_RATE_OK:
            try:
                self._i2c_bus.write_byte_data(self._i2c_address, AVR.REG_I2C_ICHARGE, rate)
                self._status = AVR.STATUS_OK
            except IOError:
                print("AVR i2c write failed for charge rate", file=sys.stderr)
                self._status = AVR.STATUS_ERROR




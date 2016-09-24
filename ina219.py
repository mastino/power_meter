"""
Rickie Kerndt <rkerndt@cs.uoregon.edu>
Python port of Adafruit_INA219 C++ code for accessing INA219 power measurements
derived from Adafruit_INA219.{h,cpp}. See ina219_license.txt.

Required python modules:
    python-smbus (from lm-sensors.org) part of debian python-smbus package. This is
    included with the raspi-jessi distro so should not need to install.

/**************************************************************************/
/*!
    @file     Adafruit_INA219.h
    @author   K. Townsend (Adafruit Industries)
    @license  BSD (see license.txt)

    This is a library for the Adafruit INA219 breakout board
    ----> https:#www.adafruit.com/products/???

    Adafruit invests time and resources providing this open source code,
    please support Adafruit and open-source hardware by purchasing
    products from Adafruit!

    @section  HISTORY

    v1.0  - First release
*/
/**************************************************************************/
"""

import smbus
from time import sleep

# ina219 shunt resistance
INA219_SHUNT_OHM = 0.1


# /*=========================================================================
#    I2C ADDRESS/BITS
#    -----------------------------------------------------------------------*/
INA219_ADDRESS                         = 0x40   # 1000000 A0+A1=GND
INA219_READ                            = 0x01
# /*=========================================================================*/

# /*=========================================================================
#    CONFIG REGISTER R/W
#    -----------------------------------------------------------------------*/
INA219_REG_CONFIG                      = 0x00
# /*---------------------------------------------------------------------*/
INA219_CONFIG_RESET                    = 0x8000  # Reset Bit

INA219_CONFIG_BVOLTAGERANGE_MASK       = 0x2000  # Bus Voltage Range Mask
INA219_CONFIG_BVOLTAGERANGE_16V        = 0x0000  # 0-16V Range
INA219_CONFIG_BVOLTAGERANGE_32V        = 0x2000  # 0-32V Range
# allowed bus voltage ranges
INA219_CONFIG_BVOLTAGERANGE_OK = frozenset([INA219_CONFIG_BVOLTAGERANGE_16V,INA219_CONFIG_BVOLTAGERANGE_32V])

INA219_CONFIG_GAIN_MASK                = 0x1800  # Gain Mask
INA219_CONFIG_GAIN_1_40MV              = 0x0000  # Gain 1, 40mV Range
INA219_CONFIG_GAIN_2_80MV              = 0x0800  # Gain 2, 80mV Range
INA219_CONFIG_GAIN_4_160MV             = 0x1000  # Gain 4, 160mV Range
INA219_CONFIG_GAIN_8_320MV             = 0x1800  # Gain 8, 320mV Range
# index of shunt gain config to maximum amperage
INA219_IDX_GAIN_TO_MILLIAMP = ((INA219_CONFIG_GAIN_1_40MV, 0.040/INA219_SHUNT_OHM),
                               (INA219_CONFIG_GAIN_2_80MV, 0.080/INA219_SHUNT_OHM),
                               (INA219_CONFIG_GAIN_4_160MV, 0.160/INA219_SHUNT_OHM),
                               (INA219_CONFIG_GAIN_8_320MV, 0.320/INA219_SHUNT_OHM))

INA219_CONFIG_BADCRES_MASK             = 0x0780  # Bus ADC Resolution Mask
INA219_CONFIG_BADCRES_9BIT             = 0x0080  # 9-bit bus res = 0..511
INA219_CONFIG_BADCRES_10BIT            = 0x0100  # 10-bit bus res = 0..1023
INA219_CONFIG_BADCRES_11BIT            = 0x0200  # 11-bit bus res = 0..2047
INA219_CONFIG_BADCRES_12BIT            = 0x0400  # 12-bit bus res = 0..4097
    
INA219_CONFIG_SADCRES_MASK             = 0x0078  # Shunt ADC Resolution and Averaging Mask
INA219_CONFIG_SADCRES_9BIT_1S_84US     = 0x0000  # 1 x 9-bit shunt sample
INA219_CONFIG_SADCRES_10BIT_1S_148US   = 0x0008  # 1 x 10-bit shunt sample
INA219_CONFIG_SADCRES_11BIT_1S_276US   = 0x0010  # 1 x 11-bit shunt sample
INA219_CONFIG_SADCRES_12BIT_1S_532US   = 0x0018  # 1 x 12-bit shunt sample
INA219_CONFIG_SADCRES_12BIT_2S_1060US  = 0x0048	 # 2 x 12-bit shunt samples averaged together
INA219_CONFIG_SADCRES_12BIT_4S_2130US  = 0x0050  # 4 x 12-bit shunt samples averaged together
INA219_CONFIG_SADCRES_12BIT_8S_4260US  = 0x0058  # 8 x 12-bit shunt samples averaged together
INA219_CONFIG_SADCRES_12BIT_16S_8510US = 0x0060  # 16 x 12-bit shunt samples averaged together
INA219_CONFIG_SADCRES_12BIT_32S_17MS   = 0x0068  # 32 x 12-bit shunt samples averaged together
INA219_CONFIG_SADCRES_12BIT_64S_34MS   = 0x0070  # 64 x 12-bit shunt samples averaged together
INA219_CONFIG_SADCRES_12BIT_128S_69MS  = 0x0078  # 128 x 12-bit shunt samples averaged together
# allowed shunt ADC configuration
INA219_CONFIG_SADCRES_OK = frozenset([INA219_CONFIG_SADCRES_12BIT_1S_532US, INA219_CONFIG_SADCRES_12BIT_2S_1060US,
                                      INA219_CONFIG_SADCRES_12BIT_4S_2130US, INA219_CONFIG_SADCRES_12BIT_8S_4260US,
                                      INA219_CONFIG_SADCRES_12BIT_16S_8510US,INA219_CONFIG_SADCRES_12BIT_32S_17MS,
                                      INA219_CONFIG_SADCRES_12BIT_64S_34MS, INA219_CONFIG_SADCRES_12BIT_128S_69MS])
INA219_CONFIG_MODE_MASK                = 0x0007  # Operating Mode Mask
INA219_CONFIG_MODE_POWERDOWN           = 0x0000
INA219_CONFIG_MODE_SVOLT_TRIGGERED     = 0x0001
INA219_CONFIG_MODE_BVOLT_TRIGGERED     = 0x0002
INA219_CONFIG_MODE_SANDBVOLT_TRIGGERED = 0x0003
INA219_CONFIG_MODE_ADCOFF              = 0x0004
INA219_CONFIG_MODE_SVOLT_CONTINUOUS    = 0x0005
INA219_CONFIG_MODE_BVOLT_CONTINUOUS    = 0x0006
INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS = 0x0007
# allowed operating modes
INA219_CONFIG_MODE_OK = frozenset([INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS,INA219_CONFIG_MODE_SANDBVOLT_TRIGGERED])

# /*=========================================================================*/

# /*=========================================================================
#    SHUNT VOLTAGE REGISTER R
#    -----------------------------------------------------------------------*/
INA219_REG_SHUNTVOLTAGE                = 0x01
# /*=========================================================================*/

# /*=========================================================================
#    BUS VOLTAGE REGISTER R
#    -----------------------------------------------------------------------*/
INA219_REG_BUSVOLTAGE                  = 0x02
# /*=========================================================================*/

# /*=========================================================================
#    POWER REGISTER R
#    -----------------------------------------------------------------------*/
INA219_REG_POWER                       = 0x03
# /*=========================================================================*/

# /*=========================================================================
#    CURRENT REGISTER R
#    -----------------------------------------------------------------------*/
INA219_REG_CURRENT                     = 0x04
# /*=========================================================================*/

# /*=========================================================================
#    CALIBRATION REGISTER R/W
#    -----------------------------------------------------------------------*/
INA219_REG_CALIBRATION                 = 0x05
# /*=========================================================================*/

# default device for I2C bus
INA219_I2C_DEVICE_NUM = 1  # corresponds to /dev/i2c-1

# permitted expected ampere range
INA219_MIN_EXP_AMP = 0.040 / INA219_SHUNT_OHM
INA219_MAX_EXP_AMP = 0.320 / INA219_SHUNT_OHM

# maximimum number of current bins for 12bit shunt adc
INA219_MAX_ADC_RES = 32767

# ina219 internal current scaling constant
INA219_CURRENT_SCALING = 0.04096

# ordered list of lsb rounded to divisor of current scaling.
# these values cover the amp range of 0.4 to 3.2A full scale
INA219_ROUNDED_LSB = (0.00001,    # 0.37267A
                      0.0000128,  # 0.4194176A
                      0.000016,   # 0.524272A
                      0.00002,    # 0.65534A
                      0.000032,   # 1.048544A
                      0.00004,    # 1.31068A
                      0.0000512,  # 1.6776704A
                      0.000064,   # 2.097088A
                      0.00008,    # 2.62136A
                      0.0001)     # 3.2767A

# defined calibration methods
INA219_CALIB_32V_2A    = 0
INA219_CALIB_32V_1A    = 1
INA219_CALIB_16V_400mA = 2

# status constants
INA219_STATUS_INIT = 0
INA219_STATUS_FAILED = 1  #calibration failed
INA219_STATUS_OK = 2      #successful calibration
INA219_STATUS_ERROR = 3   #IO error in last read/write

class INA219:

  def __init__(self, addr = INA219_ADDRESS, n = INA219_I2C_DEVICE_NUM):
      """
      Instantiates a new INA219 class.
      Requires i2c kernal modules installed and i2c_bus set to device num
      that ina219 sensor is connected (e.g /dev/i2c-n) default is
      :param addr: I2C address of INA219
      :param i2c_bus: device number for i2c bus where ina210 is attached.
                      (e.g. /dev/i2c-n where n is integer) default is
                      /dev/i2c-1
      """
      self.ina219_i2c_addr = addr
      self.ina219_calValue = 0
      self.ina219_current_lsb = 0
      self.ina219_config = 0
      self.ina219_currentDivider_mA = 0
      self.ina219_powerDivider_mW = 0
      self.ina219_bus_num = n
      self.i2c_bus = smbus.SMBus(n)

      # default values for calibration and sampling
      self._calibrator = INA219_CALIB_32V_2A
      self._sampling = 1
      self._status = INA219_STATUS_INIT


  def __bool__(self):
      """
      Calibration state of device. Returns true if ina219 was successfully calibrated otherwise false.
      :return: True/False
      """
      return not ((self._status == INA219_STATUS_INIT) or (self._status == INA219_STATUS_FAILED))

  def __str__(self):
      """
      Generate a string with the current configuration.
      :return:  string
      """
      fmt = 'i2c_addr=0x%x, config=0x%x, currentDivider_mA=%f, powerDivider_mW=%f, calValue=%d'
      return fmt % (self.ina219_i2c_addr, self.ina219_config, self.ina219_currentDivider_mA,
                     self.ina219_powerDivider_mW, self.ina219_calValue)

  def __repr__(self):
      """
      Generate a string representation of self
      :return: string
      """
      fmt = 'INA219(0x%x,%d)'
      return fmt % (self.ina219_i2c_addr, self.ina219_bus_num)


  def begin(self):
      """
      Configures ina219.
      """
      self._reset()
      sleep(0.1)
      self.setCalibration(self._calibrator, self._sampling)


  def close(self):
      """
      Resets ina219 device.
      """
      self._reset()

  def setCalibration(self, calibrator, samples=1):
      """
      Sets calibration values for ina219 using 'pre-configured settings' along
      with user selected current sample averaging.

      :param calibrator: one of INA219_CALIB_32V_2A (max 32 volt and 2 amp range),
                                INA219_CALIB_32V_1A (max 32 volt and 1 amp range),
                                INA219_CALIB_16V_400mA (max 16V and 400mA range)

      :param samples: number of samples to average for current, this directly effects cycle time of
                      ADC updates:
                      num samples - update time
                                1 -  532 usec
                                2 -  1.06 msec
                                4 -  2.13 msec
                                8 -  4.26 msec
                               16 -  8.51 msec
                               32 - 17.02 msec
                               64 - 34.05 msec
                              128 - 68.10 msec
      """
      if samples == 1:
          ina219_config_shunt = INA219_CONFIG_SADCRES_12BIT_1S_532US
      elif samples == 2:
          ina219_config_shunt = INA219_CONFIG_SADCRES_12BIT_2S_1060US
      elif samples == 4:
          ina219_config_shunt = INA219_CONFIG_SADCRES_12BIT_4S_2130US
      elif samples == 8:
          ina219_config_shunt = INA219_CONFIG_SADCRES_12BIT_8S_4260US
      elif samples == 16:
          ina219_config_shunt = INA219_CONFIG_SADCRES_12BIT_16S_8510US
      elif samples == 32:
          ina219_config_shunt = INA219_CONFIG_SADCRES_12BIT_32S_17MS
      elif samples == 64:
          ina219_config_shunt = INA219_CONFIG_SADCRES_12BIT_64S_34MS
      elif samples == 128:
          ina219_config_shunt = INA219_CONFIG_SADCRES_12BIT_128S_69MS
      else:
          raise ValueError

      if calibrator == INA219_CALIB_32V_2A:
          self._configuration(INA219_CONFIG_BVOLTAGERANGE_32V, 2.0, ina219_config_shunt,
                              INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS)
      elif calibrator == INA219_CALIB_32V_1A:
          self._configuration(INA219_CONFIG_BVOLTAGERANGE_32V, 1.0, ina219_config_shunt,
                              INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS)
      elif calibrator == INA219_CALIB_16V_400mA:
          self._configuration(INA219_CONFIG_BVOLTAGERANGE_16V, 0.4, ina219_config_shunt,
                              INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS)
      else:
          raise ValueError

      self._calibrator = calibrator
      self._sampling = samples
      self._write_ina219_config()

  @property
  def full_scale_amp(self):
      """
      Provides the value for full scale (max) ampere based on the configuration
      :return: float
      """
      return float(self.ina219_current_lsb * INA219_MAX_ADC_RES)

  @property
  def full_scale_watt(self):
      """
      Provides the value for full scale (max) watts based on the configuration
      :return: float
      """
      return float(self.ina219_current_lsb * 20 * INA219_MAX_ADC_RES)

  @property
  def full_scale_volt(self):
      """
      Provides the value for full scale (max0 volt based on the configuration
      :return: int
      """
      volt = 32
      if self._calibrator == INA219_CALIB_16V_400mA:
          volt = 16
      return volt

  def getBusVoltage_V(self):
      """
      Gets the bus voltage in volts
      :return: float
      """
      return self._getBusVoltage_raw() * 0.001

  def getShuntVoltage_mV(self):
      """
      Gets the shunt voltage in mV (+/- 327mV)
      :return: float
      """
      return self._getShuntVoltage_raw() * .01

  def getCurrent_mA(self):
      """
      Gets the current value in mA, taking into account the
      config settings and current LSB
      :return: float
      """
      return float(self._getCurrent_raw()) / self.ina219_currentDivider_mA

  def getPower_mW(self):
      """
      Gets the current power in mW, taking into account the config settings.
      This provides the power draw computed on the ina219 device and due to
      read timings will be a more accurate representation than computing
      power from getCurrent and getBusVoltage.
      :return: float
      """
      return float(self._getPower_raw()) / self.ina219_powerDivider_mW

  # private class methods:

  @staticmethod
  def _host_to_i2c(value):
      """
      Swaps byte order for 16bit word. Caution: this does not
      check for +/- overflow nor does it deal with negative
      values since we shouldn't be writing twos complement to
      the ina219 device.
      :param value: int
      :return: int
      """
      return  ((value & 0xFF) << 8) | ((value & 0xFF00) >> 8)

  @staticmethod
  def _i2c_to_host(value):
      """
      Converts i2c 16 bit twos complement word to python int
      :param value: int
      :return: int
      """
      value = ((value & 0xFF) << 8) | ((value & 0xFF00) >> 8)

      # Note: can't just extend 1 to most significant bit
      # since python will happily treat this as an overflow
      # and convert int to long (python2.7). So do this in an
      # around about way by first taking the complement,
      # add 1, clear the high bits, and finally make the result
      # negative
      if value >= 0x8000:
          value = -((~value + 1) & 0xFFFF)

      return value

  def _wireWriteRegister(self, reg, value):
      """
      Sends a single command byte over I2C
      :param reg: unsigned 8 bit register value
      :param value: unsigned 16bit value
      """
      value = INA219._host_to_i2c(value)
      self.i2c_bus.write_word_data(self.ina219_i2c_addr, reg, value)

  def _wireReadRegister(self, reg):
      """
      Reads 16 bit values over I2C
      :param reg:
      :return: int
      """
      value = self.i2c_bus.read_word_data(self.ina219_i2c_addr, reg)
      return INA219._i2c_to_host(value)

  def _getBusVoltage_raw(self):
      """
      Gets the raw bus voltage (16-bit signed integer, so +/- 32767
      :return:
      """
      value = 0
      try:
        value = self._wireReadRegister(INA219_REG_BUSVOLTAGE)
        self._status = INA219_STATUS_OK
      except IOError:
          self._status = INA219_STATUS_ERROR

      # Shift to the reight 3 to drop CNVR and OVF and multiply by LSB
      return (value >> 3) * 4

  def _getShuntVoltage_raw(self):
      """
      Gets the raw shunt voltage (16-bit signed integer, so +/- 32767
      :return:
      """
      value = 0
      try:
          value = self._wireReadRegister(INA219_REG_SHUNTVOLTAGE)
          self._status = INA219_STATUS_OK
      except IOError:
          self._status = INA219_STATUS_ERROR
      return value

  def _getCurrent_raw(self):
      """
      Gets the raw current value (16-bit signed integer, so +/- 32767
      :return:
      """
      #TODO if ina219 occassionaly resets then need to do more than below
      #TODO but first see how much a problem this really presents
      #TODO before deciding what to do
      # Sometimes a sharp load will reset the INA219, which will
      # reset the cal register, meaning CURRENT and POWER will
      # not be available ... avoid this by always setting a cal
      # value even if it's an unfortunate extra step
      #self._wireWriteRegister(INA219_REG_CALIBRATION, self.ina219_calValue)

      # Now we can safely read the CURRENT register!
      value = 0
      try:
          value = self._wireReadRegister(INA219_REG_CURRENT)
          self._status = INA219_STATUS_OK
      except IOError:
          self._status = INA219_STATUS_ERROR
      return value

  def _getPower_raw(self):
      """
      Gets the raw power value (16-bit signed integer, so +/- 32767. Power
      is computed on ina219 from voltage and current registers. These
      intermediate values are not computed simultaneously and are separated
      by delays determined from the configuration dependent on bit resolution
      and sampling. However, the timing provided by computing this on the ina219
      device is going to give a more accurate representation of power draw
      then computing from bus voltage and current reads. Hence, due to timing
      of reads do not expect power calculations with getBusVoltage and getCurrent
      to exactly match getPower
      :return: float
      """
      value = 0
      try:
          value = self._wireReadRegister(INA219_REG_POWER)
          self._status = INA219_STATUS_OK
      except IOError:
          self._status = INA219_STATUS_ERROR
      return value

  def _reset(self):
      """
      Sets the reset bit on INA219 configuration register to reset device. This will
      clear calibration/configuration of the ina219. Run begin() to restore
      configuration before continuing use.
      """
      try:
          self._wireWriteRegister(INA219_REG_CONFIG, INA219_CONFIG_RESET)
          self._status = INA219_STATUS_INIT
      except IOError:
          self._status = INA219_STATUS_ERROR


  # The following multipliers are used to convert raw current and power
  # values to mA and mW, taking into account the current config settings

  def _configuration(self, bus_volt_range, expected_max_amp, shunt_adc_res_sample, mode):
      """
      A more generalized ina219 configuration and calibration. Limited to using 12bit adc resolution
      but allows flexibility in selecting bus voltage range, current (shunt voltage) range, and
      current (shunt voltage) sampling.

      NOTE: This configuration method assumes the ina219 is using a 0.1 Ohm shunt resister. This is true
      for Andice Labs PowerPi and the Adafruit INA219 High Side DC Current Sensor.
      :param bus_volt_range: INA219_CONFIG_BVOLTAGERANGE_16V or INA219_CONFIG_BVOLTAGERANGE_32V

      :param expected_max_ampere: integer in milli ampere (allowed range: 400mA to 3200mA)

      :param shunt_adc_res_sample: only 12bit adc resolution allowed, the number of shunt volatage samples
                                   averages taken determines the overall ADC cycle time.
                                   select 1-128 sample averageing from:
                                        INA219_CONFIG_SADCRES_12BIT_1S_532US
                                        INA219_CONFIG_SADCRES_12BIT_2S_1060US
                                        INA219_CONFIG_SADCRES_12BIT_4S_2130US
                                        INA219_CONFIG_SADCRES_12BIT_8S_4260US
                                        INA219_CONFIG_SADCRES_12BIT_16S_8510US
                                        INA219_CONFIG_SADCRES_12BIT_32S_17MS
                                        INA219_CONFIG_SADCRES_12BIT_64S_34MS
                                        INA219_CONFIG_SADCRES_12BIT_128S_69MS

      :param mode: only triggered or continuous allowed:
                        INA219_CONFIG_MODE_SANDBVOLT_TRIGGERED
                        INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS

      The following configuration was taken from the Texas Instruments INA219 application
      document (SB0S448G-August 2008-Revised December 2015) along with insight from K. Townsend's calibration
      methods.
      """

      # input validation
      if bus_volt_range not in INA219_CONFIG_BVOLTAGERANGE_OK:
          raise ValueError
      if not (INA219_MIN_EXP_AMP <= expected_max_amp <= INA219_MAX_EXP_AMP):
          raise ValueError
      if shunt_adc_res_sample not in INA219_CONFIG_SADCRES_OK:
          raise ValueError

      # Calibration Register = trunc ( 0.04096 / (Current_LSB x R_shunt))
      #
      # R_shunt is the shunt resistance which is always assumed a value of 0.1 ohm.
      #
      # Current_LSB is the current per least significant bit and is set to maximum expected current divided
      # by ADC bit resolution (only 2^15 is supported). This value is calculated from the expected_max_amp
      # argument.
      #
      # The current register and power register hold values as determined by the following formulas:
      #     Current Register = (Shunt Voltage Register x Calibration Register) / 4096
      #     Power Register = (Current Register x Bus Voltage Register) / 5000
      #
      # TODO: the cal constant can be extended to externaly calibrate the ina219

      self.ina219_current_lsb = expected_max_amp / INA219_MAX_ADC_RES

      # round up current_lsb to a divisor of the current scaling value. Since scaling factor is
      # 4096 then lsb must be 2^n to get a divisor. Rounding up the lsb in this manner
      # avoids introducing a bias when truncating the calibration value.
      for lsb in INA219_ROUNDED_LSB:
          if lsb >= self.ina219_current_lsb:
              self.ina219_current_lsb = lsb
              break

      self.ina219_calValue = int(INA219_CURRENT_SCALING / (self.ina219_current_lsb * INA219_SHUNT_OHM))

      # Current in ampere is obtained from the current register by multiplying by the current_LSB. In turn
      # the power in watts is obtained by mulitplying the power register by power_LSB which is 20 times
      # the current_LSB. The following constants are provided to produce milli-ampere and milliwatt conversion
      # as a divisor in keeping with original code.
      self.ina219_currentDivider_mA = (1/(self.ina219_current_lsb * 1000))
      self.ina219_powerDivider_mW = (1/(self.ina219_current_lsb * 1000 * 20))

      # Actual configuration of the ina219.
      #
      # Shunt gain is determined from the expected_max_amp. Selects gain which provides the lesser maximum_ampere
      # which is greater or equal to the expected_max_amp.
      for gain_config, max_milliamp in INA219_IDX_GAIN_TO_MILLIAMP:
          if max_milliamp >= expected_max_amp:
              break

      # Set the configuration options
      self.ina219_config = (bus_volt_range |
                            gain_config |
                            INA219_CONFIG_BADCRES_12BIT |
                            shunt_adc_res_sample |
                            mode)

  def _write_ina219_config(self):
      """
      Writes configuration and calibration values to ina219 registers
      """
      try:
          if self.ina219_calValue != 0 and self.ina219_config != 0:
              self._wireWriteRegister(INA219_REG_CONFIG, self.ina219_config)
              self._wireWriteRegister(INA219_REG_CALIBRATION, self.ina219_calValue)
              self._status = INA219_STATUS_OK
      except IOError:
          self._status = INA219_STATUS_FAILED



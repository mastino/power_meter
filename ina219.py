"""
Rickie Kerndt <rkerndt@cs.uoregon.edu>
Python port of Adafruit_INA219 C++ code for accessing INA219 power measurements

Required python modules:
    smbus-cffi (tested with version 0.5.1)

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

INA219_CONFIG_GAIN_MASK                = 0x1800  # Gain Mask
INA219_CONFIG_GAIN_1_40MV              = 0x0000  # Gain 1, 40mV Range
INA219_CONFIG_GAIN_2_80MV              = 0x0800  # Gain 2, 80mV Range
INA219_CONFIG_GAIN_4_160MV             = 0x1000  # Gain 4, 160mV Range
INA219_CONFIG_GAIN_8_320MV             = 0x1800  # Gain 8, 320mV Range

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

INA219_CONFIG_MODE_MASK                = 0x0007  # Operating Mode Mask
INA219_CONFIG_MODE_POWERDOWN           = 0x0000
INA219_CONFIG_MODE_SVOLT_TRIGGERED     = 0x0001
INA219_CONFIG_MODE_BVOLT_TRIGGERED     = 0x0002
INA219_CONFIG_MODE_SANDBVOLT_TRIGGERED = 0x0003
INA219_CONFIG_MODE_ADCOFF              = 0x0004
INA219_CONFIG_MODE_SVOLT_CONTINUOUS    = 0x0005
INA219_CONFIG_MODE_BVOLT_CONTINUOUS    = 0x0006
INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS = 0x0007	
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
      self.ina219_currentDivider_mA = 0
      self.ina219_powerDivider_mW = 0
      self.i2c_bus = smbus.SMBus(n)

      # default calibration is set to 32V and 2A, use appropriate setCalibration_nnV_nA
      # to use something different.
      self.setCalibration_32V_2A()

  def setCalibration_32V_2A(self):
      """
      Configures to INA219 to be able to measure up to 32V and 2A of current
      Each unit of current corresponds to 100uA, and each unit of power corresponds
      to 2mW. Counter overflow occurs at 3.2A

      These calculations assume a 0.1 ohm resistor is present.
      """

      # By default we use a pretty huge range for the input voltage,
      # which probably isn't the most appropriate choice for system
      # that don't use a lot of power.  But all of the calculations
      # are shown below if you want to change the settings.  You will
      # also need to change any relevant register settings, such as
      # setting the VBUS_MAX to 16V instead of 32V, etc.
    
      # VBUS_MAX = 32V             (Assumes 32V, can also be set to 16V)
      # VSHUNT_MAX = 0.32          (Assumes Gain 8, 320mV, can also be 0.16, 0.08, 0.04)
      # RSHUNT = 0.1               (Resistor value in ohms)
      
      # 1. Determine max possible current
      # MaxPossible_I = VSHUNT_MAX / RSHUNT
      # MaxPossible_I = 3.2A
      
      # 2. Determine max expected current
      # MaxExpected_I = 2.0A
      
      # 3. Calculate possible range of LSBs (Min = 15-bit, Max = 12-bit)
      # MinimumLSB = MaxExpected_I/32767
      # MinimumLSB = 0.000061              (61uA per bit)
      # MaximumLSB = MaxExpected_I/4096
      # MaximumLSB = 0,000488              (488uA per bit)
      
      # 4. Choose an LSB between the min and max values
      #    (Preferrably a roundish number close to MinLSB)
      # CurrentLSB = 0.0001 (100uA per bit)
      
      # 5. Compute the calibration register
      # Cal = trunc (0.04096 / (Current_LSB * RSHUNT))
      # Cal = 4096 (0x1000)
      
      self.ina219_calValue = 4096
      
      # 6. Calculate the power LSB
      # PowerLSB = 20 * CurrentLSB
      # PowerLSB = 0.002 (2mW per bit)
      
      # 7. Compute the maximum current and shunt voltage values before overflow
      #
      # Max_Current = Current_LSB * 32767
      # Max_Current = 3.2767A before overflow
      #
      # If Max_Current > Max_Possible_I then
      #    Max_Current_Before_Overflow = MaxPossible_I
      # Else
      #    Max_Current_Before_Overflow = Max_Current
      # End If
      #
      # Max_ShuntVoltage = Max_Current_Before_Overflow * RSHUNT
      # Max_ShuntVoltage = 0.32V
      #
      # If Max_ShuntVoltage >= VSHUNT_MAX
      #    Max_ShuntVoltage_Before_Overflow = VSHUNT_MAX
      # Else
      #    Max_ShuntVoltage_Before_Overflow = Max_ShuntVoltage
      # End If
      
      # 8. Compute the Maximum Power
      # MaximumPower = Max_Current_Before_Overflow * VBUS_MAX
      # MaximumPower = 3.2 * 32V
      # MaximumPower = 102.4W
      
      # Set multipliers to convert raw current/power values
      self.ina219_currentDivider_mA = 10  # Current LSB = 100uA per bit (1000/100 = 10)
      self.ina219_powerDivider_mW = 2     # Power LSB = 1mW per bit (2/1)
    
      # Set Calibration register to 'Cal' calculated above	
      self._wireWriteRegister(INA219_REG_CALIBRATION, self.ina219_calValue)
      
      # Set Config register to take into account the settings above
      config = (INA219_CONFIG_BVOLTAGERANGE_32V |
                INA219_CONFIG_GAIN_8_320MV |
                INA219_CONFIG_BADCRES_12BIT |
                INA219_CONFIG_SADCRES_12BIT_1S_532US |
                INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS)
      self._wireWriteRegister(INA219_REG_CONFIG, config)


  def setCalibration_32V_1A(self):
      """
      Configures INA219 to be able to measure up to 32V and 1A
      of current. Each unit of current corresponds to 40uA, and each
      unit of power corresponds to 800mW. Counter overflow occurs at
      1.3A.

      These calculations assume a 0.1 ohm resistor is present
      """
      # By default we use a pretty huge range for the input voltage,
      # which probably isn't the most appropriate choice for system
      # that don't use a lot of power.  But all of the calculations
      # are shown below if you want to change the settings.  You will
      # also need to change any relevant register settings, such as
      # setting the VBUS_MAX to 16V instead of 32V, etc.
    
      # VBUS_MAX = 32V		(Assumes 32V, can also be set to 16V)
      # VSHUNT_MAX = 0.32	(Assumes Gain 8, 320mV, can also be 0.16, 0.08, 0.04)
      # RSHUNT = 0.1			(Resistor value in ohms)
    
      # 1. Determine max possible current
      # MaxPossible_I = VSHUNT_MAX / RSHUNT
      # MaxPossible_I = 3.2A
    
      # 2. Determine max expected current
      # MaxExpected_I = 1.0A
    
      # 3. Calculate possible range of LSBs (Min = 15-bit, Max = 12-bit)
      # MinimumLSB = MaxExpected_I/32767
      # MinimumLSB = 0.0000305             (30.5�A per bit)
      # MaximumLSB = MaxExpected_I/4096
      # MaximumLSB = 0.000244              (244�A per bit)
    
      # 4. Choose an LSB between the min and max values
      #    (Preferrably a roundish number close to MinLSB)
      # CurrentLSB = 0.0000400 (40�A per bit)
    
      # 5. Compute the calibration register
      # Cal = trunc (0.04096 / (Current_LSB * RSHUNT))
      # Cal = 10240 (0x2800)
    
      self.ina219_calValue = 10240
      
      # 6. Calculate the power LSB
      # PowerLSB = 20 * CurrentLSB
      # PowerLSB = 0.0008 (800�W per bit)
    
      # 7. Compute the maximum current and shunt voltage values before overflow
      #
      # Max_Current = Current_LSB * 32767
      # Max_Current = 1.31068A before overflow
      #
      # If Max_Current > Max_Possible_I then
      #    Max_Current_Before_Overflow = MaxPossible_I
      # Else
      #    Max_Current_Before_Overflow = Max_Current
      # End If
      #
      # ... In this case, we're good though since Max_Current is less than MaxPossible_I
      #
      # Max_ShuntVoltage = Max_Current_Before_Overflow * RSHUNT
      # Max_ShuntVoltage = 0.131068V
      #
      # If Max_ShuntVoltage >= VSHUNT_MAX
      #    Max_ShuntVoltage_Before_Overflow = VSHUNT_MAX
      # Else
      #    Max_ShuntVoltage_Before_Overflow = Max_ShuntVoltage
      # End If
    
      # 8. Compute the Maximum Power
      # MaximumPower = Max_Current_Before_Overflow * VBUS_MAX
      # MaximumPower = 1.31068 * 32V
      # MaximumPower = 41.94176W
    
      # Set multipliers to convert raw current/power values
      self.ina219_currentDivider_mA = 25      # Current LSB = 40uA per bit (1000/40 = 25)
      self.ina219_powerDivider_mW = 1         # Power LSB = 800�W per bit
    
      # Set Calibration register to 'Cal' calculated above	
      self._wireWriteRegister(INA219_REG_CALIBRATION, self.ina219_calValue)
    
      # Set Config register to take into account the settings above
      config = (INA219_CONFIG_BVOLTAGERANGE_32V |
                        INA219_CONFIG_GAIN_8_320MV |
                        INA219_CONFIG_BADCRES_12BIT |
                        INA219_CONFIG_SADCRES_12BIT_1S_532US |
                        INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS)
      self._wireWriteRegister(INA219_REG_CONFIG, config)



  def setCalibration_16V_400mA(self):
      """
      Calibration which uses the highest precision for
      current measuremtn (0.1mA), at the expense of only supporting 16V
      at 400mA max.
      """
      # VBUS_MAX = 16V
      # VSHUNT_MAX = 0.04          (Assumes Gain 1, 40mV)
      # RSHUNT = 0.1               (Resistor value in ohms)

      # 1. Determine max possible current
      # MaxPossible_I = VSHUNT_MAX / RSHUNT
      # MaxPossible_I = 0.4A

      # 2. Determine max expected current
      # MaxExpected_I = 0.4A

      # 3. Calculate possible range of LSBs (Min = 15-bit, Max = 12-bit)
      # MinimumLSB = MaxExpected_I/32767
      # MinimumLSB = 0.0000122              (12uA per bit)
      # MaximumLSB = MaxExpected_I/4096
      # MaximumLSB = 0.0000977              (98uA per bit)

      # 4. Choose an LSB between the min and max values
      #    (Preferrably a roundish number close to MinLSB)
      # CurrentLSB = 0.00005 (50uA per bit)

      # 5. Compute the calibration register
      # Cal = trunc (0.04096 / (Current_LSB * RSHUNT))
      # Cal = 8192 (0x2000)

      self.ina219_calValue = 8192

      # 6. Calculate the power LSB
      # PowerLSB = 20 * CurrentLSB
      # PowerLSB = 0.001 (1mW per bit)

      # 7. Compute the maximum current and shunt voltage values before overflow
      #
      # Max_Current = Current_LSB * 32767
      # Max_Current = 1.63835A before overflow
      #
      # If Max_Current > Max_Possible_I then
      #    Max_Current_Before_Overflow = MaxPossible_I
      # Else
      #    Max_Current_Before_Overflow = Max_Current
      # End If
      #
      # Max_Current_Before_Overflow = MaxPossible_I
      # Max_Current_Before_Overflow = 0.4
      #
      # Max_ShuntVoltage = Max_Current_Before_Overflow * RSHUNT
      # Max_ShuntVoltage = 0.04V
      #
      # If Max_ShuntVoltage >= VSHUNT_MAX
      #    Max_ShuntVoltage_Before_Overflow = VSHUNT_MAX
      # Else
      #    Max_ShuntVoltage_Before_Overflow = Max_ShuntVoltage
      # End If
      #
      # Max_ShuntVoltage_Before_Overflow = VSHUNT_MAX
      # Max_ShuntVoltage_Before_Overflow = 0.04V

      # 8. Compute the Maximum Power
      # MaximumPower = Max_Current_Before_Overflow * VBUS_MAX
      # MaximumPower = 0.4 * 16V
      # MaximumPower = 6.4W

      # Set multipliers to convert raw current/power values
      self.ina219_currentDivider_mA = 20  # Current LSB = 50uA per bit (1000/50 = 20)
      self.ina219_powerDivider_mW = 1     # Power LSB = 1mW per bit

      # Set Calibration register to 'Cal' calculated above
      self._wireWriteRegister(INA219_REG_CALIBRATION, self.ina219_calValue);

      # Set Config register to take into account the settings above
      config = (INA219_CONFIG_BVOLTAGERANGE_16V |
                        INA219_CONFIG_GAIN_1_40MV |
                        INA219_CONFIG_BADCRES_12BIT |
                        INA219_CONFIG_SADCRES_12BIT_1S_532US |
                        INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS)
      self._wireWriteRegister(INA219_REG_CONFIG, config)
      

  def getBusVoltage_V(self):
      """
      Gets the bus voltage in volts
      :return: float
      """
      return self._getBusVoltage_raw() * 0.001

  def getShuntVoltage_mV(self):
      """
      Gets the shunt voltage in mV (+/- 327mV)
      :return:
      """
      return self._getShuntVoltage_raw() * .01

  def getCurrent_mA(self):
      """
      Gets the current value in mA, taking into account the
      config settings and current LSB
      :return:
      """
      return self._getCurrent_raw() / self.ina219_currentDivider_mA

  # private class methods:

  # The following multipliers are used to convert raw current and power
  # values to mA and mW, taking into account the current config settings

  def _ina219_currentDivider_mA(self):
      pass

  def _ina219_powerDivider_mW(self):
      pass
  
  def _wireWriteRegister(self, reg, value):
      """
      Sends a single command byte over I2C
      :param reg:
      :param value:
      """
      self.i2c_bus.write_byte_data(self.ina219_i2c_addr, reg, value)

  def _wireReadRegister(self, reg):
      """
      Reads 16 bit values over I2C
      :param reg:
      :return: int
      """
      value = self.i2c_bus.read_byte(self.ina219_i2c_addr, reg)
      return value

  def _getBusVoltage_raw(self):
      """
      Gets the raw bus voltage (16-bit signed integer, so +/- 32767
      :return:
      """
      value = self._wireReadRegister(INA219_REG_BUSVOLTAGE)

      # Shift to the reight 3 to drop CNVR and OVF and multiply by LSB
      return (value >> 3) * 4

  def _getShuntVoltage_raw(self):
      """
      Gets the raw shunt voltage (16-bit signed integer, so +/- 32767
      :return:
      """
      return self._wireReadRegister(INA219_REG_SHUNTVOLTAGE)

  def _getCurrent_raw(self):
      """
      Gets the raw current value (16-bit signed integer, so +/- 32767
      :return:
      """
      # Sometimes a sharp load will reset the INA219, which will
      # reset the cal register, meaning CURRENT and POWER will
      # not be available ... avoid this by always setting a cal
      # value even if it's an unfortunate extra step
      self._wireWriteRegister(INA219_REG_CALIBRATION, self.ina219_calValue)

      # Now we can safely read the CURRENT register!
      return self._wireReadRegister(INA219_REG_CURRENT)


# This software is subject to the license described in the
# LICENSE_A+SS.txt file included with this software distribution.
# You may not use this file except in compliance with this license.
#
# Copyright (c) Garmin Canada Inc. 2018
# All rights reserved.

# This scripts implements and decodes a received manufacturer specific page

# CALIBRATION INFO
# WEIGHT : 12.27kg = 120.4N

import clr
import time
clr.AddReference('ANT+ProfileLib')
from AntPlus.Profiles.Components import DataPage
from AntPlus.Profiles.BikePower import BikePowerDisplay, SetCustomCalibrationParameterPage 
from AntPlus.Types import BindableByteArray
from System import Array, Byte, BitConverter, Int32
from System.Collections.Generic import List
import System

def stopScript():
   simulator.TurnOff()

   
simulator.TurnOn()
page = SetCustomCalibrationParameterPage()

# byte 0
# 1 = SCALE
# 2 = ?
byte_0 = Byte(1)

# bytes 1-4 : values (little endian 32 bits int)
bytes_value = BitConverter.GetBytes(Int32(1971))

# byte 5 : reserved future use
byte_5 = Byte(0)

x = List[Byte]([byte_0] + [Byte(b) for b in bytes_value] + [byte_5])
array = BindableByteArray(x)
# page.Encode(array)
page.ManufacturerSpecificData = array
simulator.SendSetCustomCalibrationParameterPage(page)
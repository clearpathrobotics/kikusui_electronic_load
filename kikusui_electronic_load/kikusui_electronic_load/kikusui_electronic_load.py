#!/usr/bin/env python3
# Software License Agreement (BSD)
#
# @author    Rhys Faultless <rfaultless@clearpathrobotics.com>
# @copyright (c) 2023, Clearpath Robotics, Inc., All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# * Redistributions of source code must retain the above copyright notice,
#   this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# * Neither the name of Clearpath Robotics nor the names of its contributors
#   may be used to endorse or promote products derived from this software
#   without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import serial
import time
from threading import Lock
import rclpy
from rclpy.node import Node
from kikusui_electronic_load_interfaces.srv import ChangeLoad
from std_msgs.msg import Bool

class KikusuiElectronicLoad(Node):
    def __init__(self):
        super().__init__('KikusuiElectronicLoad')
        self.port = '/dev/ttyUSB0'
        self.baud = 19200
        self.serial_port = serial.Serial(self.port, self.baud, timeout=None)
        self.SERIAL_READ_SIZE = 25
        self.serial_lock = Lock()

        self.operation_mode = "CP"  # "constant power" mode

        self.service_change_load = self.create_service(ChangeLoad, '/change_load', self.change_load)


    def command(self, command):
      command = str(command)
      self.clear_settings()

      time.sleep(1)
      self.serial_lock.acquire()
      self.serial_port.flush()
      self.serial_port.write(bytes(command, encoding='utf-8'))
      self.serial_lock.release()
      time.sleep(1)

      return


    def clear_settings(self):
      time.sleep(1)
      self.serial_lock.acquire()
      self.serial_port.flush()
      self.serial_port.write(bytes('*RST\n', encoding='utf-8'))
      self.serial_lock.release()
      time.sleep(1)
      return


    def load(self, on_off_setting):
      return ("INP " + str(on_off_setting) + "\n")


    def set_operation_mode(self, mode):
      return str("SOUR:FUNC:MODE " + mode + "\n")


    def set_power_range(self, power):
      if int(power) > 19:
        range_setting = "HIGH"
      elif int(power) < 2:
        range_setting = "LOW"
      else:
        range_setting = "MED"
      return ("SOUR:POW:RANG " + range_setting + "\n")


    def set_power(self, power):
      return ("SOUR:POW " + str(power) + "\n")


    def change_load(self, request, response):
        if(request.enable):
            state = str("ON")
        else:
            state = str("OFF")

        self.command(self.set_operation_mode(self.operation_mode) + self.set_power_range(request.power) \
            + self.set_power(request.power) + self.load(state) + "\r")

        response.load_response_bool = True
        return response
        
    
def main():
    rclpy.init()
    kikusui_electronic_load = KikusuiElectronicLoad()
    rclpy.spin(kikusui_electronic_load)
    kikusui_electronic_load.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
   main()

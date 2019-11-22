#!/usr/bin/env pybricks-micropython
from robot_functions import *

def diagnose():
  bool success = True
  #---
  try:
      motor_r = Motor(Port.C)
  except OSError:
      brick.display.text("motor_r port C", (30, 20))
      success = False
  #---
  try:
      motor_l = Motor(Port.B)
  except OSError:
      brick.display.text("motor_l port B", (30, 30))
      success = False
  #---
  try:
      function_r = Motor(Port.D)
  except OSError:
      brick.display.text("function_r port D", (30, 40))
      success = False
  #---
  try:
      function_l = Motor(Port.A)
  except OSError:
      brick.display.text("function_l port A", (30, 50))
      success = False
  #---
  try:
      col_l = ColorSensor(Port.S2)
  except OSError:
      brick.display.text("col_l port 3", (30, 60))
      success = False
  #---
  try:
      col_r = ColorSensor(Port.S3)
  except OSError:
      brick.display.text("col_r port 4", (30, 70))
      success = False
  #---
  # brick.light(Color.PURPLE)
  # #battery
  # if brick.battery.voltage() < 7000:
  #     brick.display.text("battery",(30,80))
  if success:
    brick.light(Color.ORANGE)
    for i in range(4)
      brick.sound.beep()
  else:
    brick.light(Color.GREEN)
  return success
#!/usr/bin/env pybricks-micropython
from pybricks import ev3brick as brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import (Port, Stop, Direction, Button, Color,
                                 SoundFile, ImageFile, Align)
from pybricks.tools import print, wait, StopWatch
from pybricks.robotics import DriveBase

def runDiagnose():
  brick.display.clear()  
  success = True
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
  if success == False:
    brick.light(Color.ORANGE)
    for i in range(2):
      brick.sound.beep(500, 50, 25)
      brick.sound.beep(500, 50, 0)
      brick.sound.beep(750, 50, 25)
      brick.sound.beep(500, 50, 0)
  else:
    brick.light(Color.GREEN)

  return success
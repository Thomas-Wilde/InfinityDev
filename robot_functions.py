#!/usr/bin/env pybricks-micropython
from pybricks import ev3brick as brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import (Port, Stop, Direction, Button, Color,
                                 SoundFile, ImageFile, Align)
from pybricks.tools import print, wait, StopWatch
from pybricks.robotics import DriveBase

import time
import math

#-----------------------------------------------------------
# wir definieren globale Variablen um auf Motoren und Sensoren zuzugreifen
function_l = Motor(Port.A, Direction.COUNTERCLOCKWISE)
function_r = Motor(Port.D, Direction.COUNTERCLOCKWISE)
motor_l = Motor(Port.B, Direction.COUNTERCLOCKWISE)
motor_r = Motor(Port.C, Direction.COUNTERCLOCKWISE)
robot   = DriveBase(motor_l, motor_r, 94.2, 130.0)
col_l = ColorSensor(Port.S2)
col_l_range = [4, 34]
col_r = ColorSensor(Port.S3)
col_r_range = [2, 33]
x = 0
ACC = 50.0
MAX = 10000000.0
WHEEL_DIAMETER = 9.42  # Raddurchmesser in cm
WHEEL_CIRCUM   = WHEEL_DIAMETER * math.pi
WHEEL_DISTANCE = 12.4
TURN_CIRCUM    = WHEEL_DISTANCE * math.pi

#-----------------------------------------------------------
# #Distanz s in cm
# def drive_t(s,t,stop = Stop.COAST):
#   s = s * 10.0
#   v = s / t
#   t = t * 1000.0
#   robot.drive_time(v, 0.0, t)
#   robot.stop(stop)

#----------------------------------------------------------------------------#
def cm_to_deg(dist):
  s_per_deg = WHEEL_CIRCUM / 360.0
  deg = dist / s_per_deg
  return deg

#-----------------------------------------------------------
# acc = Beschleunigung in degree/sec
# tor = Drehmoment in % vom maximalen Drehmoment
# vmax = maximale Geschwindigkeit
def resetMotors(acc=400.0, tor=150.0, vmax=400.0):
  print("reset motors")
  motor_l.reset_angle(0.0)
  motor_r.reset_angle(0.0)
  motor_l.set_dc_settings(tor, 0.0)
  motor_r.set_dc_settings(tor, 0.0)
  motor_l.set_run_settings(vmax, acc)
  motor_r.set_run_settings(vmax, acc)
  print(("angles: ", motor_l.angle(), motor_r.angle()))

#-----------------------------------------------------------
# Fahre Weg in cm, mit gegebener Geschwindigkeit
# s - Weg in cm (negativ für rückwärts fahren)
# v - Geschwindigkeit in mm/sec
# acc - Beschleunigung in degree/(sec*sec)
# tor - Drehmoment in % vom maximimalen
def driveDistance(s, v, acc=150.0, tor=100.0):
  resetMotors(acc, tor)  
  #--- zum rückwärts Fahren brauchen wir negativen speed
  if (s < 0.0):   
    s = s * -1.0      
    v = v * -1.0
  #--- Weg in Umdrehungen (Grad) umrechnen
  deg = cm_to_deg(s)
  run = True
  robot.drive(v, 0.0)
  while run:    
    print((motor_l.angle(), motor_r.angle(), motor_l.angle() - motor_r.angle() ))
    if (abs(motor_l.angle()) >= deg):
      run = False
  stopMotors()

#-----------------------------------------------------------
def stopMotors():
  print("stop motors")
  while ((abs(motor_l.speed()) >= 1.0 or abs(motor_l.speed()) >= 1.0)):
    robot.stop(Stop.HOLD)
  print(("motor speed: ", motor_l.speed(), motor_r.speed()))

#-----------------------------------------------------------
# Ausrichten an der Bande
def alignBackward():
  motor_l.set_dc_settings(20.0, 0.0)
  motor_r.set_dc_settings(20.0, 0.0)
  robot.drive(-70.0, 0.0)
  run = True
  while run:
    if (motor_l.stalled() and motor_r.stalled()):
      run = False
  stopMotors()

#-----------------------------------------------------------
# Ausrichten an der Bande
def alignForward():
  motor_l.set_dc_settings(20.0, 0.0)
  motor_r.set_dc_settings(20.0, 0.0)
  robot.drive(70.0, 0.0)
  run = True
  while run:
    if (motor_l.stalled() and motor_r.stalled()):
      run = False
  stopMotors()


#-----------------------------------------------------------
# Roboter drehen in Grad
# deg - Drehgrad (<0.0 für Gegenuhrzeigersinn)
def turnRobot(deg, v):
  s = TURN_CIRCUM * deg / 360.0
  turn = cm_to_deg(abs(s))
  resetMotors()
  #--- zum Drehen in Gegenuhrzeigersinn brauchen wir negativen Speed
  if (deg < 0.0):
    v = v * -1.0
  #---
  motor_l.run(v)
  motor_r.run(-v)
  run = True
  while run:
    print((motor_l.angle(), motor_r.angle()))
    if (abs(motor_l.angle()) >= turn):
      run = False
  stopMotors()

#------------------------------------------------------------------------#  
def getColorLeft():
    # Werte messen
    l = col_l.reflection()
    # skalieren auf Bereich [0, 100]
    l = (l - col_l_range[0]) / (col_l_range[1] - col_l_range[0]) * 100.0
    # abschneiden auf den Bereich [0, 100]
    l = min( max(l, 1) , 100)
    return l

#------------------------------------------------------------------------#
def getColorRight():
    # Werte messen
    r = col_r.reflection()
    # skalieren auf Bereich [0, 100]
    r = (r - col_r_range[0]) / (col_r_range[1] - col_r_range[0]) * 100.0
    # abschneiden auf den Bereich [0, 100]
    r = min( max(r, 1) , 100)
    return r

#------------------------------------------------------------------------#
def getColors():    
    return (getColorLeft(), getColorRight())

#------------------------------------------------------------------------#

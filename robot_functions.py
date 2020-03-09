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

#------------------------------------------------------------------------#

# wir definieren globale Variablen um auf Motoren und Sensoren zuzugreifen                                                     #12.4 bei City Shaper
WHEEL_DISTANCE = 10.4                                     # Radstand in cm         
WHEEL_DIAMETER = 6.24                                     # Raddurchmesser in cm                                               #9.42 bei City Shaper
#function_l = Motor(Port.A, Direction.COUNTERCLOCKWISE)    # linker  Modul Motor
#function_r = Motor(Port.D, Direction.COUNTERCLOCKWISE)    # rechter Modul Motor
motor_r = Motor(Port.C, Direction.COUNTERCLOCKWISE)       # rechter Rad Motor
motor_l = Motor(Port.B, Direction.COUNTERCLOCKWISE)       # linker  Rad Motor
robot   = DriveBase(motor_l, motor_r, WHEEL_DIAMETER * 10, 8.70)        # Fahrgrundlage mit beiden Rad Motoren, Raddurchmesser und Abstand zwischen den Mittelpunkten zweier Räder
#col_l = ColorSensor(Port.S2)                              # linker  Farbsensor
#col_l_range = [8, 90]                                     # Spektrum in dem der linke Farbsensor, die Reflektion ausgibt(8 - schwarz; 90 - weiß) in Prozent
#col_r = ColorSensor(Port.S3)                              # rechter Farbsensor
#col_r_range = [6, 67]                                     # Spektrum in dem der rechte Farbsensor, die Reflektion ausgibt(6 - schwarz; 67 - weiß) in Prozent
MAX = 10000000.0                                          # Sehr hoher Wert sollten wir die maximale Leistung wollen                                                                                             
WHEEL_CIRCUM   = WHEEL_DIAMETER * math.pi                 # Radumfang in cm                            
TURN_CIRCUM    = WHEEL_DISTANCE * math.pi                 # Umfang des Wedekreis

#------------------------------------------------------------------------#

def cm_to_deg(dist):
  s_per_deg = WHEEL_CIRCUM / 360.0
  deg = dist / s_per_deg
  return deg

#------------------------------------------------------------------------#

def deg_to_cm(deg):
  s_per_deg = WHEEL_CIRCUM / 360.0
  dist = deg * s_per_deg
  return dist

#------------------------------------------------------------------------#

# acc - Beschleunigung in Grad/Sekunde
# tor - relative Begrenzung des maximal möglichen Drehmoments in Prozent
# vmax - maximale Geschwindigkeit

def resetMotors(acc=400.0, tor=150.0, vmax=400.0):    # Setzt gemessene Motorwerte 
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
# tor - Drehmoment in % vom Maximimalen
# stop - falls True dann bremmst der Roboter ab
def driveDistance(s, v, steer=0.0, acc=150.0, tor=100.0, stop=True):
  resetMotors(acc, tor)  
  #--- zum rückwärts Fahren brauchen wir negativen speed
  if (s < 0.0):   
    s = s * -1.0      
    v = v * -1.0
  #--- Weg in Umdrehungen (Grad) umrechnen
  deg = cm_to_deg(s)
  run = True
  robot.drive(v, steer)
  while run:    
    # print((motor_l.angle(), motor_r.angle(), motor_l.angle() - motor_r.angle() ))
    if (abs(motor_l.angle()) >= deg or abs(motor_r.angle()) >= deg ):
      run = False
  if stop == True:
    stopMotors()

#-----------------------------------------------------------

# def driveForward(s, v, acc=150.0, tor=100.0):
#   target      = 50.0
#   kp          = 0.88
#   kd          = 0.10
#   ki          = 0.02
#   scale       = 0.25
#   error       = 0.0
#   last_error  = 0.0
#   delta       = 0.0
#   integral    = 0.0
#   loop_count  = 0
#   pdi         = 0.0
#   change      = 0.0
#   resetMotors(acc, tor)
  
#   if (s < 0.0):   
#     s = s * -1.0      
#     v = v * -1.0
  
#   deg = cm_to_deg(s)
#   run = True
#   robot.drive(v, steer)
  
#   while run:
#     if (abs(motor_l.angle()) >= deg or abs(motor_r.angle()) >= deg ):
#       run = False
#   if stop == True:
#     stopMotors()



#   while stop_func(loop_count, pdi, change, p0):
#     loop_count += 1
#     error      = (target - sensor_func()) / 50.0
#     delta      = last_error - error 
#     last_error = error
#     integral   = integral + error
#     pdi        = (kp * error + kd * delta + ki * integral)
#     change     =  pdi * scale * speed
#     motor_l.run(speed + change)
#     motor_r.run(speed - change)


#-----------------------------------------------------------

def stopMotors():
  print("stop motors")
  while ((abs(motor_l.speed()) >= 10.0 or abs(motor_r.speed()) >= 10.0)):
    robot.stop(Stop.HOLD)
    print((motor_l.speed(), motor_r.speed()))
  print(("motor speed: ", motor_l.speed(), motor_r.speed()))

#-----------------------------------------------------------
# Ausrichten an der Bande
# v = Geschwindigkeit, positiv = vorwärts
def alignBorder(v, tor=20.0):
  motor_l.set_dc_settings(tor, 0.0)
  motor_r.set_dc_settings(tor, 0.0)
  robot.drive(v, 0.0)
  run = True
  while run:
    if (motor_l.stalled() and motor_r.stalled()):
      run = False
  stopMotors()

#-----------------------------------------------------------
def alignForward(v=70.0, tor=20.0):
  alignBorder(abs(v), tor)

def alignBackward(v=70.0, tor=20.0):
  alignBorder(-abs(v), tor)

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
  run  = True
  while run:
    #print((motor_l.angle(), motor_r.angle()))
    if (abs(motor_l.angle()) >= turn):                            #passt nicht zu realität
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
def wingLeft():
  function_l.run_angle(135,-270,Stop.HOLD,True)

#------------------------------------------------------------------------#   
def wingRight():
  function_r.run_angle(135,-270,Stop.HOLD,False)

#--------------------------------1----------------------------------------#
# Der Roboter faehrt solange gerade aus, bis er eine Linie findet.
# v        - Geschwindigkeit
# sensor   - linker oder rechter Sensor sucht die Linie ("left" oder "right")
# color    - die Farbe der Linie, die gesucht werden soll ("black" oder "white")
# steer    - wenn != 0 dann faehrt der Roboter eine Kurve
# stop     - falls "True" dann bremst der Roboter ansonsten rollt er aus
# straight - wenn "True" dann faehrt der Roboter geradeaus, ansonsten dreht er sich
def searchLine(v=100.0, sensor = "left", color = "black", steer = 0, stop=True, straight = True):
  resetMotors()
  #--- starte beide oder byr einen Motor
  if (straight):
    robot.drive(v, steer)
  else:
    if (sensor == "left"):
      motor_l.run(v)
    else:
      motor_r.run(v)
  #--- suche die Linie  
  run = True
  while run:
    val = getColorLeft() if sensor == "left" else getColorRight()  
    if (color == "black"):
      if (abs(val) <= 5):
        run = False 
    if (color == "white"):
      if (abs(val) >= 80):
        run = False    
  if (stop):
    stopMotors()
  #--- rechne die gefahrene Distanz aus
  angle = (motor_l.angle() + motor_r.angle()) / 2.0
  dist = deg_to_cm(angle)
  return dist

#--------------------------------1----------------------------------------#
# Der Roboter folgt der Linie mit dem linken oder rechten Lichtsensor.
# v           - Geschwindigkeit
# sensor_func - getColorLeft() oder getColorRight()
# stop_func   - Funktion, welche die Stop Kondition festlegt (False fuer stop)
# scale       - Skalierungsfaktor der Korrektur MUSS NEGATIV SEIN FALLS getColorLeft()
# p           - zusätzlicher Parameter für die stop Funktion
def followLine(v, stop_func, p0 = 0.0, sensor = "right"):
  target = 50.0
  kp     = 0.88
  kd     = 0.10
  ki     = 0.02
  scale  = 0.25

  sensor_func = getColorRight   
  speed      = v
  error      = 0.0
  last_error = 0.0
  delta      = 0.0
  integral   = 0.0
  loop_count = 0
  pdi        = 0.0
  change     = 0.0

  if sensor == "left":
    sensor_func = getColorLeft   
    scale = scale * -1.0
    print("left")

  while stop_func(loop_count, pdi, change, p0):
    loop_count += 1
    error      = (target - sensor_func()) / 50.0
    delta      = last_error - error 
    last_error = error
    integral   = integral + error
    pdi        = (kp * error + kd * delta + ki * integral)
    change     =  pdi * scale * speed
    motor_l.run(speed + change)
    motor_r.run(speed - change)
    # print((delta, pdi))

#------------------------------------------------------------------------#
def numberToColor(num):
  if num == Color.BLACK:
    return "Black"
  if num == Color.BLUE:
    return "Blue"
  if num == Color.GREEN:
    return "Green"
  if num == Color.YELLOW:
    return "Yellow"
  if num == Color.RED:
    return "Red"    
  if num == Color.WHITE:
    return "White"    
  if num == Color.BROWN:
    return "Brown"
  if num == None:
    return "None"

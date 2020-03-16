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
motor_r = Motor(Port.C, Direction.CLOCKWISE)       # rechter Rad Motor
motor_l = Motor(Port.B, Direction.CLOCKWISE)       # linker  Rad Motor
robot   = DriveBase(motor_l, motor_r, WHEEL_DIAMETER * 10, 87.0)        # Fahrgrundlage mit beiden Rad Motoren, Raddurchmesser und Abstand zwischen den Mittelpunkten zweier Räder
#col_l = ColorSensor(Port.S2)                              # linker  Farbsensor
#col_l_range = [8, 90]                                     # Spektrum in dem der linke Farbsensor, die Reflektion ausgibt(8 - schwarz; 90 - weiß) in Prozent
#col_r = ColorSensor(Port.S3)                              # rechter Farbsensor
#col_r_range = [6, 67]                                     # Spektrum in dem der rechte Farbsensor, die Reflektion ausgibt(6 - schwarz; 67 - weiß) in Prozent
gyro = GyroSensor(Port.S4, Direction.COUNTERCLOCKWISE)
watch = StopWatch()
MAX = 10000000.0                                          # Sehr hoher Wert sollten wir die maximale Leistung wollen                                                                                             
WHEEL_CIRCUM   = WHEEL_DIAMETER * math.pi                 # Radumfang in cm                            
TURN_CIRCUM    = WHEEL_DISTANCE * math.pi                 # Umfang des Wedekreis
cm_per_deg = WHEEL_CIRCUM / 360.0
dailyTurnFactor = 0.95
dailyDistanceFactor = 1 

#------------------------------------------------------------------------#

def cm_to_deg(dist):
  deg = dist / cm_per_deg
  return deg

def mm_to_deg(dist):
  deg = cm_to_deg(dist) / 10
  return deg

#------------------------------------------------------------------------#

def deg_to_cm(deg):
  dist = deg * cm_per_deg
  return dist

def deg_to_mm(deg):
  dist = deg_to_cm(deg) * 10
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
  deg = cm_to_deg(s) * dailyDistanceFactor
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

def gyroDrive(v):
  target = 0
  kp = 0.9
  ki= 0.0
  kd = 0.1
  error = 0
  diff = 0
  integral = 0
  prop = 0
  last_prop = 0
  scale = 5
  delta = 100
  wait(250)
  gyro.reset_angle(0)
  count = 0
  while True:
    count += 1
    if count == 60:
      angle = gyro.angle()
      gyro.reset_angle(angle-1)
    
    angle = gyro.angle()
    if angle == 1:
      continue

    prop = target - angle
    integral += prop
    diff = last_prop - prop
    error = kp * prop + ki * integral + kd * diff
    last_prop = prop
    print(str(angle) + " " + str(prop) + " " + str(integral) + " " + str(diff) + " " + str(error) + " ")

    if diff != 0:
      print((count, diff))
      count = 0


    change = error * scale
    motor_l.run(v + change)
    motor_r.run(v - change)

#-----------------------------------------------------------------

def sensorDrive(v, last_proportional, scale = 1, kp = 1, ki = 0, kd = 0):
  global value
  print(str(deg_to_mm(motor_l.speed())))
  difference = motor_l.angle() - motor_r.angle()
  proportional = 0 - difference
  change = pidCalc(proportional, last_proportional, scale, kp, ki, kd)
  motor_r.run(v - change)
  motor_l.run(v + change)
  last_proportional = proportional
  print(str(deg_to_mm(v + change)) + " " + str(deg_to_mm(v - change)) + " " + str(deg_to_mm(abs(motor_l.speed()))))
  value = last_proportional

#-----------------------------------------------------------

def pidInit():
  global proportional
  global last_proportional
  global difference
  global integral
  global derivate
  global error
  global change
  global scale
  proportional = 0
  last_proportional = 0
  difference = 0
  integral = 0
  derivate = 0
  error = 0
  change = 0
  scale = 1

def pidCalc(proportional, last_proportional, scale, kp, ki, kd):
  global integral
  integral += proportional
  derivate = last_proportional - proportional
  error = kp * proportional + ki * integral + kd * derivate
  change = error * scale
  return change

#-----------------------------------------------------------

def acceleration(v, sAcc = 15, vStart = 0,  d_k = 1, pid = True, last_proportional = 0):
  global vCurrent
  global degAcc
  t = 0
  tnull = 0.4
  tAcc = 10 * (3 * sAcc) / v                   
  kAcc = d_k * (0.2 * v) / (tAcc**2) 
  watch.resume()
  if pid == False:                     
    while (((abs(deg_to_mm(motor_l.speed()))) <= v) or ((abs(deg_to_mm(motor_r.speed()))) <= v)) and (t <= tAcc):
      t = tnull + watch.time() / 1000
      vCurrent = vStart + 5 * kAcc * (t**2)
      robot.drive(vCurrent, 0)
      print(str(vCurrent) + " " + str(10*abs(deg_to_cm(motor_l.speed()))))
  else:
    while (((abs(deg_to_mm(motor_l.speed()))) <= v) or ((abs(deg_to_mm(motor_r.speed()))) <= v)): #and (t <= tAcc):
      t = tnull + watch.time() / 1000
      vCurrent = vStart + 5 * kAcc * (t**2)
      sensorDrive(mm_to_deg(vCurrent), last_proportional)
  degAcc = abs(motor_l.angle())
  watch.pause()
  watch.reset()

def straight(v, deg, pid = True, last_proportional = 0):
  run = True
  if pid == False:
    robot.drive(v, 0)
    while True:
      print(str(v) + " " + str(deg_to_mm(motor_l.speed())))
      if (motor_l.angle() >= deg or motor_r.angle() >= deg):
        break
  else:
    while True:
      if (motor_l.angle() >= deg or motor_r.angle() >= deg):
        break
      sensorDrive(mm_to_deg(v), last_proportional)

def deceleration(v, sDec = 15, vEnd = 0, degToGo = 1000000000000, d_k = 1, pid = True, last_proportional = 0):
  vDec = v - vEnd
  tDec = 3 * (3 * sDec) / vDec
  kDec = d_k * (0.2 * v) / (tDec**2)
  watch.resume()
  t = 0
  if pid == False:
    while ((abs(deg_to_mm(motor_l.speed()))) >= vEnd or (abs(deg_to_mm(motor_r.speed())) >= vEnd)) and (t <= tDec):
      if (motor_l.angle() >= degToGo - 90 or motor_r.angle() >= degToGo - 90):
        brick.light(Color.ORANGE)
        break
      t = watch.time() / 1000
      vCurrent = v - 3 * kDec * (t**2)
      if vCurrent >= vEnd:
        robot.drive(vCurrent, 0)
        print(str(vCurrent) + " " + str(10*abs(deg_to_cm(motor_l.speed()))))
  else:
    while ((abs(deg_to_mm(motor_l.speed()))) >= vEnd or (abs(deg_to_mm(motor_r.speed())) >= vEnd)) and (t <= tDec):
      if (motor_l.angle() >= degToGo or motor_r.angle() >= degToGo):
        brick.light(Color.ORANGE)
        break
      t = watch.time() / 1000
      vCurrent = v - 3 * kDec * (t**2)
      if vCurrent >= vEnd:
        sensorDrive(mm_to_deg(vCurrent), last_proportional)
  watch.pause()
  watch.reset()

def searchDeg(vSearch, deg, pidSearch = True, last_proportional = 0):
  run = True
  if pidSearch == False:
    robot.drive(vSearch, 0)
    while True:
      print(str(vSearch) + " " + str(abs(deg_to_mm(motor_l.speed()))))
      if (abs(motor_l.angle()) >= deg - vSearch or abs(motor_r.angle()) >= deg - vSearch):
        break
  else:
    while True:
      if (abs(motor_l.angle()) >= deg or abs(motor_r.angle()) >= deg):
        break
      sensorDrive(mm_to_deg(vSearch), last_proportional)
      print(str(vSearch) + " " + str(abs(deg_to_mm(motor_l.speed()))))
  stopMotors()



def driveSmoothly(v, s, sAcc = 15, sDec = 15, vSearch = 40, buffer = 10, pid = True):
  pidInit()
  target = 0
  last_proportional = 0
  deg = cm_to_deg(s) * dailyDistanceFactor
  d_k = 1
  if deg < (cm_to_deg(sAcc) + cm_to_deg(sDec)): #Überprüfung, ob Weg ausreichend ist + Anpassung
    v = v * 0.5
    d_k = 0.75
  resetMotors(200000)

  # Beschleunigung
  brick.sound.beep(750, 50, 25)
  acceleration(v, sAcc, 0, d_k, pid, last_proportional)
  v = vCurrent
  last_proportional = value
  print(deg_to_cm(degAcc))
  print("finish")
  # Geschwindigkeit halten
  brick.sound.beep(750, 50, 25)
  degStraight = deg - degAcc - cm_to_deg(buffer)
  straight(v, degStraight, pid, last_proportional)  
  last_proportional = value
  print("finish")
  # Abbremsen
  brick.sound.beep(750, 50, 25)
  deceleration(v, sDec, vSearch, deg, d_k, pid, last_proportional)
  last_proportional = value
  print("finish")
  # Suchen
  brick.sound.beep(750, 50, 25)
  searchDeg(vSearch, deg, pid, last_proportional)
  print("finish")
  brick.sound.beep(750, 50, 25)
  print("weg in cm: " + str(deg_to_cm(abs(motor_l.angle()))))
  print(str(motor_l.angle()) + "  " + str(motor_r.angle()))

  # robot.drive(10, 0)
  # piece = 0.1
  # piece_deg = cm_to_deg(piece)
  # n = sAcc / piece_deg  
  # vCurrent = 30.0  
  # deltaV = (v-vCurrent) / ((n**4) / 4)
  # rot_old = 0.0
  # for i in range (n):
  #   vCurrent = vCurrent + (i**3) * deltaV
  #   # print(str(vCurrent))
  #   robot.drive(vCurrent, 0)
  #   rot = abs(motor_l.angle())
  #   while(rot < (i*piece_deg)):      
  #     rot = abs(motor_l.angle())
  #   d_rot = rot - rot_old
  #   rot_old = rot
  #   print("s={:.5f}   v={:.5f}".format(deg_to_cm(d_rot),vCurrent))

  # for i in range(10, 51):                               #acceleration
  #   if (v*0.0004*(i**2)) >= (10*deg_to_cm(abs(motor_l.speed()))):
  #     robot.drive(v*0.0004*(i**2), 0)  
  #   print(str(v*0.0004*(i**2)) + " " + str(10*deg_to_cm(abs(motor_l.speed()))))
  #   #wait(5)
  # s = deg_to_cm(abs(motor_l.angle()))
  # print("weg in cm: " + str(s))
  # print("----------------------------")
  # robot.drive(v, 0)
  # wait(1500)
  # print("v " + str(10*deg_to_cm(abs(motor_l.speed()))))
  # brick.sound.beep(750, 50, 25)
  # resetMotors(20)
  # vOld = 10*deg_to_cm(abs(motor_l.speed()))
  # vCurrent = v
  # for i in range(50, 9, -1):                               #deceleration
  #   robot.drive(v*0.0004*(i**2), 0)  
  #   print(str(v*0.0004*(i**2)) + " " + str(10*deg_to_cm(abs(motor_l.speed()))))
  # for i in range(101):                                   #deceleration
  #   if (10*deg_to_cm(abs(motor_l.speed())) <= vOld) and (v-v*0.0001*(i**2)+2 < vOld):
  #     robot.drive(v-v*0.0001*(i**2), 0)
  #     print("SHIFT dec calc " + str(v-v*0.0001*(i**2)) + " dec mes " + str(10*deg_to_cm(abs(motor_l.speed()))))
  #   else:
  #     print("dec calc " + str(v-v*0.0001*(i**2)) + " dec mes " + str(10*deg_to_cm(abs(motor_l.speed()))))
  #   vOld = 10*deg_to_cm(abs(motor_l.speed()))
  # while run:
  #   if vCurrent > vLin:
  #     count +=1
  #     vCurrent = v - count*k
  #     robot.drive(vCurrent, 0)
  #     print(str(vCurrent) + " | " + str(10*deg_to_cm(abs(motor_l.speed()))))
  #     wait(10)
  #   else:
  #     run = False
  # brick.sound.beep(750, 50, 25) 
  # for i in range(51):                             #Geschwindigkeit bis s errreicht halten -> optimale v ermitteln
  #   vCurrent = 100 - 0.04*i**2
  #   robot.drive(vCurrent, 0)
  #   print(str(vCurrent) + " | " + str(10*deg_to_cm(abs(motor_l.speed()))))



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
  gyro.reset_angle(0)
  s = TURN_CIRCUM * deg / 360.0
  turn = cm_to_deg(abs(s)) * dailyTurnFactor
  resetMotors()
  #--- zum Drehen in Gegenuhrzeigersinn brauchen wir negativen Speed
  if (deg < 0.0):
    v = v * -1.0
  #---
  motor_l.run(v)
  motor_r.run(-v)
  run  = True
  while run:
    print((motor_l.angle(), motor_r.angle()))
    if (abs(motor_l.angle()) >= turn):                           
      run = False
  stopMotors()
  print(gyro.angle())

#------------------------------------------------------------------------#  
def gyroTurn(deg, v):
  if deg > 180:
    while deg > 180:
      deg -= 360
  elif deg < -180:
    while deg < -180:
      deg += 360

  print(deg)

  delta = 4
  if (deg < 0.0):
    v = v * -1.0
    delta = 2.5


  gyro.reset_angle(0)
  run = True

  while True: 
    if abs(gyro.angle()) + delta >= abs(deg):
      break
    motor_l.run(v)
    motor_r.run(-v)
  stopMotors()
  angle = gyro.angle()
  print(angle)
  brick.display.text(angle , (20, 20))


#------------------------------------------------------------------------
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
#!/usr/bin/env pybricks-micropython
from robot_functions import *

#-------------------
# Aufgabe 12 - Entwerfen und Bauen
def task12():
  alignBackward()
  driveDistance(53.0, 300.0, 250.0, 200.0)
  driveDistance(53.0, -400.0, 400.0)

#------------------------------------------------------------------------#
# Aufgabe 2 - Kran
def task2():
  alignBackward()
  driveDistance(5.0, 300.0)
  turnRobot(90.0, 300.0)
  driveDistance(17.0, 300.0)
  turnRobot(-90.0, 300.0)
  alignBackward()
  driveDistance(59.0,420.0)
  driveDistance(59.0,-550.0)
  turnRobot(90.0, 400.0)
  driveDistance(-10.0,500.0)
#------------------------------------------------------------------------#

def final():
  alignBackward()
  driveDistance(18,200)
  turnRobot(90.0,200)
  driveDistance(82.0,300.0)
  wingLeft()
  driveDistance(19.0,300.0)
  turnRobot(-90.0,200.0)
  driveDistance(-27.0,200.0)
  alignBackward()
  driveDistance(50.0,300.0)
  turnRobot(-90.0,300.0)
  wingRight()
  driveDistance(10.0,100.0)
  turnRobot(45.0,200.0)
  alignForward()
  driveDistance(58.0,200.0)



#------------------------------------------------------------------------#
def followLine(s, v=150.0, acc=150.0, tor=100.0, stop=True):
  resetMotors(acc, tor)  
  #--- zum rückwärts Fahren brauchen wir negativen speed
  if (s < 0.0):   
    s = s * -1.0      
    v = v * -1.0
  #--- Weg in Umdrehungen (Grad) umrechnen
  deg = cm_to_deg(s)
  #--- Werte fuer den PID Alogrithmus
  watch = StopWatch()
  time = watch.time()
  target_value = 0.0  # Unterschied linker und rechter Farbsensor
  integral     = 0.0
  previous_error = 0.0
  derivat = 0.0

  # PID tuning
  Kp = 1.00  # proportional gain
  Ki = 0.00  # integral gain
  Kd = 0.00  # derivative gain

  run = True
  while run:
    dt   = (watch.time() - time + 1) / 1000.0
    time = watch.time()
    cols = getColors()
    # Calculate steering using PID algorithm    
    diff = (cols[0] - cols[1])
    error = target_value - diff
    integral += (error * dt)
    derivative = (error - previous_error) / dt
    # u zero:     on target,  drive forward
    # u positive: turn right
    # u negative: turn left
    u = (Kp * error) + (Ki * integral) + (Kd * derivative)    
    if abs(u) >= 1.1*v:            
      brick.sound.beep()
      if u < 0:
        u = -1.1*v  
      else:
        u =  1.1*v  
    pow_l = v - u
    pow_r = v + u
    motor_l.run(pow_l)
    motor_r.run(pow_r)
    previous_error = error
    # print((motor_l.angle(), motor_r.angle(), motor_l.angle() - motor_r.angle() ))
    if (abs(motor_l.angle()) >= deg):
      run = False
  if stop == True:
    stopMotors()


def Mission6():
  #----------------------------
  # searchWhiteLeft(100.0)     
  way = 47.0
  s1  = 20.0
  s2  = 113.0
  driveDistance(s1, 300.0)
  way += s1
  #way += searchLine(100.0, "left", "black") 
  way += searchLine(100.0, "right", "white") 
  way += searchLine(100.0, "right", "black") 
  brick.sound.beep()
  turnRobot(15.0, 200.0)
  #ueber den Satz des Pythagoras errechnen wir, wie weit wir noch fahren muessen
  dist = math.sqrt(way*way - 23.0*23.0)
  print("to Go ***********************")
  print(s2 - dist)  
  driveDistance(s2 - dist, 300.0, 0.0, 400.0)
  driveDistance(-17.0, 300.0, -270.0, 500.0)
  driveDistance(-7.5, 200.0, 0.0, 300.0)
  alignBackward()
  searchLine(100.0, "right", "white", stop=False) 
  searchLine(100.0, "right", "black", stop=False) 
  searchLine(100.0, "right", "white") 
  turnRobot(93.0, 200.0)
  #-----------------------------------------
  #--- Werkzeug abklappen
  function_l.set_dc_settings(200.0, 0.0)
  function_l.reset_angle(0.0)
  while (abs(function_l.angle()) < 90.0):
    function_l.run_until_stalled(MAX)
  driveDistance(44.0, 300.0, 0.0, 400.0)
  alignForward(70.0, 50.0)
  driveDistance(-44.0, 300.0, 0.0, 400.0)

Mission6()


#driveDistance(25.0, 200.0, MAX)

#driveDistance(200, 70)
# change some code
#final()
# driveDistance(12.0,100.0)
# turnRobot(-135.0,200.0)
# alignBackward()
# driveDistance(-58.0,300.0)
# followLine()
# #
# alignBackward()
# driveDistance(40.0, 300.0)
# followLine()
# turnRobot(7.5, 100.0)
# stopMotors()
# driveDistance(25.0, 400.0)
# alignForward()
# driveDistance(-15.0, 200.0)
# stopMotors()
# turnRobot(90.0, 100.0)
# alignForward()
# stopMotors()
# driveDistance(-44.0, 200.0)
# stopMotors()
# turnRobot(132.0, 100.0)
# stopMotors()
# driveDistance(-25.0, 200.0)
# stopMotors()
# turnRobot(-10.0, 100.0)
# stopMotors()
# driveDistance(45.0, 200.0)
# stopMotors()
# turnRobot(-17.5, 100.0)
# stopMotors()
# driveDistance(-45.0, 200.0)
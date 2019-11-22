#!/usr/bin/env pybricks-micropython
from robot_functions import *

#-------------------
# Aufgabe 12 - Entwerfen und Bauen
def Task12():
  alignBackward()
  driveDistance(50.0, 250.0, steer = - 4.5)
  driveDistance(-22.0, 500.0)
  turnRobot(60.0, 300.0)
  driveDistance(-40.0, 500.0, stop = True)
  turnRobot(-90.0, 500.0)
#------------------------------------------------------------------------#
# Aufgabe 2 - Kran
def Task2():
  # alignBackward()
  # driveDistance(5.0, 300.0)
  # turnRobot(90.0, 300.0)
  # driveDistance(17.0, 300.0)
  # turnRobot(-90.0, 300.0)
  # alignBackward()
  # driveDistance(59.0,420.0)
  # driveDistance(59.0,-550.0)
  # turnRobot(90.0, 400.0)
  # driveDistance(-10.0,500.0)
  resetMotors()
  alignBackward()
  driveDistance(10.0,200.0)
  turnRobot(90.0,150.0)
  driveDistance(16, 100.0)
  turnRobot(-90.0, 150.0)
  driveDistance(-10.0,200.0)
  alignBackward()
  driveDistance(66.0,300.0)
#------------------------------------------------------------------------#

def final():
  alignBackward()
  driveDistance(15.0,200)
  turnRobot(90.0, 200)
  driveDistance(82.0, 300.0)
  wingLeft()
  driveDistance(27.0,300.0)
  turnRobot(80.0,200.0)
  driveDistance(26.0,300.0)
  alignForward()
  driveDistance(-27.0, 200.0)
  searchLine(-200.0, "right", "white")
  turnRobot(-26.8, 150.0)
  driveDistance(-60.0, 400.0)
  robot.drive_time(-800.0, -90.0, 4000.0)
  motor_l.set_dc_settings(5.0, 0.0)
  motor_r.set_dc_settings(5.0, 0.0)
  while True:
    motor_l.run(-100)
    motor_r.run(-100)

  # driveDistance(54.0,300.0)
  # turnRobot(-65.0,300.0)
  # wingRight()
  # driveDistance(15.0,100.0)
  # turnRobot(42.5,200.0)
  # driveDistance(-15.0, 200.0)
  # turnRobot(180.0, 150.0)
  # driveDistance(-13.0, 150.0)
  # driveDistance(-58.0, 300.0, tor = 3001)
  
  # driveDistance(83.0, 300.0)
  # searchLine(100.0, "left", "black")
  # motor_r.run(100.0)
  # run = True
  # while run:
  #   val = getColorRight()
  #   if (abs(val)<=5):
  #     run = False
  # motor_r.stop(Stop.HOLD)
  # searchLine(100.0, "right", "white")
  # motor_l.run(100.0)
  # run = True
  # while run:
  #   val = getColorLeft()
  #   if (abs(val)>=80):
  #     run = False
  # motor_l.stop(Stop.HOLD)
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
  robot.drive_time(-200.0, 90.0, 2000.0)
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

#------------------------------------------------------------------------#
  
def Task6():    # Missoin: Stau
  way = 47.0
  s1  = 20.0
  s2  = 113.0
  driveDistance(s1, 300.0)
  way += s1
  way += searchLine(100.0, "right", "white") 
  way += searchLine(100.0, "right", "black") 
  turnRobot(15.0, 200.0)
  
  # ueber den Satz des Pythagoras errechnen wir, wie weit wir noch fahren muessen
  
  dist = math.sqrt(way*way - 23.0*23.0)
  print("to Go ***********************")
  print(s2 - dist)  
  driveDistance(s2 - dist, 300.0, 0.0, 400.0)
  driveDistance(-17.0, 300.0, -270.0, 500.0)
  driveDistance(-7.5, 200.0, 0.0, 300.0)
  alignBackward()
  brick.sound.beep()
  searchLine(100.0, "right", "white", stop=False) 
  searchLine(100.0, "right", "black", stop=False) 
  searchLine(100.0, "right", "white")
  Task7() 

#------------------------------------------------------------------------#

def Task7():     # Mission: Schaukel
  turnRobot(90.0, 200.0)
  driveDistance(30.0 ,300.0, acc=400.0)
  turnRobot(-90.0, 200.0)
  alignBackward(200.0)
  driveDistance(5.0, 100.0)
  
  # Werkzeug abklappen
  
  function_l.set_dc_settings(200.0, 0.0)
  function_l.reset_angle(0.0)
  while (abs(function_l.angle()) < 90.0):
    function_l.run_until_stalled(800.0)
  turnRobot(90.0 , 200.0)
  driveDistance(20.0, 300.0)
  Task8()

#------------------------------------------------------------------------#

def Task8():      # Mission: Fahrstuhl
  driveDistance(-15.0, 250.0, 0.0, 400.0)
  turnRobot(-90.0, 200.0)
  alignBackward(200.0)
  driveDistance(26.0, 200.0, acc=300.0)
  s = searchLine(100.0, "right", "black")
  driveDistance(-2.0, 100.0)
  s -= 2.0
  b = 40.0*(20.0-s)/20.0
  a = 20.0 - s
  c = math.sqrt(a*a + b*b)
  print(s)
  c += 2.0
  turnRobot(53.0, 100.0)
  driveDistance(c, 200.0)
  function_l.set_dc_settings(200.0, 0.0)
  function_l.run_until_stalled(-400.0)
  
  # Rueckweg

  driveDistance(-40.0, 500.0, acc = 400.0)
  turnRobot(37.0, 300.0)
  function_l.run_angle(400.0,90.0,Stop.HOLD)
  driveDistance(-140.0, 500.0, acc = 400.0)

#------------------------------------------------------------------------#

# searchLine(100.0, "left", "black", straight = False)
# searchLine(100.0, "right", "black", straight = False)
# searchLine(-100.0, "left", "black", straight = False)
# searchLine(100.0, "right", "black", straight = False)
# searchLine(100.0, "left", "white", straight = False)
# searchLine(100.0, "right", "white", straight = False)


Task2()
# driveDistance(20.0, 300.0)
#driveDistance(25.0, 200.0, MAX)
#final()
#driveDistance(200, 70)
# driveDistance(12.0,100.0)
# turnRobot(-135.0,200.0)
# alignBackward()
# driveDistance(-58.0,300.0)
# followLine()
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

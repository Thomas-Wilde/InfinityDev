from robot_functions import *

#------------------------------------------------------------------------#
def stop_func_stau(loop_count, pdi, change, p):
  dist = deg_to_cm(motor_l.angle())  
  print((loop_count, dist, pdi, change))
  if (dist > p):
    return False
  return True

def Task6():    # Missoin: Stau
  way = 47.0
  s1  = 20.0
  s2  = 113.0
  driveDistance(s1, 270.0)
  way += s1
  way += searchLine(120.0, "right", "white") 
  way += searchLine(120.0, "right", "black") 
  turnRobot(15.0, 260.0)
  # ueber den Satz des Pythagoras errechnen wir, wie weit wir noch fahren muessen
  dist = math.sqrt(way*way - 23.0*23.0)
  print("to Go ***********************")
  print(s2 - dist)  
  followLine(200.0, stop_func_stau, s2 - dist)
  stopMotors()
  #driveDistance(s2 - dist, 350.0, 0.0, 400.0)
  driveDistance(-17.0, 350.0, -270.0, 500.0)
  driveDistance(-7.5, 250.0, 0.0, 300.0)
  alignBackward()
  brick.sound.beep()
  Task7() 

#------------------------------------------------------------------------#
def stop_func(loop_count, pdi, change, p):
  dist = deg_to_cm(motor_l.angle())
  print((loop_count, dist, pdi, change))
  if (dist > 15.0 and pdi > 1.0 ):
    return False
  return True

def Task7():
  searchLine(100.0, "right", "white", stop=False) 
  searchLine(100.0, "right", "black", stop=False) 
  searchLine(100.0, "right", "white")
  driveDistance(1.5, 200.0)
  turnRobot(90.0, 200.0)
  followLine(150.0, stop_func)
  stopMotors()
  resetMotors()
  # Werkzeug abklappen
  function_l.set_dc_settings(200.0, 0.0)
  function_l.reset_angle(0.0)
  while (abs(function_l.angle()) < 90.0):
    function_l.run_until_stalled(800.0)
  driveDistance(20, 200.0, 19.0, stop=False)
  resetMotors()
  driveDistance(20, 200.0, -19.0, stop=False)
  driveDistance(10, 300.0)
  Task8()
#------------------------------------------------------------------------#
def Task8():      # Mission: Fahrstuhl
  driveDistance(-15.0, 250.0, 0.0, 400.0)
  turnRobot(-90.0, 180.0)
  alignBackward(200.0)
  driveDistance(26.0, 220.0, acc=300.0)
  s = searchLine(150.0, "right", "black")
  s -= 2.0
  b = 40.0*(20.0-s)/20.0
  a = 20.0 - s
  c = math.sqrt(a*a + b*b)
  print(s)
  c += 7.0
  turnRobot(53.0, 100.0)
  resetMotors()
  followLine(150.0, stop_func_fahrstuhl, p0 = c, sensor = 'left')
  stopMotors()
  function_l.set_dc_settings(200.0, 0.0)
  function_l.run_until_stalled(-400.0)
  # Rueckweg
  function_l.run_angle(400.0,90.0,Stop.HOLD)
  driveDistance(-40.0, 500.0, acc = 400.0)
  turnRobot(32.0, 300.0)
  driveDistance(-140.0, 500.0, acc = 400.0)

def stop_func_fahrstuhl(loop_count, pdi, change, p):
  dist = deg_to_cm(motor_l.angle())  
  print((loop_count, dist, pdi, change))
  if (dist > p):
    return False
  return True
#---------------------------------------------------------#
def Run01_StauSchaukel():
  Task6()

#---------------------------------------------------------#
def stop_funcKran(loop_count, pdi, change, p):
  dist = deg_to_cm(motor_l.angle())
  print((loop_count, dist, pdi, change))
  if (dist > 10.0):
    return False
  return True

# Aufgabe 2 - Kran
def Run02_Kran():
  resetMotors()
  driveDistance(30.0, 250.0)
  searchLine(color = "black")
  brick.sound.beep()
  driveDistance(2.0, 250.0)
  brick.sound.beep()
  turnRobot(-20.0, 200.0)
  brick.sound.beep()
  resetMotors()
  followLine(100.0, stop_funcKran)
  resetMotors()
  driveDistance(100.0, 800.0, acc = 300.0)
  resetMotors()
  stopMotors()
  driveDistance(-40.0, 800.0, acc = 300.0)
  turnRobot(90.0, 500.0)
  driveDistance(-60.0, 800.0, acc = 300.0)
  turnRobot(-140.0, 700.0)

#---------------------------------------------------------#
def Run03_Haus():
  alignBackward()
  driveDistance(50.0, 250.0, steer = - 4.5)
  turnRobot(-10.0, 100.0)
  driveDistance(-22.0, 800.0, acc = 300.0)
  turnRobot(60.0, 500.0)
  driveDistance(-40.0, 800.0, stop = True, acc = 300.0)
  turnRobot(-90.0, 800.0)
  function_l.set_dc_settings(50.0, 0.0)
  function_l.run_until_stalled(200.0)
  
#---------------------------------------------------------#
def Run04_Bruecke():
  function_l.set_dc_settings(100.0, 0.0)
  alignBackward()
  driveDistance(7.0,200)
  turnRobot(75.0, 160)
  searchLine(v = 150.0, sensor = 'right')
  turnRobot(15.0, 150.0)
  resetMotors()
  followLine(200.0, stop_func_bruecke)
  stopMotors()
  wingLeft()
  driveDistance(27.0,300.0)
  turnRobot(80.0,200.0)
  driveDistance(26.0,300.0)
  alignForward()
  driveDistance(-27.0, 200.0)
  searchLine(-200.0, "right", "white")
  turnRobot(-25.0, 150.0)
  driveDistance(-60.0, 400.0)
  wait(20000)

def stop_func_bruecke(loop_count, pdi, change, p):
  dist1 = deg_to_cm(motor_l.angle())
  dist2 = deg_to_cm(motor_r.angle())
  print((loop_count, dist1, dist2, pdi, change))
  if (dist1 > 48.0 or dist2 > 42.0):
    return False
  return True
#---------------------------------------------------------#
def Run05_Innovative():
  driveDistance(54.0, 300.0)
  driveDistance(-40.0, 500.0)
  turnRobot(45.0, 500.0)
  driveDistance(-60.0, 500.0)

from robot_functions import *

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

#---------------------------------------------------------#
def Run01_StauSchaukel():
  Task6()

#---------------------------------------------------------#
# Aufgabe 2 - Kran
def Run02_Kran():
  resetMotors()
  alignBackward()
  driveDistance(10.0,200.0)
  turnRobot(90.0,150.0)
  driveDistance(16, 100.0)
  turnRobot(-90.0, 150.0)
  driveDistance(-10.0,200.0)
  alignBackward()
  driveDistance(66.0,300.0)

#---------------------------------------------------------#
def Run03_Haus():
  alignBackward()
  driveDistance(50.0, 250.0, steer = - 4.5)
  driveDistance(-22.0, 500.0)
  turnRobot(60.0, 300.0)
  driveDistance(-40.0, 500.0, stop = True)
  turnRobot(-90.0, 500.0)

#---------------------------------------------------------#
def Run04_Bruecke():
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

#---------------------------------------------------------#
def Run05_Stau():
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
  turnRobot(-90.0, 200.0)
  driveDistance(130.0 ,300.0, acc=400.0)

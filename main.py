#!/usr/bin/env pybricks-micropython
from robot_functions import *

#-------------------
# Aufgabe 12 - Entwerfen und Bauen
def Task12():
  alignStart()
  driveDistance(53.0, 300.0, 250.0, 200.0)
  driveDistance(53.0, -400.0, 400.0)

#------------------------------------------------------------------------#
def followLine():
  watch = StopWatch()
  time = watch.time()

  v            = 100.0
  target_value = 0.0
  integral     = 0.0
  previous_error = 0.0

  # PID tuning
  Kp = 0.80  # proportional gain
  Ki = 0.10  # integral gain
  Kd = 0.10  # derivative gain

  while True:
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
      if u < 0:
        u = -1.1*v  
      else:
        u =  1.1*v  

    pow_l = v - u
    pow_r = v + u
    
    # pow_l = max(min(pow_l, 1.25*v), -0.25*v)
    # pow_r = max(min(pow_r, 1.25*v), -0.25*v)
    
    motor_l.run(pow_l)
    motor_r.run(pow_r)
    
    print(u)
    previous_error = error



# def followLine2():
#   speed = 360/4  # deg/sec, [-1000, 1000]
#   dt = 500       # milliseconds
#   watch = StopWatch()
  

#   # PID tuning
#   Kp = 1  # proportional gain
#   Ki = 0  # integral gain
#   Kd = 0  # derivative gain

#   integral = 0
#   previous_error = 0

#   # initial measurment
#   cols = getColors()
#   target_value = cols[0] - cols[1]

#   # Start the main loop
#   while True:
#       # Calculate steering using PID algorithm
#       cols = getColors()
#       diff = cols[0] - cols[1]
#       error = target_value - diff
#       integral += (error * dt)
#       derivative = (error - previous_error) / dt

#       # u zero:     on target,  drive forward
#       # u positive: too bright, turn right
#       # u negative: too dark,   turn left
#       u = (Kp * error) + (Ki * integral) + (Kd * derivative)

#       # limit u to safe values: [-1000, 1000] deg/sec
#       if speed + abs(u) > 1000:
#           if u >= 0:
#               u = 1000 - speed
#           else:
#               u = speed - 1000

#       # run motors
#       if u >= 0:
#           motor_l.run_time(speed-u, dt, Stop.COAST, False)
#           motor_r.run_time(speed+u, dt, Stop.COAST, False)
#           wait(dt)
#       else:
#           motor_l.run_time(speed+u, dt, Stop.COAST, False)
#           motor_r.run_time(speed-u, dt, Stop.COAST, False)
#           wait(dt)

#       previous_error = error

# # alignStart()
# # driveDistance(15.5, 200.0)
# # stopMotors()
# # brick.sound.beep()
# # turnRobot(90.0, 100.0)
# # brick.sound.beep()
# # driveDistance(87.0, 200.0)
# # stopMotors()

#driveDistance(200, 70)
followLine()

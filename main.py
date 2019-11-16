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

#driveDistance(200, 70)
followLine()

# change some code

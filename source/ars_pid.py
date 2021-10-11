#!/usr/bin/env python

import numpy as np
from numpy import *

import os


# ROS

import rospy

import tf_conversions as tf


#
import ars_lib_helpers



class PID:

  #######
  #
  prev_time_stamp_ros = rospy.Time(0.0, 0.0)
  prev_error = 0.0
  #
  error_integral = 0.0

  # Config parameters
  #
  gains = {'P': 0.0, 'I': 0.0, 'D': 0.0}
  #
  anti_windup = {'MaxI': inf, 'MinI': -inf}
  #
  ctr_cmd_sat = {'Max': inf, 'Min': -inf}
  

  def __init__(self):

    self.prev_time_stamp_ros = rospy.Time(0.0, 0.0)
    self.prev_error = 0.0

    self.error_integral = 0.0

    self.gains = {'P': 0.0, 'I': 0.0, 'D': 0.0}
    self.anti_windup = {'MaxI': inf, 'MinI': -inf}
    self.ctr_cmd_sat = {'Max': inf, 'Min': -inf}

    return


  def setGainsPID(self, gain_P=0.0, gain_I=0.0, gain_D=0.0):
    self.gains['P']=gain_P
    self.gains['I']=gain_I
    self.gains['D']=gain_D
    return

  def setAntiWindUp(self, anti_windup_min=-inf, anti_windup_max=inf):
    self.anti_windup['MaxI']=anti_windup_max
    self.anti_windup['MinI']=anti_windup_min
    return

  def setCtrCmdSaturation(self, ctr_cmd_min=-inf, ctr_cmd_max=inf):
    self.ctr_cmd_sat['Max']=ctr_cmd_max
    self.ctr_cmd_sat['Min']=ctr_cmd_min
    return

  def resetErrorIntegral():
    self.error_integral = 0.0
    return

  def reset():
    self.prev_time_stamp_ros = rospy.Time(0.0, 0.0)
    self.prev_error = 0.0
    self.error_integral = 0.0
    return

  def call(self, curr_time_stamp, error):

    #
    control_cmd = 0.0

    # Compute delta time
    delta_time = 0.0
    if(self.prev_time_stamp_ros == rospy.Time(0.0, 0.0)):
      delta_time = 0.0
    else:
      delta_time = (curr_time_stamp-self.prev_time_stamp_ros).to_sec()

    # Compute integral
    # TODO by student
    # use:
    # error
    # delta_time
    # self.error_integral

    # Anti wind-up of the error integral
    # TODO by student
    # use:
    # self.error_integral
    # self.anti_windup

    # compute error derivative
    # TODO by student
    # use
    # error_derivative
    # error, self.prev_error
    # delta_time

    # command
    # TODO by student
    # use
    # control_cmd
    # error, error_integral, error_derivative
    # self.gains

    # saturation of the control command
    # TODO by student
    # use
    # control_cmd
    # self.ctr_cmd_sat

    # Update timestamp
    self.prev_time_stamp_ros = curr_time_stamp

    # Update prev error
    self.prev_error = error

    # End
    return control_cmd


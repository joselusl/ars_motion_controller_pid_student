#!/usr/bin/env python

import numpy as np
from numpy import *

import os


# ROS
import rclpy
from rclpy.time import Time

#
import ars_lib_helpers.ars_lib_helpers as ars_lib_helpers

#
import ars_motion_controller_pid.ars_pid as ars_pid




class ArsMotionController:

  #######

  # References
  #
  flag_set_robot_pose_ref = False
  robot_posi_ref = None
  robot_atti_quat_simp_ref = None
  #
  flag_set_robot_velo_world_ref = False
  robot_velo_lin_world_ref = None
  robot_velo_ang_world_ref = None
  #
  flag_set_robot_velo_cmd_ref = False
  # m/s
  robot_velo_lin_cmd_ref = None
  # rad/s
  robot_velo_ang_cmd_ref = None


  # Feedback
  #
  flag_set_robot_pose = False
  robot_posi = None
  robot_atti_quat_simp = None
  #
  flag_set_robot_vel_world = False
  robot_velo_lin_world = None
  robot_velo_ang_world = None


  # Commands
  robot_velo_cmd_time_stamp = Time()
  robot_velo_lin_cmd = None
  robot_velo_ang_cmd = None
  

  # Loops Internal

  # Vel loop
  # Not needed!
  #
  #vel_loop_time_stamp_ros = Time()
  #vel_loop_out_lin_cmd = None
  #vel_loop_out_ang_cmd = None
  
  # Pos loop
  #
  pos_loop_time_stamp_ros = Time()
  flag_set_pos_loop_out = False
  pos_loop_out_lin_cmd = None
  pos_loop_out_ang_cmd = None


  # PIDs
  # Pose
  #
  flag_ctr_pos_hor = True
  pos_hor_pid = ars_pid.PID()
  #
  flag_ctr_pos_z = True
  pos_z_pid = ars_pid.PID()
  #
  flag_ctr_att_yaw = True
  att_yaw_pid = ars_pid.PID()

  # Velocity
  #
  flag_ctr_vel_lin_hor = True
  vel_lin_hor_pid = ars_pid.PID()
  #
  flag_ctr_vel_lin_z = True
  vel_lin_z_pid = ars_pid.PID()
  #
  flag_ctr_vel_ang_z = True
  vel_ang_z_pid = ars_pid.PID()




  #########

  def __init__(self):

    # Commands
    self.robot_velo_cmd_time_stamp = Time()
    self.robot_velo_lin_cmd = np.zeros((3,), dtype=float)
    self.robot_velo_ang_cmd = np.zeros((1,), dtype=float)

    # Feedback
    #
    self.flag_set_robot_pose = False
    self.robot_posi = np.zeros((3,), dtype=float)
    self.robot_atti_quat_simp = ars_lib_helpers.Quaternion.zerosQuatSimp()
    #
    self.flag_set_robot_vel_world = False
    self.robot_velo_lin_world = np.zeros((3,), dtype=float)
    self.robot_velo_ang_world = np.zeros((1,), dtype=float)

    # References
    #
    self.flag_set_robot_pose_ref = False
    self.robot_posi_ref = np.zeros((3,), dtype=float)
    self.robot_atti_quat_simp_ref = ars_lib_helpers.Quaternion.zerosQuatSimp()
    #
    self.flag_set_robot_velo_world_ref = False
    self.robot_velo_lin_world_ref = np.zeros((3,), dtype=float)
    self.robot_velo_ang_world_ref = np.zeros((1,), dtype=float)
    #
    self.flag_set_robot_velo_cmd_ref = False
    self.robot_velo_lin_cmd_ref = np.zeros((3,), dtype=float)
    self.robot_velo_ang_cmd_ref = np.zeros((1,), dtype=float)

    # Internal
    # Vel loop
    # Not needed!
    #self.vel_loop_time_stamp_ros = Time()
    #self.vel_loop_out_lin_cmd = np.zeros((3,1), dtype=float)
    #self.vel_loop_out_ang_cmd = np.zeros((1,1), dtype=float)
    # Pos loop
    self.pos_loop_time_stamp_ros = Time()
    self.flag_set_pos_loop_out = False
    self.pos_loop_out_lin_cmd = np.zeros((3,), dtype=float)
    self.pos_loop_out_ang_cmd = np.zeros((1,), dtype=float)

    # PIDs
    # Pos
    #
    self.pos_hor_pid = ars_pid.PID()
    #
    self.pos_z_pid = ars_pid.PID()
    #
    self.att_yaw_pid = ars_pid.PID()

    # Vel
    #
    self.vel_lin_hor_pid = ars_pid.PID()
    #
    self.vel_lin_z_pid = ars_pid.PID()
    #
    self.vel_ang_z_pid = ars_pid.PID()

    # End
    return


  def setConfigParameters(self, config_param):

    # PIDs

    # Vel
    #
    self.flag_ctr_vel_lin_hor = config_param['control_loop_vel']['control_loop_vel_lin_hor']['flag_ctr_vel_lin_hor']
    self.vel_lin_hor_pid.setGainsPID(gain_P=config_param['control_loop_vel']['control_loop_vel_lin_hor']['PID']['gains']['gain_P'],
                                      gain_I=config_param['control_loop_vel']['control_loop_vel_lin_hor']['PID']['gains']['gain_I'],
                                      gain_D=config_param['control_loop_vel']['control_loop_vel_lin_hor']['PID']['gains']['gain_D'])
    self.vel_lin_hor_pid.setAntiWindUp(config_param['control_loop_vel']['control_loop_vel_lin_hor']['PID']['anti_wind_up'][0],
                                        config_param['control_loop_vel']['control_loop_vel_lin_hor']['PID']['anti_wind_up'][1])
    self.vel_lin_hor_pid.setCtrCmdSaturation(config_param['control_loop_vel']['control_loop_vel_lin_hor']['PID']['saturation'][0],
                                              config_param['control_loop_vel']['control_loop_vel_lin_hor']['PID']['saturation'][1])
    #
    self.flag_ctr_vel_lin_z = config_param['control_loop_vel']['control_loop_vel_lin_z']['flag_ctr_vel_lin_z']
    self.vel_lin_z_pid.setGainsPID(gain_P=config_param['control_loop_vel']['control_loop_vel_lin_z']['PID']['gains']['gain_P'],
                                    gain_I=config_param['control_loop_vel']['control_loop_vel_lin_z']['PID']['gains']['gain_I'],
                                    gain_D=config_param['control_loop_vel']['control_loop_vel_lin_z']['PID']['gains']['gain_D'])
    self.vel_lin_z_pid.setAntiWindUp(config_param['control_loop_vel']['control_loop_vel_lin_z']['PID']['anti_wind_up'][0],
                                        config_param['control_loop_vel']['control_loop_vel_lin_z']['PID']['anti_wind_up'][1])
    self.vel_lin_z_pid.setCtrCmdSaturation(config_param['control_loop_vel']['control_loop_vel_lin_z']['PID']['saturation'][0],
                                              config_param['control_loop_vel']['control_loop_vel_lin_z']['PID']['saturation'][1])
    #
    self.flag_ctr_vel_ang_z = config_param['control_loop_vel']['control_loop_vel_ang_z']['flag_ctr_vel_ang_z']
    self.vel_ang_z_pid.setGainsPID(gain_P=config_param['control_loop_vel']['control_loop_vel_ang_z']['PID']['gains']['gain_P'],
                                    gain_I=config_param['control_loop_vel']['control_loop_vel_ang_z']['PID']['gains']['gain_I'],
                                    gain_D=config_param['control_loop_vel']['control_loop_vel_ang_z']['PID']['gains']['gain_D'])
    self.vel_ang_z_pid.setAntiWindUp(config_param['control_loop_vel']['control_loop_vel_ang_z']['PID']['anti_wind_up'][0],
                                        config_param['control_loop_vel']['control_loop_vel_ang_z']['PID']['anti_wind_up'][1])
    self.vel_ang_z_pid.setCtrCmdSaturation(config_param['control_loop_vel']['control_loop_vel_ang_z']['PID']['saturation'][0],
                                              config_param['control_loop_vel']['control_loop_vel_ang_z']['PID']['saturation'][1])

    # Pos
    #
    self.flag_ctr_pos_hor = config_param['control_loop_pos']['control_loop_pos_hor']['flag_ctr_pos_hor']
    self.pos_hor_pid.setGainsPID(gain_P=config_param['control_loop_pos']['control_loop_pos_hor']['PID']['gains']['gain_P'],
                                  gain_I=config_param['control_loop_pos']['control_loop_pos_hor']['PID']['gains']['gain_I'],
                                  gain_D=config_param['control_loop_pos']['control_loop_pos_hor']['PID']['gains']['gain_D'])
    self.pos_hor_pid.setAntiWindUp(config_param['control_loop_pos']['control_loop_pos_hor']['PID']['anti_wind_up'][0],
                                    config_param['control_loop_pos']['control_loop_pos_hor']['PID']['anti_wind_up'][1])
    self.pos_hor_pid.setCtrCmdSaturation(config_param['control_loop_pos']['control_loop_pos_hor']['PID']['saturation'][0],
                                          config_param['control_loop_pos']['control_loop_pos_hor']['PID']['saturation'][1])
    #
    self.flag_ctr_pos_z = config_param['control_loop_pos']['control_loop_pos_ver']['flag_ctr_pos_z']
    self.pos_z_pid.setGainsPID(gain_P=config_param['control_loop_pos']['control_loop_pos_ver']['PID']['gains']['gain_P'],
                                  gain_I=config_param['control_loop_pos']['control_loop_pos_ver']['PID']['gains']['gain_I'],
                                  gain_D=config_param['control_loop_pos']['control_loop_pos_ver']['PID']['gains']['gain_D'])
    self.pos_z_pid.setAntiWindUp(config_param['control_loop_pos']['control_loop_pos_ver']['PID']['anti_wind_up'][0],
                                    config_param['control_loop_pos']['control_loop_pos_ver']['PID']['anti_wind_up'][1])
    self.pos_z_pid.setCtrCmdSaturation(config_param['control_loop_pos']['control_loop_pos_ver']['PID']['saturation'][0],
                                        config_param['control_loop_pos']['control_loop_pos_ver']['PID']['saturation'][1])
    #
    self.flag_ctr_att_yaw = config_param['control_loop_pos']['control_loop_pos_yaw']['flag_ctr_att_yaw']
    self.att_yaw_pid.setGainsPID(gain_P=config_param['control_loop_pos']['control_loop_pos_yaw']['PID']['gains']['gain_P'],
                                  gain_I=config_param['control_loop_pos']['control_loop_pos_yaw']['PID']['gains']['gain_I'],
                                  gain_D=config_param['control_loop_pos']['control_loop_pos_yaw']['PID']['gains']['gain_D'])
    self.att_yaw_pid.setAntiWindUp(config_param['control_loop_pos']['control_loop_pos_yaw']['PID']['anti_wind_up'][0],
                                    config_param['control_loop_pos']['control_loop_pos_yaw']['PID']['anti_wind_up'][1])
    self.att_yaw_pid.setCtrCmdSaturation(config_param['control_loop_pos']['control_loop_pos_yaw']['PID']['saturation'][0],
                                          config_param['control_loop_pos']['control_loop_pos_yaw']['PID']['saturation'][1])

    

    # End
    return


  def setRobotPosRef(self, robot_posi_ref, robot_atti_quat_simp_ref):

    self.flag_set_robot_pose_ref = True

    self.robot_posi_ref = robot_posi_ref
    self.robot_atti_quat_simp_ref = robot_atti_quat_simp_ref

    return

  def setRobotVelWorldRef(self, lin_vel_world_ref, ang_vel_world_ref):

    self.flag_set_robot_velo_world_ref = True

    self.robot_velo_lin_world_ref = lin_vel_world_ref
    self.robot_velo_ang_world_ref = ang_vel_world_ref

    return

  def setRobotVelCmdRef(self, lin_vel_cmd_ref, ang_vel_cmd_ref):

    self.flag_set_robot_velo_cmd_ref = True

    self.robot_velo_lin_cmd_ref = lin_vel_cmd_ref
    self.robot_velo_ang_cmd_ref = ang_vel_cmd_ref

    return

  def setRobotPose(self, robot_posi, robot_atti_quat_simp):
    
    self.flag_set_robot_pose = True

    self.robot_posi = robot_posi
    self.robot_atti_quat_simp = robot_atti_quat_simp

    return

  def setRobotVelWorld(self, lin_vel_world, ang_vel_world):

    self.flag_set_robot_vel_world = True

    self.robot_velo_lin_world = lin_vel_world
    self.robot_velo_ang_world = ang_vel_world

    return

  def getRobotVeloCmdTimeStamp(self):
    return self.robot_velo_cmd_time_stamp

  def getRobotVeloLinCmd(self):
    return self.robot_velo_lin_cmd

  def getRobotVeloAngCmd(self):
    return self.robot_velo_ang_cmd


  def velLoopMotionController(self, time_stamp_ros):

    # Time stamp
    self.robot_velo_cmd_time_stamp = time_stamp_ros

    # Conversion (from world to robot)
    # Reference
    pos_loop_out_lin_cmd_robot = ars_lib_helpers.Conversions.convertVelLinFromWorldToRobot(self.pos_loop_out_lin_cmd, self.robot_atti_quat_simp)
    pos_loop_out_ang_cmd_robot = ars_lib_helpers.Conversions.convertVelAngFromWorldToRobot(self.pos_loop_out_ang_cmd, self.robot_atti_quat_simp)
    # Feedback
    robot_velo_lin_robot = ars_lib_helpers.Conversions.convertVelLinFromWorldToRobot(self.robot_velo_lin_world, self.robot_atti_quat_simp)
    robot_velo_ang_robot = ars_lib_helpers.Conversions.convertVelAngFromWorldToRobot(self.robot_velo_ang_world, self.robot_atti_quat_simp)

    # Initialization
    robot_velo_lin_cmd_ff = np.zeros((3,), dtype=float)
    robot_velo_lin_cmd_fb = np.zeros((3,), dtype=float)
    robot_velo_ang_cmd_ff = np.zeros((1,), dtype=float)
    robot_velo_ang_cmd_fb = np.zeros((1,), dtype=float)

    # Velocity Linear horizontal (x & y)
    # Feedforward
    # TODO by student
    # Use: self.robot_velo_lin_cmd_ref[0:2]
    robot_velo_lin_cmd_ff[0:2]
    # Feedback
    if(self.flag_ctr_vel_lin_hor and self.flag_set_pos_loop_out and self.flag_set_robot_vel_world):
      # TODO by student
      # Use: pos_loop_out_lin_cmd_robot[0:2], robot_velo_lin_robot[0:2], time_stamp_ros, self.vel_lin_hor_pid
      robot_velo_lin_cmd_fb[0:2]
    # Total
    # TODO by student
    # Use: robot_velo_lin_cmd_ff[0:2], robot_velo_lin_cmd_fb[0:2]
    self.robot_velo_lin_cmd[0:2]

    # Velocity Linear vertical (z)
    # Feedforward
    # TODO by student
    # Use self.robot_velo_lin_cmd_ref[2]
    robot_velo_lin_cmd_ff[2]
    # Feedback
    if(self.flag_ctr_vel_lin_z and self.flag_set_pos_loop_out and self.flag_set_robot_vel_world):
      # TODO by student
      # Use: pos_loop_out_lin_cmd_robot[2], robot_velo_lin_robot[2], time_stamp_ros, self.vel_lin_z_pid
      robot_velo_lin_cmd_fb[2]
    # Total
    # TODO by student
    # Use: robot_velo_lin_cmd_ff[2], robot_velo_lin_cmd_fb[2]
    self.robot_velo_lin_cmd[2]

    # Velocity Angular (z)
    # Feedforward
    robot_velo_ang_cmd_ff[0] = self.robot_velo_ang_cmd_ref[0]
    # Feedback
    if(self.flag_ctr_vel_ang_z and self.flag_set_pos_loop_out and self.flag_set_robot_vel_world):
      error_vel_ang_z = pos_loop_out_ang_cmd_robot - robot_velo_ang_robot
      robot_velo_ang_cmd_fb[0] = self.vel_ang_z_pid.call(time_stamp_ros, error_vel_ang_z)
    # Total
    self.robot_velo_ang_cmd[0] = robot_velo_ang_cmd_ff[0] + robot_velo_ang_cmd_fb[0]

    # End
    return


  def posLoopMotionController(self, time_stamp_ros):

    # Time stamp
    self.pos_loop_time_stamp_ros = time_stamp_ros

    # Initialization
    pos_loop_out_lin_cmd_ff = np.zeros((3,), dtype=float)
    pos_loop_out_lin_cmd_fb = np.zeros((3,), dtype=float)
    pos_loop_out_ang_cmd_ff = np.zeros((1,), dtype=float)
    pos_loop_out_ang_cmd_fb = np.zeros((1,), dtype=float)

    # Linear horizontal (x & y)
    # Feedforward
    # TODO by student
    # Use: self.robot_velo_lin_world_ref[0:2]
    pos_loop_out_lin_cmd_ff[0:2]
    # Feedback
    if(self.flag_ctr_pos_hor and self.flag_set_robot_pose and self.flag_set_robot_pose_ref):
      # TODO by student
      # Use: self.robot_posi_ref[0:2], self.robot_posi[0:2], time_stamp_ros, self.pos_hor_pid
      pos_loop_out_lin_cmd_fb[0:2]
    # Total
    # TODO by student
    # Use: pos_loop_out_lin_cmd_ff[0:2], pos_loop_out_lin_cmd_fb[0:2]
    self.pos_loop_out_lin_cmd[0:2]

    # Linear vertical (z)
    # Feedforward
    # TODO by student
    # Use: self.robot_velo_lin_world_ref[2]
    pos_loop_out_lin_cmd_ff[2]
    # Feedback
    if(self.flag_ctr_pos_z and self.flag_set_robot_pose and self.flag_set_robot_pose_ref):
      # TODO by student
      # Use: self.robot_posi_ref[2], self.robot_posi[2], time_stamp_ros, self.pos_z_pid
      pos_loop_out_lin_cmd_fb[2]
    # Total
    # TODO by student
    # Use: pos_loop_out_lin_cmd_ff[2], pos_loop_out_lin_cmd_fb[2]
    self.pos_loop_out_lin_cmd[2]

    # Angular (z)
    # Feedforward
    pos_loop_out_ang_cmd_ff[0] = self.robot_velo_ang_world_ref[0]
    # Feedback
    if(self.flag_ctr_att_yaw and self.flag_set_robot_pose and self.flag_set_robot_pose_ref):
      error_att_z = ars_lib_helpers.Quaternion.errorDiffFromQuatSimp(self.robot_atti_quat_simp_ref,self.robot_atti_quat_simp)
      pos_loop_out_ang_cmd_fb[0] = self.att_yaw_pid.call(time_stamp_ros, error_att_z)
    # Total
    self.pos_loop_out_ang_cmd[0] = pos_loop_out_ang_cmd_ff[0] + pos_loop_out_ang_cmd_fb[0]

    # Flag
    self.flag_set_pos_loop_out = True

    # End
    return

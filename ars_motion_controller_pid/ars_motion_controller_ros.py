#!/usr/bin/env python

import numpy as np
from numpy import *

import time

import os

# pyyaml - https://pyyaml.org/wiki/PyYAMLDocumentation
import yaml
from yaml.loader import SafeLoader


# ROS
import rclpy
from rclpy.node import Node
from rclpy.time import Time

from ament_index_python.packages import get_package_share_directory

from std_msgs.msg import Header

import geometry_msgs.msg
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped

#
import ars_lib_helpers.ars_lib_helpers as ars_lib_helpers

#
from ars_motion_controller_pid.ars_motion_controller import *




class ArsMotionControllerRos(Node):

  #######


  # Robot frame
  robot_frame = 'robot_base_link'

  # World frame
  world_frame = 'world'



  # Vel loop freq 
  # time step
  vel_loop_freq = 50.0
  # Timer
  vel_loop_timer = None


  # Pos loop freq 
  # time step
  pos_loop_freq = 10.0
  # Timer
  pos_loop_timer = None



  # Robot command publisher
  flag_pub_robot_vel_cmd_unstamped = False
  robot_vel_cmd_unstamped_pub = None
  robot_vel_cmd_stamped_pub = None

  # Robot pose subscriber
  robot_pose_sub = None
  # Robot velocity subscriber
  robot_vel_world_sub = None

  # Robot pose subscriber
  robot_pose_ref_sub = None
  # Robot velocity subscriber
  robot_vel_world_ref_sub = None
  #
  robot_vel_cmd_ref_sub = None


  #
  config_param = None


  # Motion controller
  motion_controller = ArsMotionController()
  


  #########

  def __init__(self, node_name='ars_motion_controller_node'):
    # Init ROS
    super().__init__(node_name)

    #
    self.__init(node_name)

    return


  def __init(self, node_name='ars_motion_controller_node'):
    
    # Package path
    try:
      pkg_path = get_package_share_directory('ars_motion_controller_pid')
      self.get_logger().info(f"The path to the package is: {pkg_path}")
    except ModuleNotFoundError:
      self.get_logger().info("Package not found")
    

    #### READING PARAMETERS ###
    
    # Config param
    default_config_param_yaml_file_name = os.path.join(pkg_path,'config','config_motion_controller_pid.yaml')
    # Declare the parameter with a default value
    self.declare_parameter('config_param_motion_controller_pid_yaml_file', default_config_param_yaml_file_name)
    # Get the parameter value
    config_param_yaml_file_name_str = self.get_parameter('config_param_motion_controller_pid_yaml_file').get_parameter_value().string_value
    self.get_logger().info(config_param_yaml_file_name_str)
    self.config_param_yaml_file_name = os.path.abspath(config_param_yaml_file_name_str)

    ###


    # Load config param
    with open(self.config_param_yaml_file_name,'r') as file:
        # The FullLoader parameter handles the conversion from YAML
        # scalar values to Python the dictionary format
        self.config_param = yaml.load(file, Loader=SafeLoader)['motion_controller_pid']

    if(self.config_param is None):
      self.get_logger().info("Error loading config param motion controller pid")
    else:
      self.get_logger().info("Config param motion controller pid:")
      self.get_logger().info(str(self.config_param))


    # Parameters
    #
    self.robot_frame = self.config_param['robot_frame']
    self.world_frame = self.config_param['world_frame']
    #
    self.vel_loop_freq = self.config_param['control_loop_vel']['vel_loop_freq']
    self.pos_loop_freq = self.config_param['control_loop_pos']['pos_loop_freq']
    
    #
    self.motion_controller.setConfigParameters(self.config_param)

    
    # End
    return


  def open(self):

    # Subscriber
    # 
    self.robot_pose_sub = self.create_subscription(PoseStamped, 'robot_pose', self.robotPoseCallback, qos_profile=10)
    #
    self.robot_vel_world_sub = self.create_subscription(TwistStamped, 'robot_velocity_world', self.robotVelWorldCallback, qos_profile=10)
    
    # 
    self.robot_pose_ref_sub = self.create_subscription(PoseStamped, 'robot_pose_ref', self.robotPoseRefCallback, qos_profile=10)
    #
    self.robot_vel_world_ref_sub = self.create_subscription(TwistStamped, 'robot_velocity_world_ref', self.robotVelWorldRefCallback, qos_profile=10)
    #
    self.robot_vel_cmd_ref_sub = self.create_subscription(TwistStamped, 'robot_cmd_ref', self.robotCmdRefCallback, qos_profile=10)


    # Publisher
    # Robot cmd stamped
    self.robot_vel_cmd_stamped_pub = self.create_publisher(TwistStamped, 'robot_cmd_ctr_stamped', qos_profile=10)
    # Robot cmd unstamped
    if(self.flag_pub_robot_vel_cmd_unstamped):
      self.robot_vel_cmd_unstamped_pub = self.create_publisher(Twist, 'robot_ctr_cmd', qos_profile=10)



    # Timers
    #
    self.vel_loop_timer = self.create_timer(1.0/self.vel_loop_freq, self.velLoopTimerCallback)
    #
    self.pos_loop_timer = self.create_timer(1.0/self.pos_loop_freq, self.posLoopTimerCallback)


    # End
    return


  def run(self):

    rclpy.spin(self)

    return


  def stop(self):

    # Sleep to allow time to finish
    time.sleep(0.5)

    #
    self.publishEmptyCmd(self.get_clock().now())

    #
    return


  def close(self):

    

    return


  def publishEmptyCmd(self, time_stamp=Time()):

    #
    robot_velo_cmd_stamped_msg = TwistStamped()

    robot_velo_cmd_stamped_msg.header.stamp = time_stamp.to_msg()
    robot_velo_cmd_stamped_msg.header.frame_id = self.robot_frame

    robot_velo_cmd_stamped_msg.twist.linear.x = 0.0
    robot_velo_cmd_stamped_msg.twist.linear.y = 0.0
    robot_velo_cmd_stamped_msg.twist.linear.z = 0.0

    robot_velo_cmd_stamped_msg.twist.angular.x = 0.0
    robot_velo_cmd_stamped_msg.twist.angular.y = 0.0
    robot_velo_cmd_stamped_msg.twist.angular.z = 0.0

    #
    if(self.robot_vel_cmd_stamped_pub):
      self.robot_vel_cmd_stamped_pub.publish(robot_velo_cmd_stamped_msg)
    if(self.flag_pub_robot_vel_cmd_unstamped and self.robot_vel_cmd_unstamped_pub):
      self.robot_vel_cmd_unstamped_pub.publish(robot_velo_cmd_stamped_msg.twist)

    return


  def robotPoseCallback(self, robot_pose_msg):

    # Position
    robot_posi = np.zeros((3,), dtype=float)
    robot_posi[0] = robot_pose_msg.pose.position.x
    robot_posi[1] = robot_pose_msg.pose.position.y
    robot_posi[2] = robot_pose_msg.pose.position.z

    # Attitude quat simp
    robot_atti_quat = ars_lib_helpers.Quaternion.zerosQuat()
    robot_atti_quat[0] = robot_pose_msg.pose.orientation.w
    robot_atti_quat[1] = robot_pose_msg.pose.orientation.x
    robot_atti_quat[2] = robot_pose_msg.pose.orientation.y
    robot_atti_quat[3] = robot_pose_msg.pose.orientation.z

    robot_atti_quat_simp = ars_lib_helpers.Quaternion.getSimplifiedQuatRobotAtti(robot_atti_quat)

    #
    self.motion_controller.setRobotPose(robot_posi, robot_atti_quat_simp)

    #
    return


  def robotVelWorldCallback(self, robot_vel_msg):

    # Linear
    lin_vel_world = np.zeros((3,), dtype=float)
    lin_vel_world[0] = robot_vel_msg.twist.linear.x
    lin_vel_world[1] = robot_vel_msg.twist.linear.y
    lin_vel_world[2] = robot_vel_msg.twist.linear.z

    # Angular
    ang_vel_world = np.zeros((1,), dtype=float)
    ang_vel_world[0] = robot_vel_msg.twist.angular.z

    #
    self.motion_controller.setRobotVelWorld(lin_vel_world, ang_vel_world)

    #
    return


  def robotPoseRefCallback(self, robot_pose_ref_msg):

    # Position
    robot_posi_ref = np.zeros((3,), dtype=float)
    robot_posi_ref[0] = robot_pose_ref_msg.pose.position.x
    robot_posi_ref[1] = robot_pose_ref_msg.pose.position.y
    robot_posi_ref[2] = robot_pose_ref_msg.pose.position.z

    # Attitude quat simp
    robot_atti_quat_ref = ars_lib_helpers.Quaternion.zerosQuat()
    robot_atti_quat_ref[0] = robot_pose_ref_msg.pose.orientation.w
    robot_atti_quat_ref[1] = robot_pose_ref_msg.pose.orientation.x
    robot_atti_quat_ref[2] = robot_pose_ref_msg.pose.orientation.y
    robot_atti_quat_ref[3] = robot_pose_ref_msg.pose.orientation.z

    robot_atti_quat_simp_ref = ars_lib_helpers.Quaternion.getSimplifiedQuatRobotAtti(robot_atti_quat_ref)

    #
    self.motion_controller.setRobotPosRef(robot_posi_ref, robot_atti_quat_simp_ref)

    #
    return


  def robotVelWorldRefCallback(self, robot_vel_ref_msg):

    # Linear
    lin_vel_world_ref = np.zeros((3,), dtype=float)
    lin_vel_world_ref[0] = robot_vel_ref_msg.twist.linear.x
    lin_vel_world_ref[1] = robot_vel_ref_msg.twist.linear.y
    lin_vel_world_ref[2] = robot_vel_ref_msg.twist.linear.z

    # Angular
    ang_vel_world_ref = np.zeros((1,), dtype=float)
    ang_vel_world_ref[0] = robot_vel_ref_msg.twist.angular.z

    #
    self.motion_controller.setRobotVelWorldRef(lin_vel_world_ref, ang_vel_world_ref)

    #
    return


  def robotCmdRefCallback(self, robot_cmd_ref_msg):

    # Linear
    lin_vel_cmd_ref = np.zeros((3,), dtype=float)
    lin_vel_cmd_ref[0] = robot_cmd_ref_msg.twist.linear.x
    lin_vel_cmd_ref[1] = robot_cmd_ref_msg.twist.linear.y
    lin_vel_cmd_ref[2] = robot_cmd_ref_msg.twist.linear.z

    # Angular
    ang_vel_cmd_ref = np.zeros((1,), dtype=float)
    ang_vel_cmd_ref[0] = robot_cmd_ref_msg.twist.angular.z

    #
    self.motion_controller.setRobotVelCmdRef(lin_vel_cmd_ref, ang_vel_cmd_ref)

    #
    return


  def robotCmdPub(self):

    # Get
    robot_velo_cmd_time_stamp = self.motion_controller.getRobotVeloCmdTimeStamp()
    robot_velo_lin_cmd = self.motion_controller.getRobotVeloLinCmd()
    robot_velo_ang_cmd = self.motion_controller.getRobotVeloAngCmd()

    #
    robot_velo_cmd_stamped_msg = TwistStamped()

    robot_velo_cmd_stamped_msg.header.stamp = robot_velo_cmd_time_stamp.to_msg()
    robot_velo_cmd_stamped_msg.header.frame_id = self.robot_frame

    robot_velo_cmd_stamped_msg.twist.linear.x = robot_velo_lin_cmd[0]
    robot_velo_cmd_stamped_msg.twist.linear.y = robot_velo_lin_cmd[1]
    robot_velo_cmd_stamped_msg.twist.linear.z = robot_velo_lin_cmd[2]

    robot_velo_cmd_stamped_msg.twist.angular.x = 0.0
    robot_velo_cmd_stamped_msg.twist.angular.y = 0.0
    robot_velo_cmd_stamped_msg.twist.angular.z = robot_velo_ang_cmd[0]

    #
    if(self.robot_vel_cmd_stamped_pub):
      self.robot_vel_cmd_stamped_pub.publish(robot_velo_cmd_stamped_msg)
    if(self.flag_pub_robot_vel_cmd_unstamped and self.robot_vel_cmd_unstamped_pub):
      self.robot_vel_cmd_unstamped_pub.publish(robot_velo_cmd_stamped_msg.twist)

    #
    return


  def velLoopTimerCallback(self):

    # Get time
    time_stamp_current = self.get_clock().now()

    #
    self.motion_controller.velLoopMotionController(time_stamp_current)

    # Publish
    self.robotCmdPub()
    
    # End
    return


  def posLoopTimerCallback(self):

    # Get time
    time_stamp_current = self.get_clock().now()

    #
    self.motion_controller.posLoopMotionController(time_stamp_current)

    
    # End
    return

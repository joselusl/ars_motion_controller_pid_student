#!/usr/bin/env python3

import rclpy

from ars_motion_controller_pid.ars_motion_controller_ros import ArsMotionControllerRos


def main(args=None):

  rclpy.init(args=args)

  ars_motion_controller_ros = ArsMotionControllerRos()

  ars_motion_controller_ros.open()

  try:
      ars_motion_controller_ros.run()
  except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
      # Graceful shutdown on interruption
      pass
  finally:
    ars_motion_controller_ros.destroy_node()
    rclpy.try_shutdown()

  return 0


''' MAIN '''
if __name__ == '__main__':

  main()

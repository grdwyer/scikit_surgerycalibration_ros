import rclpy
import rclpy.time
from rclpy.node import Node
import PyKDL
import tf2_ros
from std_srvs.srv import Trigger
import numpy as np


class PivotCalibrationNode(Node):
    """
    Node that exposes the pivot calibration functions from scikit-surgerycalibration package
    https://scikit-surgery.readthedocs.io/en/latest/calibration.html

    Interfaces:
      TF logger:
        Records a TF frame until told to stop then performs the pivot calibration and publishes it as a TF
        parameter tf_ee_frame_name: to get the transforms from
        parameter tf_parent_frame_name: reference frame to use
        parameter tf_calib_frame_name: frame name of calibration
        service tf_tracking: Trigger service to start recording tfs, stop triggers the calibration to be calculated and
                             published in tf
    """
    def __init__(self):
        super().__init__("pivot_calibration")
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Parameters need to be declared in ROS 2
        self.declare_parameter("tf_ee_frame_name")
        self.declare_parameter("tf_parent_frame_name")
        self.declare_parameter("tf_calibration_frame_name")

        self.tf_ee_name = self.get_parameter("tf_ee_frame_name")
        self.tf_parent_name = self.get_parameter("tf_parent_frame_name")
        self.tf_calibration_name = self.get_parameter("tf_calibration_frame_name")

        self.srv_tf_tracking = self.create_service(Trigger, "{}/tf_tracking", self.callback_tf_tracking)
        self.state_tf_tracking = False
        self.timer_tf_lookup = self.create_timer(0.1, self.callback_tf_lookup)  # don't start the timer on startup
        self.timer_tf_lookup.cancel()

        self.tf_frames = []

    def callback_tf_tracking(self, request, response):
        self.state_tf_tracking = not self.state_tf_tracking
        if self.state_tf_tracking:
            status_message = "Starting to record for pivot calibration\n"
            "\tEE frame: {}\n\tParent frame: {}\n\tCalibration frame: {}".format(
                self.tf_ee_name.get_parameter_value().string_value,
                self.tf_parent_name.get_parameter_value().string_value,
                self.tf_calibration_name.get_parameter_value().string_value)
            self.get_logger().info(status_message)
            # start tracking timer
            self.timer_tf_lookup.reset()

        else:
            status_message = "Stopping tf lookup, captured {} transforms for calibration".format(len(self.tf_frames))
            self.get_logger().info(status_message)
            self.timer_tf_lookup.cancel()
            self.calibrate()

    async def callback_tf_lookup(self):
        #  Look up required transform and add it to a list
        try:
            # Suspends callback until transform becomes available
            trans = await self.tf_buffer.lookup_transform_async(self.tf_ee_name.get_parameter_value().string_value,
                                                                self.tf_parent_name.get_parameter_value().string_value,
                                                                rclpy.time.Time())
            # This might not timeout ever which is not the best idea
            self.get_logger().debug('Got {}'.format(repr(trans)))
            self.tf_frames.append(trans)
        except tf2_ros.LookupException as e:
            self.get_logger().error('failed to get transform {}'.format(repr(e)))

    def calibrate(self):
        """
        Procedure:
          1. process transform frames into 4x4 matrices
          2. check which calibration procedure to use:
            a. default AOS at the moment
        :return:
        """
        matrices = np.zeros((len(self.tf_frames), 4, 4))
        for trans in self.tf_frames:




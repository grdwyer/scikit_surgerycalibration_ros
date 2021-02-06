import rclpy
import rclpy.time
from rclpy.node import Node
import PyKDL
import tf2_ros
from std_srvs.srv import Trigger
import numpy as np
from sksurgerycalibration.algorithms.pivot import pivot_calibration
from geometry_msgs.msg import TransformStamped


def frame_to_matrix(frame):
    """
    Convert PyKDL Frame to matrix
    :param frame: Transform to convert
    :type frame: PyKDL.Frame
    :return:
    """
    matrix = np.eye(4)
    #  translation
    matrix[0, 3] = frame.p.x()
    matrix[1, 3] = frame.p.y()
    matrix[2, 3] = frame.p.z()

    #  Rotation
    for i in range(0, 3):
        for j in range(0, 3):
            matrix[i, j] = frame.M[i, j]

    return matrix


def trans_msg_to_frame(msg):
    """
    :param msg: transform msg to convert
    :type msg:  TransformStamped
    :return:
    """
    rot = PyKDL.Rotation.Quaternion(msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z,
                                    msg.transform.rotation.w)
    pos = PyKDL.Vector(msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z)
    return PyKDL.Frame(rot, pos)


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

        self.static_transform_publisher = tf2_ros.StaticTransformBroadcaster(self)

    def callback_tf_tracking(self, request, response):
        self.state_tf_tracking = not self.state_tf_tracking
        if self.state_tf_tracking:
            status_message = "Starting to record for pivot calibration\n"
            "\tEE frame: {}\n\tParent frame: {}\n\tCalibration frame: {}".format(
                self.tf_ee_name.get_parameter_value().string_value,
                self.tf_parent_name.get_parameter_value().string_value,
                self.tf_calibration_name.get_parameter_value().string_value)
            self.get_logger().info(status_message)
            response.success = True
            response.message = status_message
            # start tracking timer
            self.timer_tf_lookup.reset()

        else:
            status_message = "Stopping tf lookup, captured {} transforms for calibration".format(len(self.tf_frames))
            response.success = True
            response.message = status_message
            self.get_logger().info(status_message)
            self.timer_tf_lookup.cancel()
            self.calibrate()

        return response

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
          3. Publish transforms
        :return:
        """
        #  Convert to matrices
        matrices = np.zeros((len(self.tf_frames), 4, 4))
        for i in range(len(self.tf_frames)):
            frame = trans_msg_to_frame(self.tf_frames[i])
            matrix = frame_to_matrix(frame)
            matrices[i, 0:4, 0:4] = matrix

        #  calibrate
        (pointer_offset, pivot_point, residual_error) = pivot_calibration(matrices)

        self.get_logger().info("Calibration determined:\n\tPointer offset: {}\n\tPivot point: {}\n\tResidual error: {}"
                               .format(pointer_offset, pivot_point, residual_error))

        # publish tool calibration
        tool_calibration = TransformStamped()
        tool_calibration.child_frame_id = self.tf_calibration_name.get_parameter_value().string_value
        tool_calibration.header.frame_id = self.tf_ee_name.get_parameter_value().string_value
        tool_calibration.transform.translation.x = pointer_offset[0]
        tool_calibration.transform.translation.y = pointer_offset[1]
        tool_calibration.transform.translation.z = pointer_offset[2]
        tool_calibration.transform.rotation.z = 1.0
        self.static_transform_publisher.sendTransform(tool_calibration)

        # publish pivot transform
        pivot_transform = TransformStamped()
        pivot_transform.child_frame_id = self.tf_calibration_name.get_parameter_value().string_value + \
                                         "_measured_pivot_point"
        pivot_transform.header.frame_id = self.tf_parent_name.get_parameter_value().string_value
        pivot_transform.transform.translation.x = pivot_point[0]
        pivot_transform.transform.translation.y = pivot_point[1]
        pivot_transform.transform.translation.z = pivot_point[2]
        pivot_transform.transform.rotation.z = 1.0
        self.static_transform_publisher.sendTransform(pivot_transform)


def main():
    rclpy.init()
    node = PivotCalibrationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

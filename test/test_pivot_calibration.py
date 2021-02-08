import unittest
import rclpy
import rclpy.time
import PyKDL
from geometry_msgs.msg import TransformStamped, Point, Vector3
import math
import random
from scikit_surgerycalibration_ros.pivot_calibration import PivotCalibrationNode, frame_to_trans_msg


def spherical_to_cart(pivot_trans, radius, x_rot, y_rot):
    """
    Returns a point on a sphere following:
      trans = pivot_trans * x_rot_trans * y_rot_trans * radius_trans
    :param pivot_trans: initial transform of the pivot point
    :type pivot_trans: PyKDL.Frame
    :param radius: radius of the sphere
    :type radius: float
    :param x_rot: angle of rotation on x axis (radians)
    :type x_rot: float
    :param y_rot: angle of rotation on y axis (radians)
    :type y_rot: float
    :return: transform on the sphere surface
    :rtype: PyKDL.Frame
    """
    rad_trans = PyKDL.Frame(PyKDL.Vector(0, 0, radius))
    x_rot_trans = PyKDL.Frame(PyKDL.Rotation.RotX(x_rot))
    y_rot_trans = PyKDL.Frame(PyKDL.Rotation.RotY(y_rot))

    return pivot_trans * x_rot_trans * y_rot_trans * rad_trans


class TestPivotCalibration(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        pass

    @classmethod
    def tearDownClass(cls):
        pass

    def test_direct_access_abs(self):
        """
        Procedure:
          1. Get random pivot point
          2. Sample points on a sphere
          3. covert to transforms
          4. Add directly (by assignement) to pivot calibation node
          5. check returned values
        :return:
        """
        x = random.uniform(-1.0, 1.0)
        y = random.uniform(-1.0, 1.0)
        z = random.uniform(-1.0, 1.0)
        original_pivot_point = PyKDL.Frame(PyKDL.Vector(x, y, z))
        sample_size = 50
        samples = []
        radius = 0.2

        for i in range(sample_size):
            x_rot = random.uniform(-math.pi/4, math.pi/4)
            y_rot = random.uniform(-math.pi/4, math.pi/4)
            frame = spherical_to_cart(original_pivot_point, radius, x_rot, y_rot)
            msg = frame_to_trans_msg(frame)
            trans = TransformStamped()
            trans.transform = msg
            samples.append(trans)

        rclpy.init()
        pivot_calibrator = PivotCalibrationNode()
        pivot_calibrator.tf_frames = samples
        (pointer_offset, pivot_point, residual_error) = pivot_calibrator.calibrate()

        orig_pp_vec = original_pivot_point.p
        self.assertAlmostEqual(orig_pp_vec.x(), pivot_point[0, 0])
        self.assertAlmostEqual(orig_pp_vec.y(), pivot_point[1, 0])
        self.assertAlmostEqual(orig_pp_vec.z(), pivot_point[2, 0])

        self.assertAlmostEqual(radius, math.fabs(pointer_offset[2, 0]))

    def test_direct_access_noise(self):
        """
        Procedure:
          1. Get random pivot point
          2. Sample points on a sphere with noise added to radius
          3. covert to transforms
          4. Add directly (by assignement) to pivot calibation node
          5. check returned values
        :return:
        """
        x = random.uniform(-1.0, 1.0)
        y = random.uniform(-1.0, 1.0)
        z = random.uniform(-1.0, 1.0)
        original_pivot_point = PyKDL.Frame(PyKDL.Vector(x, y, z))
        sample_size = 50
        samples = []
        radius = 0.2
        noise = 0.001

        for i in range(sample_size):
            x_rot = random.uniform(-math.pi/4, math.pi/4)
            y_rot = random.uniform(-math.pi/4, math.pi/4)
            noise_rad = radius + random.gauss(0, noise)
            frame = spherical_to_cart(original_pivot_point, noise_rad, x_rot, y_rot)
            msg = frame_to_trans_msg(frame)
            trans = TransformStamped()
            trans.transform = msg
            samples.append(trans)

        rclpy.init()
        pivot_calibrator = PivotCalibrationNode()
        pivot_calibrator.tf_frames = samples
        (pointer_offset, pivot_point, residual_error) = pivot_calibrator.calibrate()

        orig_pp_vec = original_pivot_point.p
        self.assertAlmostEqual(orig_pp_vec.x(), pivot_point[0, 0], delta=noise/2)
        self.assertAlmostEqual(orig_pp_vec.y(), pivot_point[1, 0], delta=noise/2)
        self.assertAlmostEqual(orig_pp_vec.z(), pivot_point[2, 0], delta=noise/2)

        self.assertAlmostEqual(radius, math.fabs(pointer_offset[2, 0]), delta=noise/2)

import unittest
import numpy as np

from recorded_data_processing import preprocess


class TestPreProcess(unittest.TestCase):

    def test_interpolate_data(self):
        encoders = [np.array([0, 0, 0]), np.array([100, 100, 1])]
        imu = [np.array([0, 0, 9.8,
                         0, 0, 9.8,
                         0, 0]),
               np.array([0, 2, 9.8,
                         1, 0, 9.8,
                         1, 1])]

        sensor_values = preprocess.interpolate_data(imu, encoders, 0.5)
        self.assertEqual(sensor_values[0], 50)
        self.assertEqual(sensor_values[1], 50)
        self.assertEqual(sensor_values[2], 0)
        self.assertEqual(sensor_values[3], 1)
        self.assertEqual(sensor_values[4], 9.8)
        self.assertEqual(sensor_values[5], 0.5)
        self.assertEqual(sensor_values[6], 0)
        self.assertEqual(sensor_values[7], 9.8)

    def test_interpolate(self):
        self.assertEqual(preprocess.interpolate(0, 1, 0, 1, 0.5), 0.5)
        self.assertEqual(preprocess.interpolate(1, 2, 0, 1, 0.5), 1.5)
        self.assertEqual(preprocess.interpolate(1, 2, 4, 8, 6), 1.5)
        self.assertEqual(preprocess.interpolate(-10, 10, 0, 5, 0), -10)
        self.assertEqual(preprocess.interpolate(-10, 10, 0, 5, 1), -6)
        self.assertEqual(preprocess.interpolate(-10, 10, 0, 5, 2), -2)
        self.assertEqual(preprocess.interpolate(-10, 10, 0, 5, 2.5), 0)
        self.assertEqual(preprocess.interpolate(-10, 10, 0, 5, 5), 10)


if __name__ == '__main__':
    unittest.main()
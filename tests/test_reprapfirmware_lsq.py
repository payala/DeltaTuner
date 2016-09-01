import unittest
import logging
import sys
from reprapfirmware_lsq import Matrix, DeltaParameters, Tuner

logger = logging.getLogger()
logger.level = logging.INFO
stream_handler = logging.StreamHandler(sys.stdout)
logger.addHandler(stream_handler)


class MatrixTest(unittest.TestCase):
    def test_ctor(self):
        stream_handler.stream = sys.stdout
        logging.debug(Matrix(3, 3).base_matrix)
        self.assertEqual(Matrix(3, 3).base_matrix, [[0, 0, 0],
                                       [0, 0, 0],
                                       [0, 0, 0]])
        self.assertEqual(Matrix(1, 3).base_matrix, [[0, 0, 0]])
        self.assertEqual(Matrix(1, 1).base_matrix, [[0]])

    def test_swap_rows(self):
        mt = Matrix(3, 3)
        bmt = [[1, 2, 3],
               [4, 5, 6],
               [7, 8, 9]]
        mt.base_matrix = list(bmt)
        mt.swap_rows(1, 2)
        self.assertEqual(mt.base_matrix, [[1, 2, 3],
                                          [7, 8, 9],
                                          [4, 5, 6]])
        mt.base_matrix = list(bmt)
        mt.swap_rows(0, 1)
        self.assertEqual(mt.base_matrix, [[4, 5, 6],
                                          [1, 2, 3],
                                          [7, 8, 9]])
        mt.base_matrix = list(bmt)
        mt.swap_rows(1, 0)
        self.assertEqual(mt.base_matrix, [[4, 5, 6],
                                          [1, 2, 3],
                                          [7, 8, 9]])
        mt.base_matrix = list(bmt)
        mt.swap_rows(2, 0)
        self.assertEqual(mt.base_matrix, [[7, 8, 9],
                                          [4, 5, 6],
                                          [1, 2, 3]])
        mt.base_matrix = list(bmt)
        mt.swap_rows(2, 2)
        self.assertEqual(mt.base_matrix, [[1, 2, 3],
                                          [4, 5, 6],
                                          [7, 8, 9]])

    def test_gauss_jordan(self):
        mt = Matrix(3,4)
        mt.base_matrix = [[2, 1, -1, 8],
                          [-3, -1, 2, -11],
                          [-2, 1, 2, -3]]
        self.assertEqual(mt.gauss_jordan(), [2, 3, -1])


class DeltaParametersTest(unittest.TestCase):
    def test_kinematics(self):
        test_points = [
            [0, 0, 0],
            [1, 0, 0],
            [0, 1, 0],
            [0, 0, 1],
            [50, 0, 0],
            [0, 50, 0],
            [0, 0, 50],
            [25, 42.3, 90.1]
        ]

        dp = DeltaParameters(100, 50, 300, 1, 2, 3, 0.5, 0.3, 0.2)

        for tp in test_points:
            ha = dp.transform(tp, 0)
            hb = dp.transform(tp, 1)
            hc = dp.transform(tp, 2)

            self.assertAlmostEqual(tp[2], dp.inverse_transform(ha, hb, hc), delta=1e-6)

    def test_normalise_endstops(self):
        test_points = [
            [1, 2, 3],
            [0, 0, 0],
            [3, 3, 3],
            [1, 5, -9]
        ]
        for tp in test_points:
            dp = DeltaParameters(100, 50, 300, tp[0], tp[1], tp[2], 0.5, 0.3, 0.2)

            dp.normalise_endstop_adjustments()

            norm_tp = [c-(sum(tp)/3) for c in tp]
            self.assertEqual([dp.xstop, dp.ystop, dp.zstop], norm_tp)


class TunerTest(unittest.TestCase):
    def test_calc(self):
        expected_probe_points = [
            [0,     94,     0.2],
            [81.41, 47,     0.5],
            [81.41, -47,    0.7],
            [0,     -94,    0.9],
            [-81.41,-47,    0.5],
            [-81.41,47,     0.1],
            [0,     0,      0.2]
        ]
        t = Tuner(270.26, 106.04, 209.6, -0.01, 0.55, -0.53, 0.6, 0.62, 0, 94, 7, 6)

        probe_points = t.get_probe_points()

        for i, ex_pt in enumerate(expected_probe_points):
            # Check that probe points are calculated correctly
            for j, coord in enumerate(ex_pt[0:2]):
                self.assertAlmostEqual(ex_pt[j], probe_points[i][j], delta=1e-2)

        for i, ex_pt in enumerate(expected_probe_points):
            # Write the expected z errors
            probe_points[i][2] = ex_pt[2]

        t.set_probe_errors(probe_points)

        cmd, dev_before, dev_after = t.calc()

        self.assertEqual("M665 R105.22 L270.26 D-0.01 E0.53 H0.00 Z209.79", cmd[0])
        self.assertEqual("M666 X-0.05 Y0.31 Z-0.25", cmd[1])

        self.assertAlmostEqual(0.52, dev_before, delta=1e-2)
        self.assertAlmostEqual(0.02, dev_after, delta=1e-2)

        self.assertAlmostEqual(-0.05, t.new_params.xstop, delta=1e-2)
        self.assertAlmostEqual(0.31, t.new_params.ystop, delta=1e-2)
        self.assertAlmostEqual(-0.25, t.new_params.zstop, delta=1e-2)
        self.assertAlmostEqual(270.26, t.new_params.diagonal, delta=1e-2)
        self.assertAlmostEqual(105.22, t.new_params.radius, delta=1e-2)
        self.assertAlmostEqual(209.79, t.new_params.homed_height, delta=1e-2)
        self.assertAlmostEqual(-0.01, t.new_params.xadj, delta=1e-2)
        self.assertAlmostEqual(0.53, t.new_params.yadj, delta=1e-2)
        self.assertAlmostEqual(0.00, t.new_params.zadj, delta=1e-2)

if __name__ == "__main__":
    unittest.main()

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

    

if __name__ == "__main__":
    unittest.main()

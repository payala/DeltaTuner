__author__ = 'Pedro'

"""
Delta printer least-squares calibration calculator

This calculator implements the least-squares delta calibration algorithm that RepRapFirmware has built-in.
I have provided it as a service for those not running RepRapFirmware.
You can choose to calibrate the following parameters:
"""

import math
import logging

deg2rad = math.pi / 180

firmware = "smoothieware"

class Calibrator(object):
    initial_points = 7
    initial_factors = 6


class Matrix(object):
    base_matrix = None

    def __init__(self, rows, cols):
        self.base_matrix = [[0]*cols]*rows

    def swap_rows(self, i, j):
        if i==j:
            return
        tmp = self.base_matrix[i]
        self.base_matrix[i] = self.base_matrix[j]
        self.base_matrix[j] = tmp

    def gauss_jordan(self):
        """
        Performs Gauss-Jordan elimination on a matrix with numRows rows and (numRows + 1) columns
        :return: solution vector
        """
        for i, row in enumerate(self.base_matrix):
            #Swap the rows around for stable Gauss-Jordan elimination
            self.base_matrix[i:] = sorted(self.base_matrix[i:], key=lambda col: col[i], reverse=True)

            #Use row i to eliminate the ith element from previous and subsequent rows
            v = self.base_matrix[i][i]
            r = tuple(enumerate(self.base_matrix))
            for j, srow in r[i+1:] + r[:i]:
                factor = srow[i]/v
                self.base_matrix[j] = [self.base_matrix[j][k] - self.base_matrix[i][k]*factor for k, val in enumerate(self.base_matrix[i])]
                self.base_matrix[j][i] = 0

        return [d[i][-1]/d[i][i] for i, d in enumerate(self.base_matrix)]

    def __unicode__(self):
        for row in self.base_matrix:
            print(row)

class DeltaParameters(object):
    def __init__(self, diagonal, radius, height, xstop, ystop, zstop, xadj, yadj, zadj):
        self.diagonal = diagonal
        self.radius = radius
        self.homed_height = height
        self.xstop = xstop
        self.ystop = ystop
        self.zstop = zstop
        self.xadj = xadj
        self.yadj = yadj
        self.zadj = zadj
        self.recalc()

    def __init__(self, delta_params):
        self.__init__(0, 0, 0, 0, 0, 0, 0, 0, 0)
        self.clone(delta_params)

    def transform(self, machine_pos, axis):
        return machine_pos[2] + math.sqrt(self.D2 - (machine_pos[0] - self.tower_x[axis])**2 - (machine_pos[1] - self.tower_y[axis])**2)

    def inverse_transform(self, Ha, Hb, Hc):
        Fa = self.coreFa + Ha**2
        Fb = self.coreFb + Hb**2
        Fc = self.coreFc + Hc**2

        #Setup PQRSU such that x = -(S - uz)/P^, y = (P - Rz)/Q
        P = (self.Xbc * Fa) + (self.Xca * Fb) + (self.Xab * Fc)
        S = (self.Ybc * Fa) + (self.Yca * Fb) + (self.Yab * Fc)

        R = 2 * ((self.Xbc * Ha) + (self.Xca * Hb) + (self.Xab * Hc))
        U = 2 * ((self.Ybc * Ha) + (self.Yca * Hb) + (self.Yab * Hc))

        R2 = R ** 2
        U2 = U ** 2

        A = U2 + R2 + self.Q2
        minusHalfB = S * U + P * R + Ha * self.Q2 + self.tower_x[0] * U * self.Q - self.tower_y[0] * R * self.Q
        C = (S + self.tower_x[0] * self.Q) ** 2 + (P - self.tower_y[0] * self.Q) ** 2 + (Ha**2 - self.D2) * self.Q2

        rslt = (minusHalfB - math.sqrt(minusHalfB**2 - A * C))/ A
        if math.isnan(rslt):
            raise Exception("At least one probe point is not reachable. Please correct your "
                            "delta radius, diagonal rod length, or probe coordniates.")
        return rslt


    def recalc(self):
        self.tower_x = []
        self.tower_y = []
        self.tower_x.append(-(self.radius * math.cos((30 + self.xadj) * deg2rad)))
        self.tower_y.append(-(self.radius * math.sin((30 + self.xadj) * deg2rad)))
        self.tower_x.append(+(self.radius * math.cos((30 - self.yadj) * deg2rad)))
        self.tower_y.append(-(self.radius * math.sin((30 - self.yadj) * deg2rad)))
        self.tower_x.append(-(self.radius * math.sin(self.zadj * deg2rad)))
        self.tower_y.append(-(self.radius * math.cos(self.zadj * deg2rad)))

        self.Xbc = self.tower_x[2] - self.tower_x[1]
        self.Xca = self.tower_x[0] - self.tower_x[2]
        self.Xab = self.tower_x[1] - self.tower_x[0]
        self.Ybc = self.tower_y[2] - self.tower_y[1]
        self.Yca = self.tower_y[0] - self.tower_y[2]
        self.Yab = self.tower_y[1] - self.tower_y[0]
        self.coreFa = self.tower_x[0] ** 2 + self.tower_y[0] ** 2
        self.coreFb = self.tower_x[1] ** 2 + self.tower_y[1] ** 2
        self.coreFc = self.tower_x[2] ** 2 + self.tower_y[2] ** 2
        self.Q = 2 * (self.Xca * self.Yab - self.Xab * self.Yca)
        self.Q2 = self.Q ** 2
        self.D2 = self.diagonal ** 2

        # Calculate the base carriage height when the printer is homed
        temp_height = self.diagonal     # any sensible height will do here, probably even zero
        self.homed_carriage_height = self.homed_height + temp_height + \
                                     self.inverse_transform(temp_height, temp_height, temp_height)

    def compute_derivative(self, deriv, ha, hb, hc):
        perturb = 0.2   # perturbation amount in mm or degrees
        hiParams = DeltaParameters(self.diagonal, self.radius, self.homed_height, self.xstop,
                                   self.ystop, self.zstop, self.xadj, self.yadj, self.zadj)
        loParams = DeltaParameters(self.diagonal, self.radius, self.homed_height, self.xstop,
                                   self.ystop, self.zstop, self.xadj, self.yadj, self.zadj)

        if deriv == 3:
            hiParams.radius += perturb
            loParams.radius -= perturb
        elif deriv == 4:
            hiParams.xadj += perturb
            loParams.xadj -= perturb
        elif deriv == 5:
            hiParams.yadj += perturb
            loParams.yadj -= perturb
        elif deriv == 6:
            hiParams.diagonal += perturb
            loParams.diagonal -= perturb

        hiParams.recalc()
        loParams.recalc()

        zLo = loParams.inverse_transform(ha - perturb if deriv == 0 else ha,
                                         hb - perturb if deriv == 1 else hb,
                                         hc - perturb if deriv == 2 else hc)
        zHi = hiParams.inverse_transform(ha + perturb if deriv == 0 else ha,
                                         hb + perturb if deriv == 1 else hb,
                                         hc + perturb if deriv == 2 else hc)

        return (zHi - zLo)/(2 * perturb)

    def normalise_endstop_adjustments(self):
        """
        Make the average of the endstop adjustments zero, or make all endstop corrections negative, without
        changing the individual homed carriage heights
        :return:
        """
        if firmware == "Marlin" or firmware == "MarlinRC" or firmware == "Repetier":
            eav = min(self.xstop, min(self.ystop, self.zstop))
        else:
            eav = (self.xstop + self.ystop + self.zstop)/3.0

        self.xstop -= eav
        self.ystop -= eav
        self.zstop -= eav
        self.homed_height += eav
        self.homed_carriage_height += eav   # No need for a full recalc, this is sufficient

    def adjust(self, num_factors, v, norm):
        """
        Perform the 3, 4, 6 or 7-factor adjustment.
        The input vector contains the following parameters in this order:
        X, Y and Z endstop adjustments
        If we are doing 4-factor adjustment, the next argument is the delta radius. Otherwise:
        X tower X position adjustment
        Y tower Y position adjustment
        Z tower Z position adjustment
        Diagonal rod length adjustment
        :return:
        """
        old_carriage_height_A = self.homed_carriage_height + self.xstop     # save for later

        #update endstop adjustments
        self.xstop += v[0]
        self.ystop += v[1]
        self.zstop += v[2]
        if norm:
            self.normalise_endstop_adjustments()

        if num_factors >= 4:
            self.radius += v[3]

            if num_factors >= 6:
                self.xadj += v[4]
                self.yadj += v[5]

                if num_factors == 7:
                    self.diagonal += v[6]

            self.recalc()

        # Adjusting the diagonal and the tower positions affects the homed carriage height
        # We need to adjust homed_height to allow for this, to get the change that was requested in
        # the endstop corrections.
        height_error = self.homed_carriage_height + self.xstop - old_carriage_height_A - v[0]
        self.homed_height -= height_error
        self.homed_carriage_height -= height_error

    def clone(self, source_deltaparams):
        self.__dict__.update(source_deltaparams.__dict__)
        self.recalc()


class Tuner(object):
    def __init__(self, old_rod_length, old_radius, old_homed_height, old_xstop, old_ystop, old_zstop,
                 old_xpos, old_ypos, old_zpos):
        self.num_factors = 6
        self.num_points = 7
        self.x_bed_probe_points = []
        self.y_bed_probe_points = []
        self.z_bed_probe_points = []
        self.probe_points_set = False
        self.normalise = True
        self.old_params = DeltaParameters(old_rod_length, old_radius, old_homed_height, old_xstop,
                                          old_ystop, old_zstop, old_xpos, old_ypos, old_zpos)
        self.new_params = DeltaParameters(self.old_params)

    def print_vector(self, v):
        raise NotImplementedError()

    def do_delta_calibration(self):
        """
        Runs the main delta calibration calculation.
        :return:
        """
        if self.num_factors not in [3, 4, 6, 7]:
            raise Exception("{} factors requested but only 3, 4, 6 and 7 are supported".format(self.num_factors))

        if self.num_factors > self.num_points:
            raise Exception("Need at least as many points as factors you want to calibrate")

        if not self.probe_points_set:
            raise Exception("Before running the delta calibration, the probe points need to be set")

        # Transform the probing points to motor endpoints and store them in a matrix, so that we can do
        # multiple iterations using the same data
        probe_motor_positions = Matrix(self.num_points, 3)
        corrections = [0] * self.num_points
        initial_sum_of_squares = 0.0
        for i in range(self.num_points):
            corrections[i] = 0.0

            xp = self.x_bed_probe_points[i]
            yp = self.y_bed_probe_points[i]

            machine_pos = [xp, yp, 0.0]

            probe_motor_positions.base_matrix[i][0] = self.old_params.transform(machine_pos, 0)
            probe_motor_positions.base_matrix[i][1] = self.old_params.transform(machine_pos, 1)
            probe_motor_positions.base_matrix[i][2] = self.old_params.transform(machine_pos, 2)

            initial_sum_of_squares += self.z_bed_probe_points[i] ** 2

        logging.debug("Motor positions:{}".format(probe_motor_positions))

        # Do 1 or more Newton-Raphson iterations
        expected_rms_error = 0
        for iteration in range(2):
            # Build a Nx7 matrix of derivatives with respect to xa, xb, yc, za, zb, zc, diagonal.
            derivative_matrix = Matrix(self.num_points, self.num_factors)
            for i in range(self.num_points):
                for j in range(self.num_factors):
                    derivative_matrix.base_matrix[i][j] = self.old_params.compute_derivative(
                        j,
                        probe_motor_positions.base_matrix[i][0],
                        probe_motor_positions.base_matrix[i][1],
                        probe_motor_positions.base_matrix[i][2]
                    )

            logging.debug("Derivative matrix: {}".format(derivative_matrix))

            # Now build the normal equations for least squares fitting
            normal_matrix = Matrix(self.num_factors, self.num_factors + 1)
            for i in range(self.num_factors):
                for j in range(self.num_factors):
                    temp = derivative_matrix.base_matrix[0][i] * derivative_matrix.base_matrix[0][j]
                    for k in range(self.num_points):
                        temp += derivative_matrix.base_matrix[k][i] * derivative_matrix[k][j]
                    normal_matrix.base_matrix[i][j] = temp
                temp = derivative_matrix[0][i] * -(self.z_bed_probe_points[0] + corrections[0])
                for k in range(self.num_points):
                    temp += derivative_matrix.base_matrix[k][i] * -(self.z_bed_probe_points[k] + corrections[k])
                normal_matrix.base_matrix[i][self.num_factors] = temp

            logging.debug("Normal matrix: {}".format(normal_matrix))

            solution = normal_matrix.gauss_jordan()

            for i in range(self.num_factors):
                if math.isnan(solution[i]):
                    raise Exception("Unable to calculate corrections. Please make sure "
                                    "that the bed probe points are all distinct")

            logging.debug("Solved matrix: {}".format(normal_matrix))

            # Calculate the residuals for debugging
            logging.debug("Solution: {}".format(solution))
            residuals = []
            for i in range(self.num_points):
                r = self.z_bed_probe_points[i]
                for j in range(self.num_factors):
                    r += solution[j] * derivative_matrix.base_matrix[i][j]
                residuals.append(r)

            logging.debug("Residuals: {}".format(residuals))

            self.new_params.clone(self.old_params)
            self.new_params.adjust(self.num_factors, solution, self.normalise)

            # Calculate the expected probe heights using the new parameters
            expected_residuals = [0] * self.num_points
            sum_of_squares = 0.0

            for i in range(self.num_points):
                for axis in range(3):
                    probe_motor_positions.base_matrix[i][axis] += solution[axis]
                new_z = self.new_params.inverse_transform(probe_motor_positions.base_matrix[i][0],
                                                          probe_motor_positions.base_matrix[i][1],
                                                          probe_motor_positions.base_matrix[i][2])
                corrections[i] = new_z
                expected_residuals[i] = self.z_bed_probe_points[i] + new_z
                sum_of_squares += expected_residuals[i] ** 2
            expected_rms_error = math.sqrt(sum_of_squares/self.num_points)
            logging.debug("Expected probe error: {}".format(expected_residuals))

            # Decide whether to do another iteration. Two is slightly better than one, but three doesn't
            # improve things. Alternatively, we could stop when the expected RMS error is only slightly
            # worse than the RMS of the residuals.
        logging.info("Calibrated {} factors using {} points, deviation before {} after {}".format(
            self.num_factors,
            self.num_points,
            math.sqrt(initial_sum_of_squares/self.num_points),
            expected_rms_error
        ))

    def setpoints(self):
        raise NotImplementedError()

    def calc_probe_points(self, num_points, radius):
        """
        Calculates the probe points based on the number of points and the specified radius
        :param num_points: number of points to probe
        :param bed_radius: radius of those points
        :return:
        """

        if num_points == 4:
            for i in range(3):
                self.x_bed_probe_points[i] = radius * math.sin((2*math.pi*i)/3)
                self.y_bed_probe_points[i] = radius * math.cos((2 * math.pi * i) / 3)
            self.x_bed_probe_points[3] = 0.0
            self.y_bed_probe_points[3] = 0.0

            self.probe_points_set = True
        else:
            if num_points >= 7:
                for i in range(6):
                    self.x_bed_probe_points[i] = radius * math.sin((2 * math.pi * i) / 6)
                    self.y_bed_probe_points[i] = radius * math.cos((2 * math.pi * i) / 6)
            if num_points >= 10:
                for i in range(6,9):
                    self.x_bed_probe_points[i] = radius / 2 * math.sin((2 * math.pi * i) / 6)
                    self.y_bed_probe_points[i] = radius / 2 * math.cos((2 * math.pi * i) / 6)
                self.x_bed_probe_points[9] = 0.0
                self.y_bed_probe_points[9] = 0.0
                self.probe_points_set = True
            else:
                self.x_bed_probe_points[6] = 0.0
                self.y_bed_probe_points[6] = 0.0
                self.probe_points_set = True


    def get_parameters(self):
        raise NotImplementedError()

    def convert_incoming_endstops(self):
        raise NotImplementedError()

    def convert_outgoing_endstops(self):
        raise NotImplementedError()

    def set_new_parameters(self):
        raise NotImplementedError()

    def generate_commands(self):
        raise NotImplementedError()

    def calc(self):
        raise NotImplementedError()

    def copy_to_initial(self):
        self.old_params.clone(self.new_params)



if __name__ == "__main__":
    m = Matrix(3, 4)
    m.base_matrix = [[1, 0, 0, 2],[2,0,3,4],[3,3,3,5]]
    m.gauss_jordan(3)
    pass

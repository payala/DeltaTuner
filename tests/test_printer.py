import unittest
import serial
from printer import Printer
import time


class PrinterTest(unittest.TestCase):
    def setUp(self):
        self.real_serial = serial.Serial
        self.set_serial_stub()

    def set_serial_real(self):
        """ Sets Serial.serial to the real serial port """
        serial.Serial = self.real_serial

    def set_serial_stub(self):
        """ Sets all calls to Serial.serial to the serial mock"""
        serial.Serial = SerialStub

    def test_printer_setup(self):
        p = Printer("COMX")

        self.assertEqual("COMX", p.ser.port)
        self.assertEqual(115200, p.ser.baudrate)
        self.assertEqual(serial.PARITY_NONE, p.ser.parity)
        self.assertEqual(serial.STOPBITS_ONE, p.ser.stopbits)
        self.assertEqual(serial.EIGHTBITS, p.ser.bytesize)
        self.assertEqual(30, p.ser.timeout)
        self.assertEqual(False, p.ser.is_open)

    def test_send_command(self):
        p = Printer("COMX")

        tst_cmd = b"TEST COMMAND FTW"

        p.ser.stub_add_received_bytes(b"ok\n")
        p.send_command(tst_cmd)

        self.assertEqual(tst_cmd + b'\n', p.ser.stub_get_sent_bytes())

    def test_send_command_timeout(self):

        timeout = 2

        p = Printer("COMX")
        p.timeout = timeout*1000
        p.ser.timeout = timeout

        tst_cmd = b"TEST COMMAND FTW"

        ti = time.time()

        with self.assertRaises(TimeoutError):
            p.send_command(tst_cmd) # No 'ok' response was written

        self.assertTrue(timeout - 1 < time.time() - ti < timeout + 1, msg="t elapsed: {} timeout: {}".format(
            time.time() - ti,
            timeout
        ))

        self.assertEqual(tst_cmd + b'\n', p.ser.stub_get_sent_bytes())

    def test_move(self):
        p = Printer("COMX")

        p.ser.stub_add_received_bytes(b"ok\n")
        p.move(x=1, y=2, z=3, e=4, f=20)

        self.assertEqual(1, p.x)
        self.assertEqual(2, p.y)
        self.assertEqual(3, p.z)
        self.assertEqual(4, p.e)
        self.assertEqual(20, p.feedrate)

        self.assertEqual(b"G1 X1.00 Y2.00 Z3.00 E4.00 F20.00 \n", p.ser.stub_get_sent_bytes())

    def test_home(self):
        p = Printer("COMX")

        p.ser.stub_add_received_bytes(b"ok\n")
        p.home()

        self.assertEqual(b"G28\n", p.ser.stub_get_sent_bytes())

    def test_get_current_position(self):
        p = Printer("COMX")

        p.ser.stub_add_received_bytes(b"ok C: X:-10.00 Y:-20.00 Z:-30.00 E:-40.00\r\n")
        p.get_current_position()

        self.assertEqual(-10, p.x)
        self.assertEqual(-20, p.y)
        self.assertEqual(-30, p.z)
        self.assertEqual(-40, p.e)

    def test_printer_on(self):
        p = Printer("COMX")

        p.ser.stub_add_received_bytes(b"ok\n")
        p.printer_on()

        self.assertEqual(b"M80\n", p.ser.stub_get_sent_bytes())

    def test_printer_off(self):
        p = Printer("COMX")

        p.ser.stub_add_received_bytes(b"ok\n")
        p.printer_off()

        self.assertEqual(b"M81\n", p.ser.stub_get_sent_bytes())

    def test_printer_update_geometry(self):
        #self.set_serial_real()
        p = Printer("COM3")

        arm_length = 260.69 # L
        arm_radius = 104.14 # R
        tower_offsets = [0.001, 0.002, 0.003]  # [A, B, C]
        tower_angles = [0.01, 0.02, 0.03]  # [D, E, H]
        z_height = 203.45  # Max Z

        endstop_adjustments = [-0.120, 0.700, -0.580]  # [X, Y, Z]


        # Set response to M665
        p.ser.stub_add_received_bytes('A:   {} B:   {} C:   {} D:  {} E:   {} '
                                      'H:   {} L: {} R: {} Max Z  {} \r\nok\r\n'.format(
                                            *tower_offsets, *tower_angles, arm_length,
                                            arm_radius, z_height
                                      ).encode())
        # Set response to M666
        p.ser.stub_add_received_bytes('X: {} Y: {} Z: {}\nok\r\n'.format(
            *endstop_adjustments).encode())

        p.update_printer_geometry()

        self.assertAlmostEqual(arm_length, p.geom_rod_length)
        self.assertAlmostEqual(arm_radius, p.geom_radius)
        self.assertAlmostEqual(z_height, p.geom_homed_height)
        for i, v in enumerate(tower_angles):
            self.assertAlmostEqual(tower_angles[i], p.geom_angular_pos_correction[i])
            self.assertAlmostEqual(endstop_adjustments[i], p.geom_endstop_corrections[i])




class SerialStub(object):
    def __init__(self, port=None, baudrate=None, parity=None, stopbits=None, bytesize=None, timeout=None):
        self.port = port
        self.baudrate = baudrate
        self.parity = parity
        self.stopbits = stopbits
        self.bytesize = bytesize
        self.timeout = timeout
        self.is_open = False

        self.bytes_received = b""
        self.bytes_sent = b""

        self.rx_flushed = False

    def open(self):
        self.is_open = True

    def close(self):
        self.is_open = False

    def isOpen(self):
        return self.is_open

    def readall(self):
        ti = time.time()
        bytes_read = self.bytes_received
        self.bytes_received = b""

        if bytes_read == b"":
            while time.time() < ti + self.timeout:
                pass
            raise TimeoutError

        return bytes_read

    def readline(self):
        ti = time.time()
        bytes_read = self.bytes_received.decode()
        if '\n' in bytes_read:
            self.bytes_received = '\n'.join(bytes_read.split('\n')[1:]).encode()
            bytes_read = bytes_read.split('\n')[0] + '\n'
        bytes_read = bytes_read.encode()

        if b"\n" not in bytes_read:
            while time.time() < ti + self.timeout:
                pass
            raise TimeoutError

        return bytes_read

    def write(self, data):
        self.bytes_sent += data

    def flushInput(self):
        self.rx_flushed = True      # Don't delete bytes_received, so a response can be pre-written

    def stub_add_received_bytes(self, data):
        self.rx_flushed = False
        self.bytes_received += data

    def stub_get_sent_bytes(self):
        return self.bytes_sent

    def stub_clear_sent_bytes(self):
        self.bytes_sent = b""


if __name__ == "__main__":
    unittest.main()
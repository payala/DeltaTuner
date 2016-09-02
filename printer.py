import serial
import datetime


class Printer(object):
    """
    This is the base class to use as an interface to a 3D printer. It contains basic command and serial port
    handling that is common to all printers.
    """
    def __init__(self, com_port):
        ################################################
        # Communications
        ################################################
        self.is_open = False
        self.timeout = 30000
        self.ser = serial.Serial(
            port=com_port,
            baudrate=115200,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=self.timeout/1000
        )

        ###############################################
        # Printer parameters
        ###############################################

        self.x = None  # position of x axis
        self.y = None  # position of y axis
        self.z = None  # position of z axis
        self.e = None  # position of extruder
        self.feedrate = 10  # feedrate (mm/s)

    def connect(self):
        self.ser.close()
        self.ser.open()
        if self.ser.isOpen():
            self.is_open = True

    def disconnect(self):
        self.ser.close()

    def get_available_ports(self):
        """ Lists serial port names

            :raises EnvironmentError:
                On unsupported or unknown platforms
            :returns:
                A list of the serial ports available on the system
        """
        ports = [list(port) + ['Open'] for port in list(serial.tools.list_ports.comports())]

        for port in ports:
            try:
                s = serial.Serial(port[0])
                s.close()
                port[-1] = 'Closed'
            except (OSError, serial.SerialException):
                pass
        return ports

    def send_command(self, command):
        """
        Sends 'command' and waits for a returning 'ok'.
        :param command: Command to send
        :return: Response to the command, including final 'ok'
        """
        response = b''
        self.ser.flushInput()
        self.ser.write(command + b"\n")
        t = self.now_ms()
        while True:
            try:
                response += self.ser.readline()
            except TimeoutError:
                t = 0  # Force timeout later.
            if b"ok" in response:
                return response
            if self.now_ms() - t > self.timeout:
                raise TimeoutError("Did not receive response for commannd {}".format(command))

    def time_ms(self, date_time):
        return (date_time.day * 24 * 3600 + date_time.second) * 1000 + date_time.microsecond / 1000.0

    def now_ms(self):
        return self.time_ms(datetime.datetime.now())

    def is_numeric(self, possible_number):
        try:
            float(possible_number)
        except ValueError:
            return False
        else:
            return True

    def tag_if_numeric(self, tag, value):
        if self.is_numeric(value):
            return bytes("{}{:.2f} ".format(tag, float(value)), 'utf-8')
        else:
            return b""

    def move(self, x=None, y=None, z=None, e=None, f=None):
        """
        Moves the machine to the specified position with the specified feedrate (mm/s).
        If some coordinate is ommited it will remain unchanged.
        :param x: destination x coordinate
        :param y: destination y coordinate
        :param z: destination z coordinate
        :param e: destination extruder position
        :param f: feedrate (mm/s) to use in the movement
        :return: nothing
        """
        cmd = b"G1 "

        self.x = x if self.is_numeric(x) else self.x
        self.y = y if self.is_numeric(y) else self.y
        self.z = z if self.is_numeric(z) else self.z
        self.e = e if self.is_numeric(e) else self.e
        self.feedrate = f if self.is_numeric(f) else self.feedrate

        cmd += self.tag_if_numeric("X", x)
        cmd += self.tag_if_numeric("Y", y)
        cmd += self.tag_if_numeric("Z", z)
        cmd += self.tag_if_numeric("E", e)
        cmd += self.tag_if_numeric("F", f)

        self.send_command(cmd)

    def home(self):
        """
        Homes all axis
        :return:
        """

        self.send_command(b"G28")

    def get_current_position(self):
        resp = self.send_command(b"M114").decode()

        r = resp.split(' ')
        for sect in r:
            if "X" in sect:
                self.x = float(sect[2:])
            if "Y" in sect:
                self.y = float(sect[2:])
            if "Z" in sect:
                self.z = float(sect[2:])
            if "E" in sect:
                self.e = float(sect[2:])

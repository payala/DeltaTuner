import serial
import datetime
import re


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


        ###############################################
        # Printer geometry parameters
        ###############################################

        self.geom_rod_length = None
        self.geom_radius = None
        self.geom_homed_height = None
        # Endstop corrections are offsets applied to each axis' endstops
        self.geom_endstop_corrections = [None, None, None]

        # Tower angular position corrections are the angle deviation to
        # compensate for each tower
        self.geom_angular_pos_correction = [None, None, None]

        self.geom_labels = {
            'geom_rod_length': 'L',
            'geom_radius': 'R',
            'geom_homed_height': 'Max Z',
            'geom_endstop_corrections': ['X', 'Y', 'Z'],
            'geom_angular_pos_correction': ['D', 'E', 'H'],
        }
        self.re_param = None

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
        if isinstance(command, str):
            command = command.encode()
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

    def printer_on(self):
        self.send_command(b"M80")

    def printer_off(self):
        self.send_command(b"M81")

    def parse_options(self, option_string):
        """
        Parses options obtained from printer responses such as M665, M666
        :param option_string: raw string as received from the printer
        :return a dict with options as keys and option values as dict values
        """

        # Regex is: there are two types of groups:
        # 1.- Values (  [\d\.-]+  )
        # 2.- Labels (  [\w\s]*   )
        # Capture pairs of Label, Value separated by optional ':' and space
        # Labels are groups that are between two values. To match with the
        # first label, the first value is optional, and to avoid capturing,
        # it is non-capturing '(?:'
        #
        if isinstance(option_string, bytes):
            option_string = option_string.decode('ascii')

        param = r"(?:[\d\.-]+)?([\w\s]*):?\s+([\d\.-]+)\s*"

        self.re_param = re.compile(param)

        ret_dict = {}

        for match in self.re_param.findall(option_string):
            ret_dict[match[0].strip()] = float(match[1])

        return ret_dict

    def update_printer_geometry(self):
        """ Fetches printer geometry via M666 and M665 and populates geometry params """
        m665 = self.send_command(b"M665")
        params = self.parse_options(m665)

        m666 = self.send_command(b"M666")
        params.update(self.parse_options(m666))

        for attr, labels in self.geom_labels.items():
            if isinstance(labels, list):
                # vector label
                if all([l in params.keys() for l in labels]):
                    setattr(self, attr, [params[l] for l in labels])
            else:
                # scalar label
                if labels in params.keys():
                    setattr(self, attr, params[labels])

    def for_tuner(self, probe_radius):
        return (self.geom_rod_length, self.geom_radius, self.geom_homed_height,
                *self.geom_endstop_corrections, *self.geom_angular_pos_correction,
                probe_radius)

    def probe_point(self, x, y, z=1):
        self.send_command(b"G1 X0 Y0 Z100 F30000")
        self.send_command(b"G28")
        self.send_command("G1 X{} Y{} Z{} F15000".format(x, y, z).encode())

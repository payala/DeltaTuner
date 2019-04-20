import click
import os
import json
import msvcrt
import pprint

from reprapfirmware_lsq import Tuner
from printer import Printer
from colorama import Fore, Back, Style

json_filename = 'cli_tuner.json'
probe_choices = [b"2", b"5", b"8"]

num_probes = 3      # Number of times each point is probed
feeler_gage_thickness = 0.1  # Thickness of the feeler gage in mm

@click.command()
@click.option('--radius', help='Probing points will be in a circle of this radius.')
@click.option('--com_port', help='COM port to which the printer is connected')
def main(radius, com_port):

    # Check for existing options
    try:
        fp = open(json_filename, 'r')
        opts = json.load(fp)
        fp.close()
    except Exception:
        opts = {}

    radius = opts['radius'] = radius or opts.get('radius')
    com_port = opts['com_port'] = com_port or opts.get('com_port')

    if radius is None:
        radius = opts['radius'] = click.prompt(
            "Enter the radius of the probe points:",
            type=float
        )
    if com_port is None:
        com_port = opts['com_port'] = click.prompt(
            "Enter the COM port to connect to the printer"
        )

    # Save options for future calls
    fp = open(json_filename, 'w')
    json.dump(opts, fp)
    fp.close()

    # Start calibration process
    choice = click.confirm("Do you want to start the calibration process with a radius of {}mm?".format(radius))
    if not choice:
        return

    printer = Printer(com_port)
    printer.connect()
    printer.send_command(b"M80")     # Power supply on
    printer.send_command(b"G28")     # home
    while True:
        printer.update_printer_geometry()
        tuner = Tuner(*printer.for_tuner(radius))

        probe_pts = tuner.get_probe_points()

        # Obtain Z height errors
        for i, point in enumerate(probe_pts):
            probe_num = 0

            z_err = 0

            last_z_err = 0
            last_operation = "start"

            initial_z = 1

            click.echo("\n2 to lower, 5 to finish, 8 to erase previous")

            while probe_num < num_probes:
                if last_operation in ["start", "set_z_err"]:
                    # Go to the probe point
                    if last_operation == "start":
                        z = initial_z
                    else:
                        z = -last_z_err * num_probes + feeler_gage_thickness + 0.2
                    printer.probe_point(point[0], point[1], z=z)
                    last_operation = "go to probe"

                pch = ""
                while pch not in probe_choices:
                    pch = msvcrt.getch()
                if pch == b"2":
                    printer.send_command(b"G91")  # Set relative mode
                    printer.send_command(b"G1 Z-0.05")
                    printer.send_command(b"G90")  # set absolute mode
                elif pch == b"5":
                    last_operation = "set_z_err"
                    printer.get_current_position()

                    click.echo("z_err: {}".format(printer.z - feeler_gage_thickness))
                    last_z_err = -(printer.z - feeler_gage_thickness) / num_probes
                    z_err += last_z_err
                    probe_num += 1
                elif pch == b"8":
                    if last_operation != "undo":
                        z_err -= last_z_err
                        last_z_err = 0
                        probe_num -= 1
                    last_operation = "undo"

            click.echo("Average z_err for point: {}".format(z_err))

            probe_pts[i][2] = z_err

        click.echo("\nZ errors:")
        pp = pprint.PrettyPrinter(indent=4)
        pp.pprint(probe_pts)

        tuner.set_probe_errors(probe_pts)

        cmd, dev_before, dev_after = tuner.calc()

        click.echo("Commands to run for correction:\n{}\n{}".format(*cmd))

        choice = click.confirm("Deviation: {} (before: {}), do you want "
                               "to run another calibration round?".format(dev_after, dev_before))
        if not choice:
            save_choice = click.confirm("Do you want to save the result of this calibration to Flash?")
            if save_choice:
                click.echo("Saving to Flash with M500 command")
                printer.send_command(b"M500")

            click.echo("Exiting")
            printer.send_command("G28")
            printer.send_command("M81")  # Turn printer off
            return

        # Apply configuration
        printer.send_command(cmd[0].encode())
        printer.send_command(cmd[1].encode())




# def calibration_step()
#


if __name__ == "__main__":
    main()
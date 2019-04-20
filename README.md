# DeltaTuner
Tune & Calibrate your delta printer

This is the repository for the DeltaTuner software. It is based on the least squares delta calibration
implementation of RepRapFirmware. The same algorithm can be found in [Escher3D's website] for those that don't have
RepRapFirmware.

The main contribution of this software is automatic g-code generation from the PC to move the printer 
to the test points and automatically capture the height errors at each point. All the code is sent via
USB to the printer directly, so the tuning can be much faster and less error-prone.

At the moment, this code is non-functional, the first bits are being created.

# cli_tuner.py

This command-line script implements a full calibration process.

Just run cli_tuner.py and it will guide you through the process.

### Under the hood
The calibration process consists of repeatedly running this sequence:

* For each of the seven test points:
  * Go to test point (X, Y) coords with a Z of 1mm
  * Repeat three times:
    * Press 2 repeatedly to lower in 0.05mm increments until 0.1mm feeler gage shows resistance.
    * Press 5 to indicate end of this probing
    * Home and go to test point (X, Y) with a Z of 1mm
  * Probing height average is calculated from the three measurements before
* RMS height error is calculated and printed to screen
* The current printer geometry is requested
* The adjustment algorithm is run with the data from the probing and the printer geometry.
* The new printer geometry is applied
* A message is printed to ask if the data should be saved permanently in the printer firmware
or if a new calibration cycle should be performed. 
    
    

[Escher3D's website]: http://escher3d.com/pages/wizards/wizarddelta.php
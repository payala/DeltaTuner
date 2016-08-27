# DeltaTuner
Tune & Calibrate your delta printer

This is the repository for the DeltaTuner software. It is based on the least squares delta calibration
implementation of RepRapFirmware. The same algorithm can be found in [Escher3D's website] for those that don't have
RepRapFirmware. The website is this one:

The main contribution of this software is automatic g-code generation from the PC to move the printer 
to the test points and automatically capture the height errors at each point. All the code is sent via
USB to the printer directly, so the tuning can be much faster and less error-prone.

At the moment, this code is non-functional, the first bits are being created.

[Escher3D's website]: http://escher3d.com/pages/wizards/wizarddelta.php
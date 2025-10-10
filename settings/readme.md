# Note

After first inserting a SD card into the ILI SD port on the display the firmware will create
the required directory tree.

Place the setting files into the settings folder.

The files are not compatible with the old firmware.

DPM/DPH voltage and current settings are not restored and need to be set manually.


The settings file for aluminum is just a very rough example. Not very accurate and 
I will refine it. But it gives a good starting point as Aluminum is not
very EDM friendly. It creates an oxid layer very quick and then stops conducting.

The cut needs to remove material faster then the oxidation can happen.

I made a cut in 20mm Aluminum with the settings and it did the cut in a single pass.


The file uses a very fast vFd drop reaction with the vdrop threshold close to the idle vFd. If DPM is used with 60v log the vFd value without load and adjust vdrop threshold to be like 200-300 below that idle value. For example: If pwm is enabled and DPM is at 60v and it shows like 3800 vFd set the threshold to 3600 or 3500. My box is configured for the DPH and there vFd read 3970 at 84v. The DPM will/should produce 3970 at 60v and therefore requires different vdrop threshold settings.

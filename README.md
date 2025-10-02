* Bugs fixed
  - changing steps per mm via ui was bricked - fixed (5.3.2025)
</br>
</br>

# Reamer mode is not tested yet. Wire and sinker works.
```diff
+ 1. Validate homing routines, XYZ and all with Z enabled and disabled (z not tested but it should work)
+ 2. Validate probing routines (right front edge confirmed working. Other egdes not tested. Should work)
+ 3. Validate sinker mode with and without retractions (Y positive, Y negative, X negative tested with and without flushing interval)
- 4. Validate reamer mode
+ 5. Validate 2d wire mode ( did some cuts. Seems to work)
```
</br>
</br>
</br>

```diff
______ _                 _                  ______________  ___
| ___ \ |               | |                |  ___|  _  \  \/  |
| |_/ / |__   __ _ _ __ | |_ ___  _ __ ___ | |__ | | | | .  . |
|  __/| '_ \ / _` | '_ \| __/ _ \| '_ ` _ \|  __|| | | | |\/| |
| |   | | | | (_| | | | | || (_) | | | | | | |___| |/ /| |  | |
\_|   |_| |_|\__,_|_| |_|\__\___/|_| |_| |_\____/|___/ \_|  |_/
```                                                               

</br>
</br>

# Youtube demo

[![PhantomEDM demo video](https://img.youtube.com/vi/0D8Diti-8R8/0.jpg)](https://www.youtube.com/watch?v=0D8Diti-8R8)

</br>
</br>

# PhantomEDM

    Firmware for wire and sinker EDM machines build with the G-EDM EVO electronics.

    It supports router type CNC machines with three axis plus a spindle stepper. Rotary axis support is planned for the future.

    For 2D wire EDM a z axis is not needed. 

</br>
</br>


# Install

    Download the repo and extract the folder. Load the folder into visual studio code.
    Make sure the platform.io extension is installed in vstudio. After the folder is loaded it will prepare everything.
    This can take some time. Once finished restart vcode. If you already have a running G-EDM firmware on the ESP you can
    update it by placing the compiled binary on the SD card and just insert it. The initial version will require a restart of the ESP
    to start the update. Once the Phantom release is installed a restart is not required anymore for update over SD. 

    To use update via SD card press the compile button. The firmware.bin file should be located in the project folder at /.pio/build/esp32dev/firmware.bin Just copy the firmware.bin file into the top root directory of the SD card.

    If this is the initial install on a fresh ESP without any G-EDM firmware it needs to be flashed from vcode. Just press the upload button while the board is connected via USB. On windows it may be needed to install the USB to UART drivers first: https://www.silabs.com/developer-tools/usb-to-uart-bridge-vcp-drivers

</br>
</br>

# Factory reset after initial install and on updates is recommended

After the machine receives an update it is good to factory reset everything. Insert an SD card and open the SD menu. Press the last tab and touch the factory reset button. This deletes the NVS key value store used to store the settings from the last session.



</br>
</br>

# Adjust vFd to match the new stock configuration

The new code has a different configuration for the voltage feedback (vFd) short circuit threshold. 
The new default vFd should be almost 4000 @ the max possible voltage. Set the DPM/DPH to the max voltage. if the DPH is used with the 0-80V PSU something around 80V is the max possible. With the DPM it is 60V. Then turn the motionswitch to OFF and press the start button on the display. This will enable the scope. Ensure that the DPM/DPH is also turned on and adjust the Poti on the pulseboard until the reading below "vFd" shows almost 4000. It should never go above 4000. Also please follow this tutorial to have the initial low voltage vFd set: https://www.youtube.com/watch?v=eksJdCTOryA

It is very important to have the vFd set to a low voltage feedback before ever connecting the JST bridge to the sensorcircuit!

</br>
</br>

# Hidden features

1: If the screen is touched on bootup it will enforce a display recalibration. Once the display turns black it can be untouched and the calibration will start
</br>
</br>
2: If for some reason the display blanks out in the process it is possible to re-init the display by touching any point on the display that is not a touch element. In the process only two elements are touchable. Settings button and pause button. Touching the area above those buttons will re-init the display (don't know if it works in the case of a bricked screen) Works only if the settingsmenu is not open.


</br>
</br>



# What machine should you use?

    This is up to you as long as the machine is a router (no moving table) Support for moving tables will follow but the code started to get complex and therefore I focused on routers and skipped support for mills.

    The Motionboard runs with 4 TMC2209 drivers and I use big Nema17 steppers with 1.67A. A few people removed the TMC drivers and used the raw step/dir signals to run external drivers with Nema23 motors. It is possible too.

I highly recommend to take a look at the work from Alex Treseder for an easy to build wire EDM machine:</br>

https://github.com/alextreseder/picoEDM

The EVOII router is just another option but much harder too build, more expensive and a little overweight:</br>

https://github.com/G-EDM/GEDM-EVO2-CNC


</br>
</br>
</br>
Contact me for PCBs or order them on PCBway. 

[>>> Motionboard <<<](https://www.pcbway.com/project/shareproject/G_EDM_Low_budget_DIY_Wire_EDM_machine_34e1e043.html)
</br>
</br>
[>>> Pulseboard EVOIII <<<](https://www.pcbway.com/project/shareproject/G_EDM_EVOIII_board_Low_budget_DIY_Wire_EDM_machine_482d5b7a.html)



</br>
</br>
Schematics and Gerber files can be found here:

https://github.com/G-EDM/GEDM-EVO2-CNC/tree/main/files/schematics-and-gerbers


</br>
</br>
</br>
# Follow the project:

[>>> Follow the project on Youtube <<<](https://www.youtube.com/@G-EDM/videos)

[>>> Stay informed on Hackaday <<<](https://hackaday.io/project/190371-g-edm)

[>>> Get involved on Discord <<<](https://discord.gg/9cTsyDkEbe)

</br>
</br>

# Donations

    * You want to donate something to support the project? 
    * Paypal: paypal.me/gedmdev
    * Bitcoin: bc1q9akp00a5hceh9n3jc9wfttxuwuk9c7da0sqkr8
</br>
<img src="https://raw.githubusercontent.com/G-EDM/G-EDM/main/images/artwork/donations/donate.png">

</br>
</br>

# Legal notes

    The author of this project is in no way responsible for whatever people do with it.
    No warranty. 


</br>   
</br>

# Credits to whom credits belong

    Thanks for the support and help to keep the project going.

    @ Tanatara
    @ 8kate
    @ Ethan Hall
    @ Esteban Algar
    @ 666
    @ td1640
    @ Nikolay
    @ MaIzOr
    @ DANIEL COLOMBIA
    @ charlatanshost
    @ tommygun
    @ renyklever
    @ Zenitheus
    @ gerritv
    @ cnc
    @ Shrawan Khatri
    @ Alex Treseder
    @ VB



# Responsible for the content provided

    Lautensack Roland (Germany)
    Contact: goblin-dev@proton.me

</br>

# License

    All files provided are for private use only if not declared otherwise and any form of commercial use or redistribution of the protected files is prohibited. 
    

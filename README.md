# Warning

It seems there could be an issue if this version is updated on an ESP containing the "old/stable" firmware. If there is the stable firmware present it is currently recommended to use a fresh ESP for this version or hard reset the board. The issue may be related to a different esp-idf version used in the new code.

It seems to work if the ESP with the old version gets a real factory reset with esptool:

To fully reset your ESP32, you can use a tool called esptool to erase the flash memory. Connect your ESP32 to your computer, install Python and esptool, then run the command `python -m esptool --chip esp32 erase_flash` in your terminal while holding the BOOT button on the device.

# Developer Code (Use on own risk)

Only people who don't need a readme here should install it.


Install Tips: Don't flash the firmware from PC to ESP if there is already an older firmware present. Use the automatic SD firmware update by placing the firmware.bin file created with vstudio onto the SD card top root directory "/". Insert the card and wait for the update to start and finish. It is possible that the old code used a different way to store the display calibration data. The developer code uses NVS to store the data and it is possible that a re calibration of the display is needed. if it doesn't start it on it's own and the touch does not work correct just press the display a littler harder while powering everything on. Press until the display turns black and untouch. Follow the instructions shown on the screen then.

Normally the firmware should reset the NVS storage but it is recommended to do a manual factory reset after updating.

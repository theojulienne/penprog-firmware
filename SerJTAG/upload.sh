echo "erase"
dfu-programmer at90usb82 erase
echo "flash"
dfu-programmer at90usb82 flash USBtoSerial.hex
echo "eeprom"
dfu-programmer at90usb82 flash-eeprom USBtoSerial.eep 

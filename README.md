# SenseDistance

Measure distance with ESP32-S2 mini board / ESP32 POE and VL53L0X sensor

## Licence    
    
[Licensed under the European Union Public License (EUPL)-1.2-or-later](https://joinup.ec.europa.eu/collection/eupl/eupl-text-eupl-12)    
    
[Why licensed under EUPL-1.2: it is compatible to GPL and compatible to EU-rights and regulations](https://joinup.ec.europa.eu/collection/eupl/join-eupl-licensing-community)    

[Based on ESP32_ABL - Thank you!](https://github.com/raibisch/ESP32_ABL)

## Functions in Web-Interface

- INDEX-Page: display the last measured distance 
- CONFIG-DATA: set and store parameter and WiFi-credentials
- EVENT-LOG: Serial debug logging
- UPDATE: over-the-air (Wifi) Software update
- INFO: Version, Build, Temp(ESP-intern), IP, Timeout, Charge-Cnt, RSSI

### Fetch actual Values and State

`http:<your-ip>/fetch`

Receive:
`42`

decoded:
`<distance [mm]>`

## Download and Installation

* To download click the DOWNLOAD ZIP button, rename the uncompressed folder 'ESP32_SenseDistance'
...or clone it with git 

### PlatformIO and Arduino-IDE

Projekt was build and testet with PlatformIO.

Take care to upload the 'data' folder to the SPIFFS filesystem 
see: https://randomnerdtutorials.com/esp32-vs-code-platformio-spiffs/

With the following two command you can create and upload the SPIFFS image:

```
pio run --target buildfs
pio run --target uploadfs
```

### Flash Program (without PlatformIO or Arduino-IDE)

If you do not want to compile the program from source:

for the ESP32-S2 mini board I supply the actual firmware-version
- got to subfolder `firmware/lolin_s2_mini/V1_2`

- put ESP32-S2 board to flash-mode: 
- disconnect USB then press and hold "0"-button
- reconnect USB hold "0"-button
- then release "0"-button
--> now the board is in program-mode

- run `flash.sh` (for Linux) or `flash.bat` (for Windows)
(needs 'esptool.py') --> ask google

- after flashing reset or disconnect USB
- search WIFI connetions
- connect (without password) 
- start your webbrowser at "192.168.4.1" (this is the startpage for the APP)
  to connect to your home-route navigate to "Setup" --> "Config-Data" and change:

```config
varWIFI_s_Mode=STA; 
varWIFI_s_Password=mypassword;
varWIFI_s_SSID=myrouter;
```

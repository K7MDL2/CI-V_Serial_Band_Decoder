| Pi CPUs | ![alt text][Pi5B] | ![alt text][Pi4B] |    
| --- | --- | --- |

| Network Options | ![alt text][POE++] | ![alt text][VLAN] |
| --- | --- | --- |

| Radios supported | ![alt text][IC-905] | ![alt text][IC-705] | ![alt text][IC-9700] |
| --- | --- | --- | --- |

| Python versions tested | ![alt text][Python311] | ![alt text][Python312] |
| --- | --- | --- |

| wfView | ![alt text][wfView] |
| --- | --- |

[Pi5B]: https://img.shields.io/badge/-Pi%205B-purple "Pi 5B"
[Pi4B]: https://img.shields.io/badge/-Pi%204B-green "Pi 4B"
[Pi3B]: https://img.shields.io/badge/-Pi%203B-orange "Pi 3B"
[IC-905]: https://img.shields.io/badge/-IC--905-cyan "IC-905"
[IC-705]: https://img.shields.io/badge/-IC--705-cyan "IC-705"
[IC-9700]: https://img.shields.io/badge/-IC--9700-cyan "IC-9700"
[Python311]: https://img.shields.io/badge/-Python%203.11-red "Python311"
[Python312]: https://img.shields.io/badge/-Python%203.12-red "Python312"
[POE++]: https://img.shields.io/badge/-POE++-yellow "POE++"
[VLAN]: https://img.shields.io/badge/-VLAN-blue "VLAN"
[wfView]: https://img.shields.io/badge/-wfView%202.04-purple "wfView"

### Update Mar 15, 2025 
1. Completed support for the IC-9700.  It now handles the Sub RX in 2 ways.  The Sub RX band frequency can be displayed but ignored for BAND and PTT output purposes, always using the MAIN band for relay control.  Or it can operate as normal cross-band split, switching the BAND and PTT outputs between the 2 bands.  See the setup info below for MAIN_TX (IC-9700 only).  Using March 15 build of wfView, has a fix for ID command though I have worked around it for now.

### Update Mar 13, 2025
1. I changed the name of the main file from CI-V_Serial.py to CIV_Serial.py. The install script and service files are updated to match.  You will want to delete /usr/local/bin/CI-V_Serial.py.  This is to make some things cleaner, code does not like hyphens in variables.
2. GPIO PTT input is now working.  The config file has 2 new settings, one for GPIO input pin number and WIRED_PTT to select WIRED or POLL mode. Polled mode is rather slow since all polling is in 1 second minimum resolution timer thread. Some commands are sent less frequently.  Most are held off during PTT.
3. At startup the radio model (aka CIV address) is read from the config file to load a preferred default Frequency table.  When connected to a radio, the actual CIV address is detected and used to set the model dynamically reloading the frequency table.  Long term it will be easier to allow at least 1 alternate address in case there are 2 of the same model radios in use connected to a logger.  wfView uses controller address 0xE1.  I am using 0xE0.
4. The IC-9700, IC-705 and IC-905 are tested.  I only have the borrowed 9700 for a short time longer so most of the testing is on that for now.  I am still working on getting the support for the 2nd receiver and cross band split working correctly.  It was working then I did major rework to the serial port code to improve error handling and attempt recovery (still in progress).
5. Cross band split on all radios is mostly working. The delay between the band change and PTT is missing.  Also have to work out duplex vs split.
6. Serial port connection recovery is a work in progress.
7. I am using wfView 2.04 March 10 weekly build.  I used the full build script. See Setup and Usage section on this page.  Choose rig-pty1 for the virtual serial port.

### CI-V Serial Band Decoder Program

This is a Python serial port based CI- V band decoder targeting the IC-705, IC-905, and IC-9700 for now.  The use scenario is nearly identical to the TCP sniffer version at https://github.com/K7MDL2/IC905_Ethernet_Decoder, but does not require tapping into the RF Unit comms.  If you use this with a IC-905 and you are not trying to use a single ethernet cable to reach the remote RF Unit, then there is no need for a POE Inserter or any remote switches and VLANs if there is only the Pi at the end of the cable.

In this case I am using ethernet to reach a remote located cabinet with a Pi CPU inside along with other ethernet gear, same as the ethernet TCP sniffer version, but tp get the CI-V data remotely, I am running 'wfView' remote control program (https://wfview.org/) on the Pi which can share the radio CI-V comms over a virtual serial port. From the outside it looks and acts the same as the ethernet sniffer unit but use standard CI-V protocol and a normal LAN connection to the Pi.

The code is a mashup of my USB CI-V Band Decoder C ported to Python, and the ethernet IC-905 Band Decoder which is in Python.  I could have converted the USB project from USB serial to plain serial fairly quickly but I wanted to do it in Python:
1. For the fun of it as this is only my 3rd or 4th significant Python project
2. Run the same hardware in either scenario (ethernet or serial) 
3. Leverage most of the same operations, scripts, config files, logging, output formatting, and Wiki page instructions from the ethernet version

I initially looked at adding GPIO code to wfView and spotted a few places in that code to make such additions.  Then I thought I could simply run a standalone program using the virtual serial ports and not touch the wfView code so I cranked up the keyboard and produced this.  There is likely already a similar Linux and/or Windows CI-V decoder somewhere including commercial units but I wanted to do my own in Python for the reasons above.   My Desktop Band Decoder app is in Python but is my own comm protocol and is USB and ethernet based, targeting a Teensy custom PCB I designed.

Screen shot as of March 10, 2025.  Now has labels for Filter, Mode and Datamode
![{7D67B7BA-FFF0-4CA6-BC95-779014BE8707}](https://github.com/user-attachments/assets/e27a474d-1b84-4575-8a4e-aa2391595cb9)

This now has working frequency and band from serial CI-V along with the same GPIO working as the TCP version for the 905.  The CI-V parser is ported from the C code and fed into the existing Python version functions for frequency and PTT.  One difference is that with CI-V I can poll the radio to get info.  The TCP version is read-only.  

I am porting over selected CI-V library functions and poll the radio at startup.  They include polled PTT, date, time, UTC offset, location, preamp, attenuator, frequency, and split.  I calculate grid square to 8 places.  GPIO for relays is working.   If you run this at the same time as TCP905.py, make sure to use separate GPIO pin assignments so they do not conflict.

### ToDo

To be added: 
1. I have a 7" HDMI touchscreen I may add some graphics UI.
2. Stretch goal: Direct connect to the radio LAN port and not require wfView in the middle.

### Networking Options

The TCP905 version decoder requires a tap into the dedicated controller to RF Unit ethernet data.  The RF Unit is set up on a managed switch(es) in a VLAN and port mirroring is enabled on that VLAN so we can monitor the data.  The data exchanged is not CI-V and is unpublished.  Since there is direct access tothe ethernet, CPU usage is very low. ATV mode however exceeds a Pi3 100MB ethernet capacity.

This decoder uses standard CI-V commands using wfView as a ethernet to serial bridge. This lets you use the LAN/WiFi side ethernet connection uses standard CI-V protocols. It requires no VLAN or port mirror so standrd unmanged switches work.  Putting wfview on the same CPU as the Decodewr app and using virtual serial to connect them lets you place the CPU anywhere your ethernet network can reach.  Even WiFi might work.  Someday I hope to directly access the radio via the LAN with no bridging software required, but for now the serial is the easy way to do this. Also a direct connecxtion woudsl let a smaller CPU like a Pi 3 to work.

Bottom line, if you already have, or can run an ethernet cable out to where you want your band decoder relays, you plug in a Pi4 or Pi5 with some relay or other suitable IO into the network and control your gear. The IO mapping can be customised per band.  I have not tested this on the Pi4 yet.  I doubt it will work on a Pi3. The Pi5 is running about 60% CPU and GPU at 24% with the wfView spectrum on or off.  A Pi4 will be pressed.

### Setup and usage

wfView is used as a LAN to serial bridge. Installing wfView on the Pi is fairly simple.  I used the fullbuild-wfview script.  You can find the wfView 2.x files, instructions, and install/build script for Pi here.  See the wfView section for installing wfView.
   
Use the Wiki pages on the IC905 TCP Ethnernet Decoder project.  This program is a variation and is very similar with just a few name changes along witthe config notes applied below.
[https://github.com/K7MDL2/IC905_Ethernet_Decoder](https://github.com/K7MDL2/IC905_Ethernet_Decoder/wiki)

The main app name here is CIV_Serial.py and has the same install script, logs, config, and service files with names changed from Decoder905 to Decoder. 
 This permits parallel operation with the TCP905 version which is how things are in my dev environment.  Be sure to use unique IO pins to avoid GPIO conflicts. The 'view_log' tool is now 'view_Decoder_log'.

I support multiple radios.  I auto-detect the CI-V address and dynamically change the frequency table and IO pin table reloading the necessary variables to switch between the 905 (and 9700 which is the same) and the 705, hopefully other radios later.  

I have a config file entry inside the file ~/Decoder.config that will be overriden by the detected CI-V address.  In the future I am thinking of using these entries to override the address to account for non-standard CI-V addresses or multiple radio configs in a single file. They could have different IO pin assignments. For now it is used to specifiy the initial frequency table loaded before a CI-V address is detected.

    # --------------------------------
    # 
    # Enter the model radio to get the right band assignments
    # Choices are IC705, IC905, or IC9700
    #
    # --------------------------------

    RADIO_MODEL=IC9700

The parameter MAIN_TX is defaulted to 1.  The IC-9700 always transmits on the MAIN band.  When Sub RX is enabled, the Sub RX frequency will be displayed by the program for VFOA.  When MAIN_TX=1 the BAND and PTT output pins will be set to the MAIN band and will ignore the SubRX band.  When MAIN_TX=0, then the BAND and PTT outputs will switch to the Sub RX band in RX, and then to the MAIN band during TX.  This is cross-band split like the 905 and 705.  I have a programmable delay between the BAND and PTT outputs set to 0.2Sec.

    # -------------------------------- 
    # IC-9700 Only
    # 1 = Use main band only for Band Relays
    # 0 = Switch Band relays to Sub Band in RX, then to MAIN band in TX (crossband split)
    # --------------------------------

    MAIN_TX = 1
    
If the program is current running in the background as a systemd service then use the stop/start utilities to cause the config to be reloaded.

Here are the new PTT input pin and inversion mode additions

    # To use a wired GPIO input set this = 1, else set to 0 for Polled PTT status
    WIRED_PTT=1

    # ---------------------------------
    # This is the pin assigned to monitor a wired PTT input from the radio
    # ---------------------------------

    GPIO_PTT_IN_PIN=16
    GPIO_PTT_IN_PIN_INVERT=True

Be sure to edit the band and pin number for your external IO hardware.  In my test setup I have a 3 relay HAT module.  I am using 1 relay for PTT and the other 2 as indication to me that the band IO is working.  There is only 2 relays so I just set them up to change as I sequentially go through the bands.  Since there are only 2 relays for band, there can be 4 states - all on, #1 on, #2 on, all off. Since there are 6 bands, 2 of them will have to use the same pattern as 2 other bands.  More relays of course is better.  

Realize that the IO procress looks through the list of pins and applies the pattern one pin at a time. If you only have 1 relay, say for PTT, then the PTT pattern will be b'000001' or 0x01.  The lowest pin in the map (ptt band 0 here) will be set first followed by the others. If all 6 ptt pins are set to the same pin IO number then the pin will be set at first then unset for the remaining 5 bit positions.  To make this work, assign the band 0 ptt io pin to your relay pin number, then assign the rest of the ptt pins 1-5 to some other unused pin.

### wfView setup

I am using wfView 2.04 March 15 weekly build (or later).  I used the full build script. The wfView home page is here:

https://www.wfview.org

There are standard releases on the download page.  Scroll down the page you will see info for weekly builds and install instructions.  I am using weekly builds found here:

https://wfview.org/download/

https://www.wfview.org/public_builds/

https://wfview.org/wfview-user-manual/headless-server/  for headless server build - appears to be serial only radio interface so not useful for this project

I normally choose the latest version.  If it has problems it is easy enough to use an earlier verison.  I have last years 1.65 loaded also for testing on occasion.  It has minimal support for the IC-905 so I use 2.X builds.  Further down on the Download page are install instructions.  For the Pi I use the fullbuild-wfview.sh script.  You download the script, make it executable (chmod + x filename), then run it.

    https://gitlab.com/eliggett/scripts/-/blob/master/fullbuild-wfview.sh

After the install script is done there will be a desktop icon. You can also use command line.  Once started, open Settings->Radio Access and enter your radio IP address and login info.  Once you have a good connection to your radio, go to Settings->External Control and choose 'rig-pty1' for the virtual serial port.   You can use wfView as normal now and the band deocder will do its thing.

To reduce CPU usage, once I have the settings confirmed working and saved,  I can run "wfview -platform vnc" on the command line to run without the usual graphics and on a Pi5 it reduces CPU around half to 30%.  Be sure to stop and start the Decoder service each time you stop/start wfView (for now).


### Visual Studio Code and Code Server setup

I have been using VS Code for Arduino, ESP-IDF, and Python for years now.  It has integrated GitHub source control support and runs on multiple platforms and is far more productive than the Arduino IDE.  The ESP-IDF runs as an extension in VS Code.  Running the UI on a Pi4 was a bit tedious so I edited mostly on the PC.  

With the much faster Pi5 here and this project, I have started using Code-Server running on the Pi5.  It allows you to run the VS Code UI using a browser on the local or any remote machine with all the same features including an integrated terminal and for Python a debugger.  I highly recommend it. For Python in particular setup is very simple.  See these links below.  I have used both the manual and the scripted installs, both are easy and work.  The version I used is 4.98.0.

    https://github.com/coder/code-server
    https://coder.com/docs/code-server/install

Another similar way to run VS Code for dev on the Pi is run VS Code on your main computer and use the blue "Open a Remote Window" icon in the lower left corner.  Choose ssh conection to your Pi.  Enter the IP and credentials and it sets up code server remotely.

Another useful tool is FilelZilla which provides a file explorer type view between the local and one or more remote machines.

| Pi CPUs | ![alt text][Pi5B] | ![alt text][Pi4B] |    Network Options | ![alt text][POE++] | ![alt text][VLAN] |
| --- | --- | --- | --- | --- | --- |

| Radios supported | ![alt text][IC-905] | ![alt text][IC-705] | ![alt text][IC-9700] |
| --- | --- | --- | --- |

| Python versions tested | ![alt text][Python311] | ![alt text][Python312] | 
| --- | --- | --- |

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


This is a Python serial port based CI- V band decoder targeting the IC-705, IC-905, and IC-9700 for now.  The use scenario is nearly identical to the TCP sniffer version at https://github.com/K7MDL2/IC905_Ethernet_Decoder, but does not require tapping into the RF Unit comms.  If you use this with a IC-905 and you are not trying to use a single ethernet cable to reach the remote RF Unit, then there is no need for a POE Inserter or any remote switches and VLANs if there is only the Pi at the end of the cable.

In this case I am using ethernet to reach a remote located cabinet with a Pi CPU inside along with other ethernet gear, same as the ethernet TCP sniffer version, but tp get the CI-V data remotely, I am running 'wfView' remote control program (https://wfview.org/) on the Pi which can share the radio CI-V comms over a virtual serial port. From the outside it looks and acts the same as the ethernet sniffer unit but use standard CI-V protocol and a normal LAN connection to the Pi.

The code is a mashup of my USB CI-V Band Decoder C ported to Python, and the ethernet IC-905 Band Decoder which is in Python.  I could have converted the USB project from USB serial to plain serial fairly quickly but I wanted to do it in Python:
1. For the fun of it as this is only my 3rd or 4th significant Python project
2. Run the same hardware in either scenario (ethernet or serial) 
3. Leverage most of the same operations, scripts, config files, logging, output formatting, and Wiki page instructions from the ethernet version

I initially looked at adding GPIO code to wfView and spotted a few places in that code to make such additions.  Then I thought I could simply run a standalone program using the virtual serial ports and not touch the wfView code so I cranked up the keyboard and produced this.  There is likely already a similar Linux and/or Windows CI-V decoder somewhere including commercial units but I wanted to do my own in Python for the reasons above.   My Desktop Band Decoder app is in Python but is my own comm protocol and is USB and ethernet based, targeting a Teensy custom PCB I designed.

Screen shot as of March 10, 2025.  Now has labels for Filter, Mode and Datamode
![{7D67B7BA-FFF0-4CA6-BC95-779014BE8707}](https://github.com/user-attachments/assets/e27a474d-1b84-4575-8a4e-aa2391595cb9)

This now has working frequency and band from serial CI-V along with the same GPIO working as the TCP version for the 905.  The CI-V parser is being ported from the C code and fed into the existing Python version functions for frequency and PTT.  One difference is that with CI-V I can poll the radio to get info.  The TCP version is read-only.  

I am porting over selected CI-V library functions and poll the radio at startup.  They include polled PTT, date, time, UTC offset, location, preamp, attenuator, frequency, and split.  I calculate grid square to 8 places.  GPIO for relays is working.   If you run this at the same time as TCP905.py, make sure to use separate GPIO pin assignments so they do not conflict.

### ToDo

To be added: 
1. Add GPIO PTT input monitoring for wired PTT and hook it and the polled PTT into the existing PTT function.  Currently polling every 1 sec for CI-V PTT status
2. I have a 7" HDMI touchscreen I may add some graphics UI.
3. Get dynamic radio config switching working.
5. Stretch goal: Direct connect to the radio LAN port and not require wfView in the middle.
   

### Setup and usage

Use the Wiki pages on the IC905 TCP Ethnernet Decoder project.  This is very similar with just a few name changes.
[https://github.com/K7MDL2/IC905_Ethernet_Decoder](https://github.com/K7MDL2/IC905_Ethernet_Decoder/wiki)

The main app name here is CI-V_Serial.py and has the same install script, logs, config, and service files with names changed from Decoder905 to Decoder. 
 This permits parallel operation with the TCP905 version which is how things are in my dev environment.  The 'view_log' tool is now 'view_decoder_log'.

I support multiple radios.  I auto-detect the CI-V address and dynamically change the frequency table and IO pin table reloading the necessary variables to switch between the 905 (and 9700 which is the same) and the 705, hopefully other radios later.  

I have a config file entry inside the file ~/Decoder.config that will be overrriden by the detected CI-V address.  In the future I am thinking of using these entries to override the address to account for non-stanbard CI-V addresses or multiple radio configs in a single file. They could have different IO pin assignments.

    # --------------------------------
    # 
    # Enter the model radio to get the right band assignments
    # Choices are IC705, IC905
    # For the IC-9700 choose IC905.  Uncomment the desired selection
    #
    # --------------------------------

    #RADIO_MODEL=IC905
    RADIO_MODEL=IC705

Not setting the right model results in the wrong band frequency limits and band labels applied and GPIO won't be correct.   If you change this you must restart the program.  If it is running as a systemd service then use the stop/start utilities.

### Visual Studio Coe and Coder Server setup

I have been using VS Code for Arduino, ESP-IDF, and Python for years now.  It has integrated GitHub source control support and runs on multiple platforms and is far more productive than the Arduino IDE.  The ESP-IDF runs as an extension in VS Code.  Running the UI on a Pi4 was a bit tedious so I edited mostly on the PC.  

With the much faster Pi5 here and this project, I have started using Code-Server running on the Pi5.  It allows you to run the VS Code UI use a browser on the local or any remote machine with all the same features including an integrated terminal and for Python a debugger.  I highly recommend it. For Python in particular setup is very simple.  See these links below.  I have used both the manual and the script installs, both are easy and work.  The current version is 4.98.0.

    https://github.com/coder/code-server
    https://coder.com/docs/code-server/install

Another useful too is FilelZilla which provides a file explorer type view between the local and one or more remote machines.

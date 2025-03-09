| Uses | ![alt text][Pi5B] | ![alt text][Pi4B] | ![alt text][Pi3B] | ![alt text][IC-905] | ![alt text][Python311] | ![alt text][Python312] | ![alt text][POE++] | ![alt text][VLAN] | 
| --- | --- | --- | --- | --- | --- | --- | --- | --- |

[Pi5B]: https://img.shields.io/badge/-Pi%205B-purple "Pi 5B"
[Pi4B]: https://img.shields.io/badge/-Pi%204B-green "Pi 4B"
[Pi3B]: https://img.shields.io/badge/-Pi%203B-orange "Pi 3B"
[IC-905]: https://img.shields.io/badge/-IC--905-cyan "IC-905"
[Python311]: https://img.shields.io/badge/-Python%203.11-red "Python311"
[Python312]: https://img.shields.io/badge/-Python%203.12-red "Python312"
[POE++]: https://img.shields.io/badge/-POE++-yellow "POE++"
[VLAN]: https://img.shields.io/badge/-VLAN-blue "VLAN"


This is a Python serial port based CI- V band decoder targeting the IC-705, IC-905, and IC-9700 for now.  The use scnario is nearly identical to the TCP sniffer version, but does not require tapping into the RF Unit comms.

In this case I am using ethernet to reach a remote located Pi CPU, same as the ethernet sniffer version, but I am running wfView remote control program on the Pi which can share the radio CI-V comms over a virtual serial port. From the outside it should, when completed, look and act the same as the ethernet sniffer unit but use standard CI-V protocol.

The code is a mashup of my USB CI-V Band Decoder C ported to Python, and the ethernet IC-905 Band Decoder which is in Python.  I could have converted the USB project from USB to plain serial fairly quickly but I wanted to do it in Python:
1. For the fun of it as this is only my 3rd or 4th significant Python project
2. Run the same hardware in either scenario (ethernet or serial) 
3. Leverage most of the same operations, scripts, config files, logging, output formatting, and Wiki page instructions from the ethernet version

The initial upload to this repo has working frequency and band from serial CI-V along with the same GPIO working as the TCP version.  The CI-V parser is being ported from the C code and fed into the existing Python version functions for freuqency and Ptt.  One difference is that with CI-V I can poll the radio to get info.  The TCP version is read-only.  I still have a lot of the CI-V library to port over, including adding serial TX to poll the radio.

I initially looked at adding GPIO code to wfView and spotted a few places in that code to make such additions.  Then I thought I could simply run a standalone program using the virtual serial ports and not touch the wfView code so I cranked up the keyboard and produced this.   There is likely already a similar Linux and/or Windows CI-V decoder but I wanted to do my own in Python for the reasons above.   My Desktop Band Decoder app is in Python but is my own comm protocol and is USB and ethernet based, targeting a Teensy custom PCB I designed.


### Setup and usage

Use the Wiki pages on the IC905 TCP Ethnernet Decoder project.  This is very similar with just a few name changes.
https://github.com/K7MDL2/IC905_Ethernet_Decoder

The main app name is CI-V_Serial.py and has all teh same install script, logs, config, and service files with changed names from Decoder905 to Decoder.  This permits parallel operation with the TCP905 version.   The view_log is now view_decoder_log.

One significant change in this version is multiple radio support.  While I autodetect the CI-V address, I have no working means to dynamically change the frequency table to switch between the 905 (and 9700 which is the same) and the 705 and later other radios.  I have a new config file entry 

This is inside the file ~/Decoder.config

    # --------------------------------
    # 
    # Enter the model radio to get the right band assignments
    # Choices are IC705, IC905
    # For the IC-9700 choose IC905
    #
    # --------------------------------

    #RADIO_MODEL=IC905
    RADIO_MODEL=IC705

Not setting the right model results in the wrong band frequency limits and band labels applied and GPIO won't be correct.
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




This is a Python serial port based CI- V band decoder targeting the IC-705, IC-905, and IC-9700 for now.  There is a twist available that can make use of this nearly identically to the TCP sniffer version, but not require tapping into the RF Unit comms.  

In this case I am usign ethernet to reach a remote located Pi CPU, same as the ethener snuiffer version, but I am running wfView remote control program on the Pi which can share the radio CI-V comms over a virtual serial port. From the outside it (should, when completed) look and act the same as the ethernet sniffer unit but use standard CI-V protocol.

The code is a mashup of my USB CI-V Band Decoder C ported to Python, and the ethernet IC-905 Band Decoder which is in Python.  I could have converted the USB project from USB to plain serial fairly quickly but I wanted to do it in Python:
1. For the fun of it as this is only my 3rd or 4th significnat Python project
2. Run the same hardware in either scenario (ethnernet or serial) 
3. Leverage most of the same operations, scripts, config files, logging, output formatting, and Wiki page instructions from the ethernet version

The initial upload to this repo has frequency and band from serial working along with the same IO as the TCP version.  The CI-V parser is being ported from the C code and fed into the existing Python version functions for freuency and Ptt.  One difference is that with CI-V I can poll the radio to get info.  The TCP version is read-only.




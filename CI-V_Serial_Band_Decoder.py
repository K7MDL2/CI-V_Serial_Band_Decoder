#!/usr/bin/python

import serial, time
import os
import sys
import numpy as np
from RPi import GPIO
import subprocess as sub
from threading import Timer
from typing import Callable
from datetime import datetime as dtime
from enum import Enum, auto
#import itertools

#  if dht is enabled, and connection is lost, ther program will try to recover the connection during idle periods
dht11_enable = True  # enables the sensor and display of temp and humidity
dht11_OK = True   #  Tracks online status if connection to the DHT11 temp sensor
dht11_poll_time = 300   # pol lthe DHT11 (if installed) every X seconds.
TempC = 0
TempF = 0
Humidity = 0
RING_SIZE = 512
ring_head = 0
ring_tail = 0
ring_data = [0] * RING_SIZE
next_head = 0
CONTROLLER_ADDRESS = 0xe0   # E1 used by wfView  #Controller address - not used for receive validation, used on TX to radio.
BROADCAST_ADDRESS = 0x00
START_BYTE = 0xfe  # Start byte
STOP_BYTE = 0xfd   # Stop byte
CMD_READ_FREQ = 0x03    # Read operating frequency data
radio_address_received = 0
radio_address = 0
init_done = 0
ALL_RADIOS = 0
IC705 = 0xa4
IC905 = 0xac
IC9700 = 0xa2
RADIO_ADDR = ALL_RADIOS   # default ALL_RADIOS

class cmds(Enum):
    CIV_C_F_SEND = 0
    CIV_C_F1_SEND = auto()
    CIV_C_F_READ = auto()
    CIV_C_F25A = auto()
    CIV_C_F25B = auto()
    CIV_C_F26 = auto()
    CIV_C_F26A = auto()
    CIV_C_F26B = auto()
    CIV_C_F25A_SEND = auto()
    CIV_C_F25B_SEND = auto()
    CIV_C_MOD_READ = auto()
    CIV_C_MOD_SET = auto()
    CIV_C_MOD_SEND = auto()
    CIV_C_MOD1_SEND = auto()
    CIV_C_MOD_USB_F1_SEND = auto()
    CIV_C_MOD_USB_SEND = auto()
    CIV_C_USB_D0_F2_SEND = auto()
    CIV_C_USB_D1_F2_SEND = auto()
    CIV_C_LSB_D0_F2_SEND = auto() 
    CIV_C_LSB_D1_F2_SEND = auto() 
    CIV_C_FM_D1_F1_SEND = auto() 
    CIV_C_ATTN_READ = auto()
    CIV_C_ATTN_OFF = auto()
    CIV_C_ATTN_ON = auto()
    CIV_C_SPLIT_READ = auto()
    CIV_C_SPLIT_OFF_SEND = auto()
    CIV_C_SPLIT_ON_SEND = auto()
    CIV_C_RFGAIN = auto()
    CIV_C_AFGAIN = auto()
    CIV_C_RFPOWER = auto()
    CIV_C_S_MTR_LVL = auto()
    CIV_C_PREAMP_READ = auto()
    CIV_C_PREAMP_OFF = auto()
    CIV_C_PREAMP_ON = auto()
    CIV_C_PREAMP_ON2 = auto()
    CIV_C_AGC_READ = auto()
    CIV_C_AGC_FAST = auto()
    CIV_C_AGC_MID = auto()
    CIV_C_AGC_SLOW = auto()
    CIV_C_CW_MSGS = auto()
    CIV_C_BSTACK = auto()
    CIV_C_MY_POSIT_READ = auto()
    CIV_C_MY_POSIT_DATA = auto()
    CIV_C_RF_POW = auto()
    CIV_C_TRX_ON_OFF = auto()
    CIV_C_TRX_ID = auto()
    CIV_C_TX = auto()
    CIV_C_DATE = auto()
    CIV_C_TIME = auto()
    CIV_C_UTC_READ_905 = auto()
    CIV_C_UTC_READ_705 = auto()
    CIV_C_UTC_READ_9700 = auto()
    CIV_C_DUPLEX_READ = auto()
    CIV_C_DUPLEX_SEND = auto()
    CIV_C_RIT_XIT = auto()
    CIV_C_RIT_ON_OFF = auto()
    CIV_C_XIT_ON_OFF = auto()
    CIV_C_RADIO_OFF = auto()
    CIV_C_RADIO_ON = auto()
    CIV_C_SCOPE_ON = auto()
    CIV_C_SCOPE_OFF = auto()
    CIV_C_SCOPE_ALL = auto()
    CIV_R_NO_GOOD = auto()
    CIV_R_GOOD = auto()
    End_of_Cmd_List = auto()


cmd_List = [
    [cmds.CIV_C_F_SEND,          1,0x00],                      # send operating frequency to all
    [cmds.CIV_C_F1_SEND,         1,0x05],                      # send operating frequency to one
    [cmds.CIV_C_F_READ,          1,0x03],                      # read operating frequency
    [cmds.CIV_C_F25A,            2,0x25,0x00],                 # read selected VFO (00) operating frequency
    [cmds.CIV_C_F25B,            2,0x25,0x01],                 # read unselected VFO (01)
    [cmds.CIV_C_F26,        	 1,0x26],                      # read selected VFO m data, filt -  26 datafield template; selected VFO; mode, data on/off(0-1), filter (1-3);
    [cmds.CIV_C_F26A,        	 2,0x26,0x00],                 # read/set selected VFO m data, filt
    [cmds.CIV_C_F26B,       	 2,0x26,0x01],                 # read/set  un- selected VFO m data, filt
    [cmds.CIV_C_F25A_SEND,       2,0x25,0x00],                 # set selected VFO frequency
    [cmds.CIV_C_F25B_SEND,       2,0x25,0x01],                 # set un-selected VFO frequency

    [cmds.CIV_C_MOD_READ,        1,0x04],               	      # read Modulation Mode in use
    [cmds.CIV_C_MOD_SET,         3,0x06,0x23,0x02],  		      # set mode to ATV and FIL2, same 2 byte filed for cmds 1, 4, and 6
    [cmds.CIV_C_MOD_SEND ,       1,0x01],                      # send Modulation Mode to all
    [cmds.CIV_C_MOD1_SEND,       1,0x06],                      # send Modulation Mode to one
    [cmds.CIV_C_MOD_USB_F1_SEND, 3,0x06,0x01,0x01],            # send USB Filter 1 
    [cmds.CIV_C_MOD_USB_SEND,    2,0x06,0x01],                 # send USB Filter 1 
    [cmds.CIV_C_USB_D0_F2_SEND,  5,0x26,0x00,0x01,0x00,0x02],  # selected VFO; mod USB; Data OFF; RX_filter F2;
    [cmds.CIV_C_USB_D1_F2_SEND,  5,0x26,0x00,0x01,0x01,0x02],  # selected VFO; mod USB; Data ON;  RX_filter F2;
    [cmds.CIV_C_LSB_D0_F2_SEND,  5,0x26,0x00,0x00,0x00,0x02],  # selected VFO; mod USB; Data OFF; RX_filter F2;
    [cmds.CIV_C_LSB_D1_F2_SEND,  5,0x26,0x00,0x00,0x01,0x02],  # selected VFO; mod USB; Data ON;  RX_filter F2;
    [cmds.CIV_C_FM_D1_F1_SEND,   5,0x26,0x00,0x05,0x01,0x01],  # selected VFO; mod USB; Data ON;  RX_filter F2;
    
    [cmds.CIV_C_ATTN_READ,   	1,0x11],                  	  # Attn read state
    [cmds.CIV_C_ATTN_OFF,   		2,0x11,0x00],                 # Attn OFF
    [cmds.CIV_C_ATTN_ON,    		2,0x11,0x10],                 # Attn 10dB (144, 432, 1200 bands only)
    [cmds.CIV_C_SPLIT_READ,      1,0x0F],                      # read Split OFF
    [cmds.CIV_C_SPLIT_OFF_SEND,  2,0x0F,0x00],                 # set split OFF
    [cmds.CIV_C_SPLIT_ON_SEND,   2,0x0F,0x01],                 # Set split ON
    [cmds.CIV_C_RFGAIN,          2,0x14,0x02],                 # send/read RF Gain
    [cmds.CIV_C_AFGAIN,          2,0x14,0x01],                 # send/read AF Gain
    [cmds.CIV_C_RFPOWER,         2,0x14,0x0A],                 # send/read selected bands RF power
    [cmds.CIV_C_S_MTR_LVL,       2,0x15,0x02],                 # send/read S-meter level (00 00 to 02 55)  00 00 = S0, 01 20 = S9, 02 41 = S9+60dB
    [cmds.CIV_C_PREAMP_READ,     2,0x16,0x02],             	  # read preamp state
    [cmds.CIV_C_PREAMP_OFF,      3,0x16,0x02,0x00],            # send/read preamp 3rd byte is on or of for sending - 00 = OFF, 01 = ON
    [cmds.CIV_C_PREAMP_ON,       3,0x16,0x02,0x00],            # send/read preamp 3rd byte is on or of for sending - 00 = OFF, 01 = ON
    [cmds.CIV_C_PREAMP_ON2,      3,0x16,0x02,0x02],            # send/read preamp 3rd byte is on or of for sending - 00 = OFF, 01 = ON - not on 905
    [cmds.CIV_C_AGC_READ,        2,0x16,0x12],                 # send/read AGC  01 = FAST, 02 = MID, 03 = SLOW
    [cmds.CIV_C_AGC_FAST,        3,0x16,0x12,0x01],            # send/read AGC  01 = FAST, 02 = MID, 03 = SLOW
    [cmds.CIV_C_AGC_MID,         3,0x16,0x12,0x02],            # send/read AGC  01 = FAST, 02 = MID, 03 = SLOW
    [cmds.CIV_C_AGC_SLOW,        3,0x16,0x12,0x03],            # send/read AGC  01 = FAST, 02 = MID, 03 = SLOW
    [cmds.CIV_C_CW_MSGS,         1,0x17],                      # Send CW messages see page 17 of prog manual for char table
    [cmds.CIV_C_BSTACK,          2,0x1A,0x01],                 # send/read BandStack contents - see page 19 of prog manual.  
                                                                    # data byte 1 0xyy = Freq band code
                                                                    # dat abyte 2 0xzz = register code 01, 02 or 03
                                                                    # to read 432 band stack register 1 use 0x1A,0x01,0x02,0x01
    [cmds.CIV_C_MY_POSIT_READ,   2,0x23,0x00],          	    # read my GPS Position
    [cmds.CIV_C_MY_POSIT_DATA,   1,0x23],          	    	  # read my GPS Position
    [cmds.CIV_C_RF_POW,          2,0x14,0x0A],            		# send / read max RF power setting (0..255 == 0 .. 100%)
    [cmds.CIV_C_TRX_ON_OFF,      1,0x18],                 		# switch radio ON/OFF
    [cmds.CIV_C_TRX_ID,          2,0x19,0x00],            		# ID query
    [cmds.CIV_C_TX,              2,0x1C,0x00],            		# query of TX-State 00=OFF, 01=ON
    # the following three commands don't fit for IC7100 !!!
    [cmds.CIV_C_DATE,            4,0x1A,0x05,0x00,0x94],  		# + 0x20 0x20 0x04 0x27 for 27.4.2020
    [cmds.CIV_C_TIME,            4,0x1A,0x05,0x00,0x95],  		# + 0x19 0x57 for 19:57
    #[cmds.CIV_C_UTC,           4,0x1A,0x05,0x00,0x96],  		# + 0x01,0x00,0x00 = +1h delta of UTC to MEZ
    [cmds.CIV_C_UTC_READ_905,    4,0x1A,0x05,0x01,0x81],     #  Get UTC Offset
    #[cmds.CIV_C_UTC_SEND,      4,0x1A,0x05,0x00,0x96],  		# + 0x01,0x00,0x00 = +1h delta of UTC to MEZ
    [cmds.CIV_C_UTC_READ_705,    4,0x1A,0x05,0x01,0x70],  		# + 0x01,0x00,0x00 = +1h delta of UTC to MEZ
    [cmds.CIV_C_UTC_READ_9700,   4,0x1A,0x05,0x01,0x84],  		# + 0x01,0x00,0x00 = +1h delta of UTC to MEZ
    [cmds.CIV_C_DUPLEX_READ,		1,0x0C],          	    	  # read Duplex Offset  - has 3 bytes frequency offset data
    [cmds.CIV_C_DUPLEX_SEND,     1,0x0D],	          	    	# send Duplex Offset
    [cmds.CIV_C_RIT_XIT,			2,0x21,0x00],          	    # read or send RIT/XIT Offset  - has 3 bytes frequency offset data  XIT and RIT share this Offset value
    [cmds.CIV_C_RIT_ON_OFF,		2,0x21,0x01],	          	  # send or send RIT ON or Off status 00 = , 01 = t
    [cmds.CIV_C_XIT_ON_OFF,		2,0x21,0x02],	          	  # send or send XIT Offset
    [cmds.CIV_C_RADIO_OFF,		2,0x18,0x00],	          	  # Turn Off the radio
    [cmds.CIV_C_RADIO_ON,		2,0x18,0x01],	          	  # Turn on the radio
    [cmds.CIV_C_SCOPE_ON,        3,0x27,0x11,0x01],          # send/read Scope wave data output ON
    [cmds.CIV_C_SCOPE_OFF,       3,0x27,0x11,0x00],          # send/read Scope wave data output OFF
    [cmds.CIV_C_SCOPE_ALL,       1,0x27],                    # send/read Scope catch all to avoid no match found error outputs
    [cmds.CIV_R_NO_GOOD,         1,0xFA],                    # Message received from radio was no good
    [cmds.CIV_R_GOOD,            1,0xFB]                     # Message received from radio was good
]                        

#  Freq_table:
#  These band edge frequency values are based on the radio message VFO
#    values which have no offset applied
#  We use this value then once we know the band we can add the
#    fixed offset and then display the actual dial frequency

#  The 10G band values in this table are just dummy values until
#    the 10G transverter is hooked up to observe the actual values

# The band and ptt values are the mapping to the group of
#    6 pins for band and 6 pins for ptt
#    Set the pin value(s) = 1 that you want activated when the band is active
#    There is an inversion flag to corert for buffer inversions

# At startup all pins will be set to 0 then initialized once the band
#    is first determined.

# For BCD output to the Remote BCD Decoder board, edit the band
#    values = 0 through 5 so only using 3 io pins
#    and ptt values will all be set to 1 using only 1 io pin
# Example values for BCD decoder
#   2M decimal   0 or in binary format 0b0000000
#   70cm decimal 1 or in binary format 0b0000001
#   23cm decimal 2 or in binary format 0b0000010
#   13cm decimal 3 or in binary format 0b0000011
#    6cm decimal 4 or in binary format 0b0000100
#    3cm decimal 5 or in binary format 0b0000101
# Set all bands ptt to decimal 1 or in binary format 0b0000001

# thsi is set up for 3 wire BCD + 1 wire PTT for the Remote Decdoer board
Freq_table = { '2M': {
                    'lower_edge':144000000,
                    'upper_edge':148000000,
                        'offset':0,
                          'band':0b00000000,
                           'ptt':0b00000001,
                },
                '70cm': {
                    'lower_edge':430000000,
                    'upper_edge':450000000,
                        'offset':0,
                          'band':0b00000001,
                           'ptt':0b00000001,
                },
                '23cm': {
                    'lower_edge':1240000000,
                    'upper_edge':1300000000,
                        'offset':0,
                          'band':0b00000010,
                           'ptt':0b00000001,
                },
                '13cm': {
                    'lower_edge':2300000000,
                    'upper_edge':2550000000,
                        'offset':0,
                          'band':0b00000011,
                           'ptt':0b00000001,
                },
                '6cm': {
                    'lower_edge':5600000000,
                    'upper_edge':5800000000,
                        'offset':0,
                          'band':0b00000100,
                           'ptt':0b00000001,
                },
                '3cm': {
                    'lower_edge':10000000000,    # 10000,000.000 = 1,389,000,000
                    'upper_edge':10500000000,
                        'offset':0,    #10.0G - 1.389G = 8,611,000,000
                          'band':0b00000101,
                           'ptt':0b00000001,
                }
            }

# IO-Table:
# These are the GPIO pin assignments of BAND and PTT outputs.
# 1 or more pins may be assigned to any band so they are not band specific.
# The band and ptt keys in the Freq_table map the bank of pins to a band

# We use up to 6 pins for band output and up to 6 for PTT
# BCD mode will use fewer pins and the extras will be ignored
# set the inversion this to match your hardware.  Buffering usually inverts the logic

# The 3 relay HAT I have uses pin CH1=26  CH2=20  CH3=21 (25, , 28, 29 using Wiring Pi numbers on the board

# This is set up for  3-wire BCD Band and 1-wire PTT for the Remote BCD DEcdoer board
IO_table = {
                 0x01 : {
                      'band_pin':5,  #4,
                   'band_invert':False,
                       'ptt_pin':0,  #16, for 4 relay hat,  #17, for 3 relay hat, 0 for antenna only and no PTT
                    'ptt_invert':False,
                 },
                 0x02 : {
                      'band_pin':6,  #3,
                   'band_invert':False,
                       'ptt_pin':0,
                    'ptt_invert':True,
                 },
                 0x04 : {
                      'band_pin':13, #2,
                   'band_invert':False,
                       'ptt_pin':0,
                    'ptt_invert':True,
                 },
                 0x08 : {
                      'band_pin':0,
                   'band_invert':True,
                       'ptt_pin':0,
                    'ptt_invert':True,
                 },
                 0x10: {
                      'band_pin':0,
                   'band_invert':True,
                       'ptt_pin':0,
                    'ptt_invert':True,
                 },
                 0x20 : {
                      'band_pin':0,
                   'band_invert':True,
                       'ptt_pin':0,
                    'ptt_invert':True,
                }
            }


#  __________________________________________________________________
#
#  GPIO outputs for Band and PTT
#  __________________________________________________________________
#

class OutputHandler:

    def get_time(self):
        d = dtime.now()
        return d.strftime("%m/%d/%y %H:%M:%S")


    def gpio_config(self):
        GPIO.setmode(GPIO.BCM)
        for i in IO_table:
            band_pin = IO_table[i]['band_pin']
            band_invert =  IO_table[i]['band_invert']
            ptt_pin = IO_table[i]['ptt_pin']
            ptt_invert = IO_table[i]['ptt_invert']
            print("i=", format(i, '06b'), "band_pin:", band_pin, " ptt_pin", ptt_pin)
            GPIO.setup(band_pin, GPIO.OUT, initial=band_invert)
            GPIO.setup(ptt_pin,  GPIO.OUT, initial=ptt_invert)
        print("GPIO pin mode setup complete", flush=True)


    def ptt_io_output(self, band, ptt):
        for __band_name in Freq_table:
            if (__band_name == band):
                band_pattern = Freq_table[__band_name]['ptt']
                # Found a band match, set ptt io pin(s)
                if ptt:
                    p = bd.colored(255,0,0,"(+TX++)")
                else:
                    p = bd.colored(45,255,95,"(-RX--)")

                b = bd.colored(255,235,145, format(str(band),"5"))
                bp = bd.colored(0,255,255, format(band_pattern,'06b'))
                print(p,"Output for "+b+" Pattern:"+bp, flush=True)

                if (ptt):
                    ptt = 0xff
                else:
                    ptt = 0x00

                for __pins in IO_table:
                    pin_invert = IO_table[__pins]['ptt_invert']
                    io_pin     = IO_table[__pins]['ptt_pin']
                    pin_state  = (band_pattern & __pins & ptt)

                    pin_state = bool(pin_state)   # convert decimal number to a boolean value

                    if pin_invert:
                        pin_state = pin_state ^ 1 # invert the pin
                        #print("pin state after inversion:", int(pin_state))

                    #print("index", __pins, "pin state:", pin_state,"on",io_pin, "inverted", pin_invert)

                    GPIO.output(io_pin, pin_state)  # set our pin


    def band_io_output(self, band):
        # turn on selected band output
        for __band_name in Freq_table:
            if (__band_name == band):
                band_pattern = Freq_table[__band_name]['band']
                # Found a band match, now loop through the set of IO pins
                t = bd.colored(235,110,200, "(BAND )")
                b = bd.colored(255,225,145, format(str(band),"5"))
                p = bd.colored(0,255,255, format(band_pattern,'06b'))
                print(t,"Output for "+b+" Pattern:"+p, flush=True)
                template = 0x0000

                for __pins in IO_table:
                    pin_invert = IO_table[__pins]['band_invert']
                    io_pin     = IO_table[__pins]['band_pin']
                    pin_state  = (band_pattern & __pins)

                    if pin_state:
                        pin_state = 1
                    else:
                        pin_state = 0

                    if pin_invert:
                        pin_state = pin_state ^ 1 # invert the pin
                        #print("pin state after inversion:", int(pin_state))

                    #print("index", __pins, "pin state:", pin_state,"on",io_pin, "inverted", pin_invert)
                    GPIO.output(io_pin, pin_state)

#-----------------------------------------------------------------------------
#
#   Threaded timer for periodic logging of things such as temp, or
#     restart a failed temp device connection without blocking the main program
#
#-----------------------------------------------------------------------------

class RepeatedTimer(object):
    def __init__(self, interval: int, function: Callable, args=None, kwargs=None):
        super(RepeatedTimer, self).__init__()
        self._timer = None
        self.interval = interval
        self.function = function
        self.args = [] if args is None else args
        self.kwargs = {} if kwargs is None else kwargs
        self.is_running = False
        self.start()

    def _run(self):
        self.is_running = False
        self.start()
        self.function(*self.args, **self.kwargs)

    def start(self):
        if not self.is_running:
            self._timer = Timer(self.interval, self._run)
            self._timer.start()
            self.is_running = True

    def stop(self):
        self._timer.cancel()
        self.is_running = False

#  __________________________________________________________________
#
#  Packet data processing function thread object
#  __________________________________________________________________
#


class DecoderThread(object):
    def __init__(self, function: Callable, args=None, kwargs=None):
        super(DecoderThread, self).__init__()
        self._timer = None
        self.function = function
        self.args = [] if args is None else args
        self.kwargs = {} if kwargs is None else kwargs
        self.is_running = False
        self.start()

    def _run(self):
        self.is_running = False
        self.start()
        self.function(*self.args, **self.kwargs)

    def start(self):
        if not self.is_running:
            self.is_running = True

    def stop(self):
        self._timer.cancel()
        self.is_running = False

#  __________________________________________________________________
#
#  Packet data processing functions
#  __________________________________________________________________
#

class BandDecoder(): #OutputHandler):
# We are inheriting from OutputHandler class so we can access its functions
#   and variables as if they were our own.
    vfoa_band = ""
    vfoa_band_split_Tx = ""
    vfob_band = ""
    selected_vfo = 0
    unselected_vfo = 255
    selected_vfo_split_Tx = 0
    split_status = 255
    preamp_status = 255
    atten_status =255
    ptt_state = 0
    modeA = 255
    filter = 255
    datamode = 255
    in_menu = 0
    payload_len = 0
    payload_copy = ""
    payload_ID = 0
    payload_ID_byte = 0
    payload_Attrib_byte = 0
    frequency_init = 0

    def __init__(self):
        self.__offset = 0
        self.__freq_last = 0
        self.__vfoa_band_last = 255
        self.__ptt_state_last = 255
        self.PTT_hang_time = 0.3
        self.__split_status_last = 255

    #--------------------------------------------------------------------
    #  Process config file
    #--------------------------------------------------------------------

    def str_to_bool(self, s):
        return {'true': True, 'false': False}.get(s.lower(), False)


    def read_DHT(self, key_value_pairs):
        global dht11_enable
        global dht11_poll_time
        #for key in key_value_pairs:
        dht11_enable = self.str_to_bool(key_value_pairs['DHT11_ENABLE'])
        dht11_poll_time = int(key_value_pairs['DHT11_TIME'])


    def read_split(self, key_value_pairs):
        #Initialize split based on last known status
        self.split_status = int(key_value_pairs['RADIO_SPLIT'])
        #print("Read file",self.split_status)


    def read_band(self, key_value_pairs):
        # Initialize the band and VFOs to the last saved values, lost likely will be correct.
        # Alternative is to do nothng and block PTT
        __band_name = key_value_pairs['RADIO_BAND']
        __offset = Freq_table[__band_name]['offset']
        self.vfoa_band = self.vfob_band = self.vfoa_band_split_Tx = __band_name
        self.selected_vfo = self.unselected_vfo = self.selected_vfo_split_Tx = Freq_table[__band_name]['lower_edge'] + __offset +1
        self.ptt_state = 0


    def read_patterns(self, key_value_pairs):
        Freq_table['2M']['band'] = band_2M = int(key_value_pairs['BAND_2M'],base=16)
        Freq_table['2M']['ptt'] = ptt_2M = int(key_value_pairs['PTT_2M'],base=16)
        #print("2M Band pattern:  ", hex(band_2M), "PTT:", hex(ptt_2M))

        Freq_table['70cm']['band'] = band_70cm = int(key_value_pairs['BAND_70cm'],base=16)
        Freq_table['70cm']['ptt'] = ptt_70cm = int(key_value_pairs['PTT_70cm'],base=16)
        #print("70cm Band pattern:", hex(band_70cm), "PTT:", hex(ptt_70cm))

        Freq_table['23cm']['band'] = band_23cm = int(key_value_pairs['BAND_23cm'],base=16)
        Freq_table['23cm']['ptt'] = ptt_23cm = int(key_value_pairs['PTT_23cm'],base=16)
        #print("23cm Band pattern:", hex(band_23cm), "PTT:", hex(ptt_23cm))

        Freq_table['13cm']['band'] = band_13cm = int(key_value_pairs['BAND_13cm'],base=16)
        Freq_table['13cm']['ptt'] = ptt_13cm = int(key_value_pairs['PTT_13cm'],base=16)
        #print("13cm Band pattern:", hex(band_13cm), "PTT:", hex(ptt_13cm))

        Freq_table['6cm']['band'] = band_6cm = int(key_value_pairs['BAND_6cm'],base=16)
        Freq_table['6cm']['ptt'] = ptt_6cm = int(key_value_pairs['PTT_6cm'],base=16)
        #print("6cm Band pattern: ", hex(band_6cm), "PTT:", hex(ptt_6cm))

        Freq_table['3cm']['band'] = band_3cm = int(key_value_pairs['BAND_3cm'],base=16)
        Freq_table['3cm']['ptt'] = ptt_3cm = int(key_value_pairs['PTT_3cm'],base=16)
        #print("3cm Band pattern: ", hex(band_3cm), "PTT:", hex(ptt_3cm))


    def read_band_pins(self, key_value_pairs):
        IO_table[0x01]['band_pin'] = gpio_band_0_pin = int(key_value_pairs['GPIO_BAND_0_PIN'])
        IO_table[0x01]['band_invert'] = gpio_band_0_pin_invert = self.str_to_bool(key_value_pairs['GPIO_BAND_0_PIN_INVERT'])
        #print("Band Pin 0: ", gpio_band_0_pin, " Invert:", gpio_band_0_pin_invert)

        IO_table[0x02]['band_pin'] = gpio_band_1_pin = int(key_value_pairs['GPIO_BAND_1_PIN'])
        IO_table[0x02]['band_invert'] = gpio_band_1_pin_invert = self.str_to_bool(key_value_pairs['GPIO_BAND_1_PIN_INVERT'])
        #print("Band Pin 1: ", gpio_band_1_pin, " Invert:", gpio_band_1_pin_invert)

        IO_table[0x04]['band_pin'] = gpio_band_2_pin = int(key_value_pairs['GPIO_BAND_2_PIN'])
        IO_table[0x04]['band_invert'] = gpio_band_2_pin_invert = self.str_to_bool(key_value_pairs['GPIO_BAND_2_PIN_INVERT'])
        #print("Band Pin 2: ", gpio_band_2_pin, " Invert:", gpio_band_2_pin_invert)

        IO_table[0x08]['band_pin'] = gpio_band_3_pin = int(key_value_pairs['GPIO_BAND_3_PIN'])
        IO_table[0x08]['band_invert'] = gpio_band_3_pin_invert = self.str_to_bool(key_value_pairs['GPIO_BAND_3_PIN_INVERT'])
        #print("Band Pin 3: ", gpio_band_3_pin, " Invert:", gpio_band_3_pin_invert)

        IO_table[0x10]['band_pin'] = gpio_band_4_pin = int(key_value_pairs['GPIO_BAND_4_PIN'])
        IO_table[0x10]['band_invert'] = gpio_band_4_pin_invert = self.str_to_bool(key_value_pairs['GPIO_BAND_4_PIN_INVERT'])
        #print("Band Pin 4: ", gpio_band_4_pin, " Invert:", gpio_band_4_pin_invert)

        IO_table[0x20]['band_pin'] = gpio_band_5_pin = int(key_value_pairs['GPIO_BAND_5_PIN'])
        IO_table[0x20]['band_invert'] = gpio_band_5_pin_invert = self.str_to_bool(key_value_pairs['GPIO_BAND_5_PIN_INVERT'])
        #print("Band Pin 5: ", gpio_band_5_pin, " Invert:", gpio_band_5_pin_invert)


    def read_ptt_pins(self, key_value_pairs):
        IO_table[0x01]['ptt_pin'] = gpio_ptt_0_pin = int(key_value_pairs['GPIO_PTT_0_PIN'])
        IO_table[0x01]['ptt_invert'] = gpio_ptt_0_pin_invert = self.str_to_bool(key_value_pairs['GPIO_PTT_0_PIN_INVERT'])
        #print("PTT Pin 0: ", gpio_ptt_0_pin, " Invert:", gpio_ptt_0_pin_invert)

        IO_table[0x02]['ptt_pin'] = gpio_ptt_1_pin = int(key_value_pairs['GPIO_PTT_1_PIN'])
        IO_table[0x02]['ptt_invert'] = gpio_ptt_1_pin_invert = self.str_to_bool(key_value_pairs['GPIO_PTT_1_PIN_INVERT'])
        #print("PTT Pin 1: ", gpio_ptt_1_pin, " Invert:", gpio_ptt_1_pin_invert)

        IO_table[0x04]['ptt_pin'] = gpio_ptt_2_pin = int(key_value_pairs['GPIO_PTT_2_PIN'])
        IO_table[0x04]['ptt_invert'] = gpio_ptt_2_pin_invert = self.str_to_bool(key_value_pairs['GPIO_PTT_2_PIN_INVERT'])
        #print("PTT Pin 2: ", gpio_ptt_2_pin, " Invert:", gpio_ptt_2_pin_invert)

        IO_table[0x08]['ptt_pin'] = gpio_ptt_3_pin = int(key_value_pairs['GPIO_PTT_3_PIN'])
        IO_table[0x08]['ptt_invert'] = gpio_ptt_3_pin_invert = self.str_to_bool(key_value_pairs['GPIO_PTT_3_PIN_INVERT'])
        #print("PTT Pin 3: ", gpio_ptt_3_pin, " Invert:", gpio_ptt_3_pin_invert)

        IO_table[0x10]['ptt_pin'] = gpio_ptt_4_pin = int(key_value_pairs['GPIO_PTT_4_PIN'])
        IO_table[0x10]['ptt_invert'] = gpio_ptt_4_pin_invert = self.str_to_bool(key_value_pairs['GPIO_PTT_4_PIN_INVERT'])
        #print("PTT Pin 4: ", gpio_ptt_4_pin, " Invert:", gpio_ptt_4_pin_invert)

        IO_table[0x020]['ptt_pin'] = gpio_ptt_5_pin = int(key_value_pairs['GPIO_PTT_5_PIN'])
        IO_table[0x20]['ptt_invert'] = gpio_ptt_5_pin_invert = self.str_to_bool(key_value_pairs['GPIO_PTT_5_PIN_INVERT'])
        #print("PTT Pin 5: ", gpio_ptt_5_pin, " Invert:", gpio_ptt_5_pin_invert)

    def init_band(self, key_value_pairs):
        self.read_DHT(key_value_pairs)
        self.read_patterns(key_value_pairs)
        self.read_band_pins(key_value_pairs)
        self.read_ptt_pins(key_value_pairs)
        io.gpio_config()
        self.vfoa_band = self.frequency(self.selected_vfo)
        self.ptt()

    def write_split(self, split):
        file_path = os.path.expanduser('~/.Decoder.split')
        try:
            with open(file_path,'w+') as file:
                split_str = "RADIO_SPLIT="+str(split)
                file.write(split_str)
        except FileNotFoundError:
            print(f"The file {file} does not exist in the home directory.")
        except Exception as e:
            print(f"An error occured: {e}")


    def write_band(self, band):
        file_path = os.path.expanduser('~/.Decoder.band')
        try:
            with open(file_path,'w+') as file:
                band_str = "RADIO_BAND="+band
                file.write(band_str)
        except FileNotFoundError:
            print(f"The file {file} does not exist in the home directory.")
        except Exception as e:
            print(f"An error occured: {e}")


    def hexdump(self, data: bytes):
        def to_printable_ascii(byte):
            return chr(byte) if 32 <= byte <= 126 else "."

        offset = 0
        while offset < len(data):
            chunk = data[offset : offset + 16]
            hex_values = " ".join(f"{byte:02x}" for byte in chunk)
            ascii_values = "".join(to_printable_ascii(byte) for byte in chunk)
            print(f"{offset:08x}  {hex_values:<48}  |{ascii_values}|", flush=True)
            offset += 16

    # -------------------------------------------------------------------
    #
    # DHT-11 Humidity and Temperature sensor
    #
    #   The DHT-11 and CPU temperature is logged with each print to
    #       screen at the end of the print line.
    #
    #--------------------------------------------------------------------

    def get_cpu_temp(self):
        temp = os.popen("vcgencmd measure_temp").readline()
        return temp.replace("temp=", "").strip()[:-2]

    def write_temps(self, line):   #, file):
        file_path = os.path.expanduser('~/Temperatures.log')
        try:
            with open(file_path,'a') as file:
                file.write(line)
        except  FileNotFoundError:
            print(f"The file {file} does not exist in the home directory.")
        except Exception as e:
            print(f"An error occured: {e}")

    def read_dht(self, file):
        f = open(file,"rt")
        value = int(f.readline())
        f.close
        return value

    def read_temps(self):
        global dht11_OK
        global dht_enable
        global TempC
        global TempF
        global Humidity

        t = h = tF = 2    #  a value of zero inidicates failure so starting with 1

        if (dht11_enable == False):   # failures result in bus timeout delays so do not try again
            return t, h, tF

        try:
            if  (dht11_OK):
                t = self.read_dht("/sys/bus/iio/devices/iio:device0/in_temp_input")/1000
                h = self.read_dht("/sys/bus/iio/devices/iio:device0/in_humidityrelative_input")/1000
                tF = t * (9 / 5) + 32

        # If failure is due to device not present, bus timeout delays
        # since we are running in our own thread we can retry the read and
        # bus timeouts won't affect the main radio message handling

        except RuntimeError as error:
            # Errors happen fairly often, DHT's are hard to read, just keep going
            print("DHT11 RunTime Eror =", error.args[0], flush=True)
            t = h = tF = 0
            dht11_OK = True

        except Exception as error:
            print("DHT11 Read error = ",error, flush=True)
            t = h = tF = 0
            dht11_OK = True
            #raise error

        TempC = t
        TempF = tF
        Humidity = h

        return t, h, tF


    def temps(self):
        global dht11_enable

        if dht11_enable:
            (temp, hum, temp_F) = self.read_temps()
        else:
            temp = hum= temp_F = 0
        cpu = self.get_cpu_temp()
        tim = dtime.now()
        temp_str = (tim.strftime("%m/%d/%Y %H:%M:%S%Z")+"  Temperature: %(f)0.1fÂ°F  %(t)0.1fÂ°C  Humidity: %(h)0.1f%%  CPU: %(c)sÂ°C  DHT11:%(e)d" % {"t": temp, "h": hum, "f": temp_F, 'c': cpu, 'e': int(dht11_enable)})
        print(self.colored(100,120,255,"(TEMP )"), temp_str, flush=True)
        self.write_temps(temp_str+"\n")


    def check_msg_valid(self):
        if (self.payload_ID != 0xa803 and self.payload_ID != 0x0000):
            if (self.payload_copy[0x000a] != 0x44 and self.payload_copy[0x000b] != 0x00):
                #print("Rejected message from ID", format(self.payload_ID, "04x"))
                return  1 # get avoid garbage versions
            else:
                #print("Accepted message from ID", format(self.payload_ID, "04x"))
                return 0   # return 1 for bad, 0 for good


    def p_status(self, TAG):
        global TempC
        global TempF
        global Humidity
        cpu = self.get_cpu_temp()
        #tim = dtime.now()
        print(self.colored(175,210,240,"("+TAG+")"),
            #tim.strftime("%m/%d/%y %H:%M:%S%Z"),
            " VFOA Band:"+self.colored(255,225,165,format(self.vfoa_band,"4")),
            " A:"+self.colored(255,255,255,format(self.selected_vfo, "11")),
            " B:"+self.colored(215,215,215,format(self.unselected_vfo, "11")),
            " Split:"+self.colored(225,255,90,format(self.split_status, "1")),
            " M:"+format(self.modeA, "1"),
            " F:"+format(self.filter, "1"),
            " D:"+format(self.datamode, "1"),
            " P:"+format(self.preamp_status, "1"),
            " A:"+format(self.atten_status, "1"),
            " PTT:"+self.colored(115,195,110,format(self.ptt_state, "1")),
            #" Menu:"+format(self.in_menu, "1"),   #  this toggles 0/1 when in menus, and/or when there is spectrum flowing not sure which
            " Src:0x"+format(self.payload_ID, "04x"),
            #" T:%(t)0.1fÂ°F  H:%(h)0.1f%%  CPU:%(c)sÂ°C" % {"t": TempF, "h": Humidity, "c": cpu}, # sub in 'temp' for deg C
            flush=True)


    # If we see corrupt values then look at the source.
    # Some messages are overloaded - meaning they can have radio
    #   status or have other spectrum like data in the same length
    #   and ID+Attrib combo.  Calling check_msg_valid to filter out
    #   bad stuff based on observed first row byte patterns

    def case_x18(self):  # process items in message id # 0x18
        #self.hexdump(self.payload_copy)
        #print("(ID:18) Length",self.payload_len)

        if self.check_msg_valid():
            return

        self.split_status = self.payload_copy[0x001b] # message #0xd400 @ 0x0001
        # There is no preamp or atten on bands above 23cm
        if (self.vfoa_band == "13cm" or self.vfoa_band == "6cm"):
            self.atten_status = 0
            self.preamp_status = 0
        else:
            self.atten_status = self.payload_copy[0x011d]
            self.preamp_status = self.payload_copy[0x011c]
        self.modeA = self.payload_copy[0x00bc]
        self.filter = self.payload_copy[0x00bd]+1
        self.datamode = self.payload_copy[0x00be]
        self.in_menu = self.payload_copy[0x00d8]
        self.frequency(self.selected_vfo)
        #self.p_status("ID:18") # print out our state


    # get this message when split is changed
    #  x18 has the status
    # process items in message #0xd4 0x00
    # attr = 01 is long and mostly zeros
    # d400 can be filled with other type data on some occasions, maybe band specific, not sure.

    def case_xD4(self):
        #self.hexdump(self.payload_copy)
        #print("(ID:D4) Length",self.payload_len)

        if self.check_msg_valid():
            return

        self.split_status = self.payload_copy[0x001b] # message #0xd400 @ 0x0001
        #self.split_status = self.payload_copy[0x00b3] # message #0xd400 @ 0x0001
        self.modeA = self.payload_copy[0x00bc]
        self.filter = self.payload_copy[0x00bd]+1
        self.datamode = self.payload_copy[0x00be]
        self.in_menu = self.payload_copy[0x00d8]
        self.frequency(self.selected_vfo)
        #self.p_status("ID:D4") # print out our state


    # convert little endian bytes to int frequency
    # vfo is the starting address for desired in the payload
    def get_freq(self, __payload, vfo):
        np.set_printoptions(formatter={'int':hex})
        freq_hex_dec = np.array([0, 0, 0, 0],dtype=np.uint8)

        for i in range(0, 4, 1):
            freq_hex_dec[i] = (__payload[vfo+i])
        #print(freq_hex_dec)

        # Convert from ascii array to binary hex byte string
        freq_hex_str = freq_hex_dec.tobytes()
        #print(freq_hex_str)

        # Flip from little to big endian
        byte_data = bytes(freq_hex_str)
        little_endian_bytes = byte_data[::-1]
        little_endian_hex_str = little_endian_bytes.hex()
        freq = int(little_endian_hex_str, base=16)
        #print(freq)
        # Now we have a decimal frequency
        return freq


    def frequency(self, freq):  # 0xd8 0x00 is normal tuning update, 0x18 on band changes
        
        # Duplex used split status byte for VFO swap, just has vfoB set different with offset
        # split is updated in message ID 0xD4.  Here we can also pick it up and not wait for 
        # someone to press the split button to generate the d4 event.
        
        # Returns the payload hex converted to int.
        # This need to have the band offset applied next
        __vfoa = freq
        #print("(Freq) VFO A = ", __vfoa)
        __vfob = freq
        #print("(Freq) VFO B = ", __vfob)

        if (self.vfoa_band == "13cm" or self.vfoa_band == "6cm"):
            self.atten_status = 0
            self.preamp_status = 0

        if (self.split_status != self.__split_status_last):
            self.write_split(self.split_status)
            self.__split_status_last = self.split_status

        # Look for band changes
        if (__vfoa != self.__freq_last):
            # Search the Freq_table to see what band these values lie in
            for __band_name in Freq_table:
                if (__vfoa >= Freq_table[__band_name]['lower_edge'] and
                    __vfoa <= Freq_table[__band_name]['upper_edge'] ):
                    # Found a band match, print out the goods
                    self.__offset = Freq_table[__band_name]['offset']
                    self.selected_vfo = __vfoa + self.__offset
                    self.vfoa_band = __band_name

                if (__vfob >= Freq_table[__band_name]['lower_edge'] and
                    __vfob <= Freq_table[__band_name]['upper_edge'] ):
                    # Found a band match, print out the goods
                    self.__offset = Freq_table[__band_name]['offset']
                    self.unselected_vfo = __vfob + self.__offset
                    self.vfob_band = __band_name
            self.p_status("FREQ ") # print out our state
            #print("  Lower edge = ", Freq_table[self.vfoa_band]['lower_edge'] + self.__offset,
            #      "  Upper edge = ", Freq_table[self.vfoa_band]['upper_edge'] + self.__offset,
            #      "  Offset = ", self.__offset)

            #  set band outputs on band changes
            if (self.vfoa_band != self.__vfoa_band_last):
                io.band_io_output(self.vfoa_band)
                self.__vfoa_band_last = self.vfoa_band
                self.write_band(self.vfoa_band)  # update the status file on change

            self.__freq_last = __vfoa
        else:
            self.p_status("FREQ ") # print out our state

        return self.vfoa_band


    # When spectrum is enabled AND there is noise+signal > ref line
    #   dump spectrum data in 0xe8001
    #  bytes 0000-0011 never change spectrum starts a 0x0012
    def spectrum(self):
        #hexdump(s"(spectrum)"elf.payload_copy)
        pass


    # 0x2c 0x00 - get at start of PTT and occaionally in RX
    # 0x2c 0x01 - get at most(but not all) mode changes.  Some are skipped.
    #      at pos 0x08 always = 0x24
    #      attr byte 8   byte 9   byte A  byte B   byte bd
    #      0x01 24       0x01     44      00       4 = RTTY   FM,SSB mode, mo msg for cw
    #      0x01 24       0x01     44      00       1 = USB
    #      0x01 24       0x01     44      00       7 = FM
    #      0x01 24       0x01     44      00       8 = DV then
    #           24         01     c0      02    in DV and often repeats and has gps data in place of frequency and mode, still 308 long
    #  Goes bad
    #      0x01 24         01     C0      02      garbage
    #   0 is LSB
    #   2 is cw
    #   6 is AM

    def mode(self): # not likely the real mode change as only some issue this message
        #  must be some other primary event
        #hexdump(self.payload_copy)
        #print("Length",self.payload_len)

        if self.check_msg_valid():
            return

        self.split_status = self.payload_copy[0x00b3] # message #0xd400 @ 0x0001
        self.modeA  = self.payload_copy[0x00bc]
        self.filter = self.payload_copy[0x00bd]+1
        self.datamode = self.payload_copy[0x00be]
        self.frequency()


    # PTT sequence
    # 0xe8-00 - see Github Wiki pages for examples of mesaage ID flow
    # 0xe8-01 is spectrum data
    def ptt(self):
        #self.hexdump(self.payload_copy)
        #print("Length",self.payload_len)

        if self.check_msg_valid():
            return

        # watch for PTT value changes
        if (self.vfoa_band != ""):   # block PTT until we know what band we are on
            #print("PTT called")

            if self.payload_ID != 0x0000:
                if self.frequency_init == 0:
                    self.frequency()
                    self.frequency_init = 1
                self.ptt_state = self.payload_copy[0x00ef]
            else:
                self.ptt_state = 0
                #self.__ptt_state_last = 255

            if (self.ptt_state != self.__ptt_state_last):
                #print("PTT state =", self.ptt_state)
                if (self.ptt_state == 1):  # do not TX if the band is still unknown (such as at startup)
                    #print("PTT TX VFO A Band = ", self.vfoa_band, "VFO B Band = ", self.vfob_band, "  ptt_state is TX ", self.ptt_state, " Msg ID ", hex(self.payload_copy[0x0001]))
                    if (self.split_status == 1): # swap selected and unselected when split is on during TX
                        self.vfoa_band_split_Tx = self.vfoa_band  # back up the original VFOa band
                        self.selected_vfo_split_Tx = self.selected_vfo  # back up original VFOa
                        self.selected_vfo = self.unselected_vfo  # during TX assign b to a
                        self.vfoa_band = self.vfob_band
                        #print("PTT TX1 VFO A Band = ", self.vfoa_band, "VFO B Band = ", self.vfob_band,  " ptt_state is TX ", self.ptt_state)

                        # skip the band switch and delay if on the same band
                        if (self.vfoa_band != self.vfoa_band_split_Tx):
                            self.p_status("SPLtx")
                            io.band_io_output(self.vfoa_band)
                            time.sleep(self.PTT_hang_time)
                            print("Delay:",self.PTT_hang_time,"sec", flush=True)
                        else:
                            self.p_status(" DUP ")
                        io.ptt_io_output(self.vfoa_band, self.ptt_state)
                    else:
                        self.p_status(" PTT ")
                        io.ptt_io_output(self.vfoa_band, self.ptt_state)

                else:   #(self.ptt_state == 0):
                    #print("PTT-RX VFO A Band =", self.vfoa_band, "VFO B Band =", self.vfob_band, "VFOA BAND SPLIT TX =", self.vfoa_band_split_Tx, "SELECTED VFO SPLIT TX =", self.selected_vfo, " ptt_state is RX ", self.ptt_state) #, " Msg ID ", hex(self.payload_copy[0x0001]))
                    if (self.split_status == 1): # swap selected and unselected when split is on during TX
                        self.vfoa_band = self.vfoa_band_split_Tx
                        self.selected_vfo = self.selected_vfo_split_Tx
                        #print("PTT-RX1 VFO A Band = ", self.vfoa_band, "VFO B Band = ", self.vfob_band,  " ptt_state is RX ", self.ptt_state)

                        # skip the band switch and delay if on the same band
                        if (self.vfoa_band != self.vfob_band):
                            self.p_status("SplRx")
                            io.ptt_io_output(self.vfoa_band, self.ptt_state)
                            time.sleep(self.PTT_hang_time)
                            print("Delay:",self.PTT_hang_time,"sec", flush=True)
                            io.band_io_output(self.vfoa_band)
                        else:
                            #self.p_status(" DUP ")
                            io.ptt_io_output(self.vfoa_band, self.ptt_state)
                            pass
                    else:
                        #self.p_status(" PTT ")
                        io.ptt_io_output(self.vfoa_band, self.ptt_state)

                self.__ptt_state_last = self.ptt_state


    def TX_on(self):
        print("(Tx_on) Transmitting... - sometimes not")
        self.hexdump(self.payload_copy)
        print("(TX_on) Length:", self.payload_len)


    def dump(self):
        print("Dump for message 0x"+format(self.payload_ID,"04x")+"  Len:", format(self.payload_len), flush=True)
        self.hexdump(self.payload_copy)
        #print("(dump) Length:", self.payload_len)


    def bcd_hex_to_decimal(self, bcd_hex):
        # Convert the BCD hex string to an integer
        bcd_int = int(bcd_hex, 10)
        # Initialize the decimal string
        decimal_str = ""
        # Process each BCD digit
        while bcd_int > 0:
            # Extract the last BCD digit (4 bits)
            bcd_digit = bcd_int & 0xF
            # Convert the BCD digit to a decimal digit and prepend to the decimal string
            decimal_str = str(bcd_digit) + decimal_str
            # Shift right by 4 bits to process the next BCD digit
            bcd_int >>= 4
        return decimal_str.zfill(2)


    # Bytes 0xA9=day 0xAA=Month 0xAB=2-digit year 0xAC=UTC_Hr 0xAD=Min 0xAE=Sec
    def time_sync(self):
        #self.hexdump(self.payload_copy)
        #print("time_sync", self.payload_copy)
        #print("(time) Length:", self.payload_len)
        tday=self.bcd_hex_to_decimal(str(self.payload_copy[0xa9]))
        tmon=self.bcd_hex_to_decimal(str(self.payload_copy[0xaa]))
        tyr=self.bcd_hex_to_decimal(str(self.payload_copy[0xab]))
        thr=self.bcd_hex_to_decimal(str(self.payload_copy[0xac]))
        tmin=self.bcd_hex_to_decimal(str(self.payload_copy[0xad]))
        tsec=self.bcd_hex_to_decimal(str(self.payload_copy[0xae]))
        #print("Date: %02s/%02s/%02s  Time: %02s:%02s:%02s" % (tmon,tday,tyr,thr,tmin,tsec), flush=True)

    def unhandled(self):
        return "unhandled message"


    def case_default(self):
        self.hexdump(self.payload_copy)
        __payload_len = len(self.payload_copy)
        print("(case_default) Unknown message,ID:0x"+format(self.payload_ID,'04x')+"  Length:", __payload_len, flush=True)
        return "no match found"


    def colored(self, r, g, b, text):
        return f"\033[38;2;{r};{g};{b}m{text}\033[0m"

#
#   End of class BandDecoder
#

#--------------------------------------------------------------------
#  Read config file
#--------------------------------------------------------------------

def read_config(config_file):
    global dht11_enable
    global dht11_poll_time

    try:
        with open(config_file,"r") as file:
            key_value_pairs = {}
            current_key = None
            current_value = None
            for line in file:
                line = line.strip()
                # Check if the line is empty or a comment (starts with #)
                if not line or line.startswith('#'):
                    # Skip this line
                    continue
                # Check if the line starts with a tab character
                elif line.startswith('\t'):
                    # Check if we have a current key and value
                    if current_key is not None and current_value is not None:
                        # Append the line to the current value
                        current_value += '\n' + line.lstrip()
                    else:
                        # Log an error and skip this line
                        print(f"Error: Invalid line format - {line}")
                        continue
                else:
                    # Check if we have a current key and value
                    if current_key is not None and current_value is not None:
                        # Add the current key-value pair to the dictionary
                        key_value_pairs[current_key] = current_value
                    # Split the line into key and value
                    key_value = line.split('=', 1)
                    if len(key_value) != 2:
                        # Log an error and skip this line
                        print(f"Error: Invalid line format - {line}")
                        continue
                    # Set the current key and value
                    current_key = key_value[0].strip()
                    current_value = key_value[1].strip()
            # Check if we have a current key and value
            if current_key is not None and current_value is not None:
                # Add the current key-value pair to the dictionary
                key_value_pairs[current_key] = current_value
            # Return the dictionary of key-value pairs
            #print(key_value_pairs)
            return key_value_pairs

    except FileNotFoundError:
            print(f"The file {config_file} does not exist in the home directory.")
            bd.write_band(bd.vfoa_band)
    except Exception as e:
            print(f"An error occurred: {e}")



def CIV_Action(cmd_num:int, data_start_idx:int, data_len:int, msg_len:int, rd_buffer):
    global cmds
    global radio_address
    global ALL_RADIOS
    global IC705
    global IC905
    global IC9700
    global RADIO_ADDR
    
    #print("CIV_Action: Entry - cmd_num = %x data_len= %d  cmd = %X  data_start_idx = %d  data_len = %d  rd_buffer:%X %X %X %X %X %X %X %X %x %X %X", cmd_num, data_len, cmd_List[cmd_num][2],
    #         data_start_idx, data_len, rd_buffer[0], rd_buffer[1], rd_buffer[2], rd_buffer[3], rd_buffer[4], rd_buffer[5], rd_buffer[6],
    #         rd_buffer[7],rd_buffer[8], rd_buffer[9], rd_buffer[10], radio_address)
  
    match (cmd_num):
  
        case cmds.CIV_C_F25A.value | cmds.CIV_C_F25B.value | cmds.CIV_C_F_READ.value | cmds.CIV_C_F_SEND.value | cmds.CIV_C_F1_SEND.value:
            if  ((data_len == 5 or (radio_address == IC905 and data_len == 6)) and (rd_buffer[4] == 0 or rd_buffer[4] == 0 or rd_buffer[4] == 5 or rd_buffer[4] == 0x25)):
                    mul = 1
                    f = 0
                    for i in (n + data_start_idx  for n in range(data_start_idx + data_len)): 
                    #for (i = data_start_idx; i < data_start_idx + data_len; i++):
                        if (rd_buffer[i] == 0xfd):
                            continue  #spike
                        f += (rd_buffer[i] & 0x0f) * mul
                        mul *= 10  # * decMulti[i * 2 + 1];
                        f += (rd_buffer[i] >> 4) * mul
                        mul *= 10  #  * decMulti[i * 2];
                    print("CIV_Action:  Freq:", f);
                    #read_Frequency(f, data_len);
                    bd.frequency(f)

        case _: bd.case_default()     # anything we have not seen yet comes to here


    def case_default(self):
        #self.hexdump(self.payload_copy)
        #__payload_len = len(self.payload_copy)
        #print("(case_default) Unknown message,ID:0x"+format(self.payload_ID,'04x')+"  Length:", __payload_len, flush=True)
        #return "no match found"
        print("default")


#  __________________________________________________________________
#
#   Packet Capture and Filtering
#  __________________________________________________________________
#

def Get_Radio_address():
    retry_Count = 0
    global init_done
    global radio_address_received
    global radio_address
    global ALL_RADIOS
    global IC705
    global IC905
    global IC9700
    global RADIO_ADDR
    
    if (radio_address == 0x00 or radio_address == 0xff or radio_address == 0xe0):
        if (radio_address_received == 0 or radio_address_received == 0xe0):
                print("Get_Radio_address: Radio not found - retry count = ", retry_Count)
                #vTaskDelay(100)
                retry_Count += 1
        else:
                radio_address = radio_address_received
                print("Get_Radio_address: Radio found at ", hex(radio_address))
                retry_Count = 0
                init_done = 1
    
    if (radio_address == IC705):
        # copy the Bands_705 structure content to the Bands structure
        #memcpy(bands, bands_705, sizeof(bands));
        pass

    if (radio_address == IC905 or radio_address == IC9700):
        # for all other radios (at least the 905 and 9700)
        # copy the Bands_905 structure content to the Bands structure
        #memcpy(bands, bands_905, sizeof(bands));
        pass
            
    return retry_Count


def remove():
    global RING_SIZE
    global ring_head
    global ring_tail
    global ring_data

    if (ring_head != ring_tail):
        c = ring_data[ring_tail]
        ring_tail = (ring_tail + 1) % RING_SIZE
        return c
    else:
        return -1

def parse_packet(payload):
    global START_BYTE
    global STOP_BYTE
    global radio_address_received
    global radio_address
    global cmds
    global cmd_list
    
    cmd_num = 255
    match = 0
    data_start_idx = 0
    read_buffer = [0] * 40
    c = 0
    j = 0
    i = 0
    
    # Extract the data from the ring buffer
    while c != -1:
        c = remove()
        if c != -1:
            if (c != 0xFD):
                read_buffer[j] = int(c)
                j += 1
            else:
                read_buffer[j] = int(c)
                j += 1
                break  # end of message, we are done for now.  IRQ will only call us for 1 message at a time
    
    # Now parse the string
    data_len = j;
    #print("String copied, now parse it")

    if (data_len):
        msg_len = data_len
        
        #print("processCatMessages", read_buffer, data_len)
        
        cmd_num = 255
        match = 0
        
        if (msg_len > 0):
            #print("processCatMessages <++ Rx Raw Msg: ")
            #for k in range(msg_len):
            #    print(read_buffer[k])
            #print("msg_len = %d END\n", msg_len)
            
            if (read_buffer[0] == (START_BYTE) and read_buffer[1] == (START_BYTE)):
                radio_address_received = read_buffer[3]
                #print("radio_address_received = ", radio_address_received)
                if (radio_address_received != 0 and radio_address == 0):
                    Get_Radio_address()
                ##if (read_buffer[3] == radio_address) {
                if (read_buffer[3] != 0):
                    #if (read_buffer[2] == CONTROLLER_ADDRESS || read_buffer[2] == BROADCAST_ADDRESS)
                    for cmd_num in range(cmds.End_of_Cmd_List.value):  # loop through the command list structure looking for a pattern match        
                        #print("processCatMessages: list index = ", cmd_num, "cmd byte len = ", cmd_List[cmd_num][1], "cmd byte 1 = ", cmd_List[cmd_num][2])            
                        for i in (n + 1 for n in range(cmd_List[cmd_num][1])):   #Break out if no match. Make it to the bottom and you have a match
                            #print("processCatMessages: byte index =",i,"  cmd_num=", cmd_num, "from radio, current byte from radio = ",read_buffer[3+i],"next byte=0x",read_buffer[3+i+1],"on remote length=",cmd_List[cmd_num][1]," and cmd=",hex(cmd_List[cmd_num][2]))
                            #print("Cmd len = ", cmd_List[cmd_num][1], "Cmd Byte 1 = ", cmd_List[cmd_num][1], "readbuff 4", read_buffer[3+i])
                            if ((cmd_List[cmd_num][1+i]) != read_buffer[3+i]):  #and cmd_List[cmd_num][1] > 1 ):
                                #print("processCatMessages: Skip this one - Matched 1 element: look at next field, if any left. CMD Body Length = ", cmd_List[cmd_num][1], "  CMD(i)  = ", hex(cmd_List[cmd_num][i+1]), "  next RX byte = ", read_buffer[3+i+1])
                                match = 0
                                break
                            
                            match += 1
                            #print("processCatMessages: Possible Match: Len = ", cmd_List[cmd_num][1], "CMD1 = ", read_buffer[4], "  CMD2  = ", read_buffer[5], " Data1/Term  = ",read_buffer[6])

                        #if (read_buffer[3+i] == hex(STOP_BYTE))  # if the next byte is not a stop byte then it is the next cmd byte or maybe a data byte, depends on cmd length
       
                        if (match != 0 and (match == cmd_List[cmd_num][1])):
                            #print("processCatMessages: FOUND MATCH - Cmd_num= ", cmd_num, "  Len = ", cmd_List[cmd_num][1], "CMD1 = ", read_buffer[4], "  CMD2  = ", read_buffer[5], " Data1/Term  = ",read_buffer[6], "msg Length = ", data_len)
                            break
                    
                    data_start_idx = 4 + cmd_List[cmd_num][1]
                    data_len = msg_len - data_start_idx - 1
                    if (data_len < 0):
                        data_len = 0
                    
                    #print("processCatMessages: cmd = ",cmd_num, "  cmd len = ", cmd_List[cmd_num][1], "  CMD1 Byte = ", hex(cmd_List[cmd_num][2]), "  data_start_idx = ", data_start_idx,"  data_len = ", data_len)

                    if (cmd_num >= cmds.End_of_Cmd_List.value):
                        print("processCatMessages: No message match found - cmd_num = %d  read_buffer[4 & 5] = %X %X", cmd_num, read_buffer[4], read_buffer[5])
                        print("processCatMessages:", hex(read_buffer))
                    else:
                        print("processCatMessages: Call CIV_Action CMD = ", cmd_num)
                        CIV_Action(cmd_num, data_start_idx, data_len, msg_len, read_buffer)
        
    #print(read_buffer)
    #bd.hexdump(payload)
    #payload_len = len(payload)
    #print("Payload Length = ", payload_len)
    #if (i > 6):
    #    print("call mh")
        #mh.switch_case(read_buffer, i)  # this extracts and routes messages to functions

    
# -------------------------------------------------------------------

# Ring buffers for Serial RX
def add(c):
    global RING_SIZE
    global ring_head
    global ring_tail
    global ring_data
    global next_head

    next_head = (ring_head + 1) % RING_SIZE
    if (next_head != ring_tail):
        # there is room
        ring_data[ring_head] = int(c)
        ring_head = next_head
        return 0
    else:
        print("Err: No room left in RX queue")
        # no room left in the buffer
        return -1   
    


def serial_sniffer(args):
    #initialization and open the port

    #possible timeout values:
    #    1. None: wait forever, block call
    #    2. 0: non-blocking mode, return immediately
    #    3. x, x is bigger than 0, float allowed, timeout block call

    ser = serial.Serial()
    ser.port = "rig-pty1"
    ser.baudrate = 19200
    ser.bytesize = serial.EIGHTBITS #number of bits per bytes
    ser.parity = serial.PARITY_NONE #set parity check: no parity
    ser.stopbits = serial.STOPBITS_ONE #number of stop bits
    #ser.timeout = None          #block read
    ser.timeout = 0           #non-block read
    #ser.timeout = 2              #timeout block read
    ser.xonxoff = False     #disable software flow control
    ser.rtscts = False     #disable hardware (RTS/CTS) flow control
    ser.dsrdtr = False       #disable hardware (DSR/DTR) flow control
    ser.writeTimeout = 2     #timeout for write
    
    print("Starting Program")
    try:
        try: 
            ser.open()
        except Exception (e):
            print ("error open serial port: " + str(e))
            exit()

        if ser.isOpen():
            
            ser.flushInput() #flush input buffer, discarding all its contents
            ser.flushOutput()#flush output buffer, aborting current output 
                        #and discard all that is in buffer
            #write data
            #ser.write("AT+CSQ")
            #print("write data: AT+CSQ")
            #time.sleep(0.5)  #give the serial port sometime to receive the data
            
            while (1):
                response = ser.readline()
                if response != b'':
                    #print("read data: ", response, flush=True)
                    for c in response:
                        add(c)    ## usb loop task will pull this data out
                        if (c == 0xFD):  ## end of a complete message, process it.
                            parse_packet(response)
                            #print("EOF")
                            # call to processing function here

        else:
            print("cannot open serial port ")

    except Exception (e1):
        print("error communicating...: " + str(e1))

    except KeyboardInterrupt:
        bd.write_split(bd.split_status)
        bd.wcrite_band(bd.vfoa_band)
        print('Done')
        dht.stop()
        #dc.stop()

    finally:
        GPIO.cleanup()
        ser.close()
        sys.exit()


if __name__ == '__main__':
    #import sys
    key_value_pairs = {}
    io = OutputHandler()  # instantiate our classes
    bd = BandDecoder()
    print("CI-V Serial Band Decoder - K7MDL Mar 2025")
    tim = dtime.now()
    print("Startup at", tim.strftime("%m/%d/%Y %H:%M:%S%Z"), flush=True)

    split_file = os.path.expanduser("~/.Decoder.split")  # saved state for last known split status
    if not os.path.exists(split_file):
        bd.write_split(0)
    radio_split = read_config(split_file)
    bd.read_split(radio_split)

    band_file = os.path.expanduser("~/.Decoder.band")    # last known band value
    if not os.path.exists(band_file):
        bd.write_band("2M")
    radio_band = read_config(band_file)
    bd.read_band(radio_band)

    # read in config, split and band files
    config_file = os.path.expanduser("~/Decoder.config")
    if os.path.exists(config_file):
        key_value_pairs = read_config(config_file)

    bd.init_band(key_value_pairs)

    # Update the temperature log
    bd.write_temps("Program Startup\n")

    dht = RepeatedTimer(dht11_poll_time, bd.temps)
    print("DHT11 enabled:", dht11_enable, "  DHT11 Poll time:",dht11_poll_time, flush=True)
    
    # Start the main program
    #dc = DecoderThread(serial_sniffer(sys.argv))   # option to run main program in a thread
    serial_sniffer(sys.argv)
    
    #  Program never returns here
    io = None
    bd = None
    mh = None






# file: CIV.py

from enum import Enum, auto

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

Freq_table_905 = { 
                 '0': {
                      'bandname':'2M',
                    'lower_edge':144000000,
                    'upper_edge':148000000,
                        'offset':0,
                          'band':0b00000000,
                           'ptt':0b00000001,
                 },
                '1': {
                      'bandname':'70cm',
                    'lower_edge':430000000,
                    'upper_edge':450000000,
                        'offset':0,
                          'band':0b00000001,
                           'ptt':0b00000001,
                 },
                '2': {
                      'bandname':'23cm',
                    'lower_edge':1240000000,
                    'upper_edge':1300000000,
                        'offset':0,
                          'band':0b00000010,
                           'ptt':0b00000001,
                 },
                '3': {
                      'bandname':'13cm',
                    'lower_edge':2300000000,
                    'upper_edge':2550000000,
                        'offset':0,
                          'band':0b00000011,
                           'ptt':0b00000001,
                 },
                '4': {
                      'bandname':'6cm',
                    'lower_edge':5600000000,
                    'upper_edge':5800000000,
                        'offset':0,
                          'band':0b00000100,
                           'ptt':0b00000001,
                 },
                '5': {
                      'bandname':'3cm',
                    'lower_edge':10000000000,    # 10000,000.000 = 1,389,000,000
                    'upper_edge':10500000000,
                        'offset':0,    #10.0G - 1.389G = 8,611,000,000
                          'band':0b00000101,
                           'ptt':0b00000001,
                }
            }

Freq_table_705 = { 
                 '0': {
                      'bandname':'LF',
                    'lower_edge':1,
                    'upper_edge':1800000,
                        'offset':0,
                          'band':0b00000000,
                           'ptt':0b00000001,
                 },
                '1': {
                      'bandname':'HF',
                    'lower_edge':1800000,
                    'upper_edge':50000000,
                        'offset':0,
                          'band':0b00000001,
                           'ptt':0b00000001,
                 },
                '2': {
                      'bandname':'6M',
                    'lower_edge':50000000,
                    'upper_edge':54000000,
                        'offset':0,
                          'band':0b00000010,
                           'ptt':0b00000001,
                 },
                '3': {
                      'bandname':'FMAIR',
                    'lower_edge':54000000,
                    'upper_edge':144000000,
                        'offset':0,
                          'band':0b00000011,
                           'ptt':0b00000001,
                 },
                '4': {
                      'bandname':'2M',
                    'lower_edge':144000000,
                    'upper_edge':200000000,
                        'offset':0,
                          'band':0b00000100,
                           'ptt':0b00000001,
                 },
                '5': {
                      'bandname':'70cm',
                    'lower_edge':400000000,    
                    'upper_edge':470000000,
                        'offset':0,    
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

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
import copy
from CIV import *
import math

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
use_wired_PTT = 0  # 0 = poll for TX status, 1 use GPIO input
PTT = 0
TX_last = 0
p_longitude = 0.0
p_latitude = 0.0   #used with grid sqaure calc
Longitude = None
Latitude = None
Grid_Square = None
hr_off = None
min_off = None
shift_dir = None
UTC = 0  # 0 = local, 1 = UTC time format
MODES_NUM = 16
mode_idx = 0
datamode = 0
filt = 0
mode_idx = 0

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
        for __band_num in Freq_table:
            if (__band_num == band):
                __band_pattern = Freq_table[__band_num]['ptt']
                __bandname = Freq_table[__band_num]['bandname']
                # Found a band match, set ptt io pin(s)
                if ptt:
                    p = bd.colored(255,0,0,"(+TX++)")
                else:
                    p = bd.colored(45,255,95,"(-RX--)")

                b = bd.colored(255,235,145, format(str(__bandname),"5"))
                bp = bd.colored(0,255,255, format(__band_pattern,'06b'))
                print(p,"Output for "+b+" Pattern:"+bp, flush=True)

                if (ptt):
                    ptt = 0xff
                else:
                    ptt = 0x00

                for __pins in IO_table:
                    pin_invert = IO_table[__pins]['ptt_invert']
                    io_pin     = IO_table[__pins]['ptt_pin']
                    pin_state  = (__band_pattern & __pins & ptt)

                    pin_state = bool(pin_state)   # convert decimal number to a boolean value

                    if pin_invert:
                        pin_state = pin_state ^ 1 # invert the pin
                        #print("pin state after inversion:", int(pin_state))

                    #print("index", __pins, "pin state:", pin_state,"on",io_pin, "inverted", pin_invert)

                    GPIO.output(io_pin, pin_state)  # set our pin


    def band_io_output(self, band):
        # turn on selected band output
        for __band_num in Freq_table:
            if (__band_num == band):
                __band_pattern = Freq_table[__band_num]['band']
                __bandname = Freq_table[__band_num]['bandname']
                # Found a band match, now loop through the set of IO pins
                t = bd.colored(235,110,200, "(BAND )")
                b = bd.colored(255,225,145, format(str(__bandname),"5"))
                p = bd.colored(0,255,255, format(__band_pattern,'06b'))
                print(t,"Output for "+b+" Pattern:"+p, flush=True)
                template = 0x0000
                
                for __pins in IO_table:
                    pin_invert = IO_table[__pins]['band_invert']
                    io_pin     = IO_table[__pins]['band_pin']
                    pin_state  = (__band_pattern & __pins)

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
    def __init__(self, interval, function, *args, **kwargs):
        #super(RepeatedTimer, self).__init__()
        self._timer = None
        self.interval = interval
        self.function = function
        self.args = args
        self.kwargs = kwargs
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
        if self._timer:
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
    bandname = ""
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
    in_menu = 0
    payload_len = 0
    payload_copy = ""
    payload_ID = 0
    payload_ID_byte = 0
    payload_Attrib_byte = 0
    frequency_init = 0
    global MODES_NUM
    global ModeStr
    global datamode
    global FilStr
    global filt
    global modeList
    global mode_idx

    def __init__(self):
        self.__offset = 0
        self.__freq_lastA = 0
        self.__freq_lastB = 0
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
        __band_num = key_value_pairs['RADIO_BAND']
        __offset = Freq_table[__band_num]['offset']
        self.bandname = Freq_table[__band_num]['bandname']
        self.vfoa_band = self.vfob_band = self.vfoa_band_split_Tx = __band_num
        self.selected_vfo = self.unselected_vfo = self.selected_vfo_split_Tx = Freq_table[__band_num]['lower_edge'] + __offset +1
        self.ptt_state = 0


    def read_model(self, key_value_pairs):
        global Freq_table
        # Initialize the Freq Table for a chosen radio model.
        __radio_model = key_value_pairs['RADIO_MODEL']
        if __radio_model == 'IC705':
            Freq_table = copy.copy(Freq_table_705)
        else:
            Freq_table = copy.copy(Freq_table_905)
        print("Radio Model is 905", __radio_model)


    def read_patterns(self, key_value_pairs):
        Freq_table['0']['band'] = band = int(key_value_pairs['BAND_0'],base=16)
        Freq_table['0']['ptt'] = ptt = int(key_value_pairs['PTT_0'],base=16)
        print("Band 0 BAND:", band, "PTT:", hex(ptt))

        Freq_table['1']['band'] = band = int(key_value_pairs['BAND_1'],base=16)
        Freq_table['1']['ptt'] = ptt = int(key_value_pairs['PTT_1'],base=16)
        print("Band 1 BAND:", band, "PTT:", hex(ptt))

        Freq_table['2']['band'] = band = int(key_value_pairs['BAND_2'],base=16)
        Freq_table['2']['ptt'] = ptt = int(key_value_pairs['PTT_2'],base=16)
        print("Band 2 BAND:", band, "PTT:", hex(ptt))

        Freq_table['3']['band'] = band = int(key_value_pairs['BAND_3'],base=16)
        Freq_table['3']['ptt'] = ptt = int(key_value_pairs['PTT_3'],base=16)
        print("Band 3 BAND:", band, "PTT:", hex(ptt))

        Freq_table['4']['band'] = band = int(key_value_pairs['BAND_4'],base=16)
        Freq_table['4']['ptt'] = ptt = int(key_value_pairs['PTT_4'],base=16)
        print("Band 4 BAND:", band, "PTT:", hex(ptt))

        Freq_table['5']['band'] = band = int(key_value_pairs['BAND_5'],base=16)
        Freq_table['5']['ptt'] = ptt = int(key_value_pairs['PTT_5'],base=16)
        print("Band 5 BAND:", band, "PTT:", hex(ptt))


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
        self.read_model(key_value_pairs)
        self.read_DHT(key_value_pairs)
        self.read_patterns(key_value_pairs)
        self.read_band_pins(key_value_pairs)
        self.read_ptt_pins(key_value_pairs)
        io.gpio_config()
        self.vfoa_band = self.frequency(self.selected_vfo, self.unselected_vfo)  # 0 is vfoa
        self.ptt(PTT)


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
        temp_str = (tim.strftime("%m/%d/%Y %H:%M:%S%Z")+"  Temperature: %(f)0.1f°F  %(t)0.1f°C  Humidity: %(h)0.1f%%  CPU: %(c)s°C  DHT11:%(e)d" % {"t": temp, "h": hum, "f": temp_F, 'c': cpu, 'e': int(dht11_enable)})
        print(self.colored(100,120,255,"(TEMP )"), temp_str, flush=True)
        self.write_temps(temp_str+"\n")


    def p_status(self, TAG):
        global TempC
        global TempF
        global Humidity

        cpu = self.get_cpu_temp()
        #tim = dtime.now()
        
        if self.split_status == 17:
            splits = 'D'  # duplex
        elif self.split_status == 1:
            splits = '1'  # split on
        else: 
            splits = '0'   # split off
            
        print(self.colored(175,210,240,"("+TAG+")"),
            #tim.strftime("%m/%d/%y %H:%M:%S%Z"),
            " VFOA Band:"+self.colored(255,225,165,format(self.bandname,"4")),
            " A:"+self.colored(255,255,255,format(self.selected_vfo, "11")),
            " B:"+self.colored(215,215,215,format(self.unselected_vfo, "11")),
            " Split:"+self.colored(225,255,90,splits),
            " M:"+modeList[mode_idx][1],
            " F:"+FilStr[filt],
            " D:"+format(ModeStr[datamode], "10"),
            " P:"+format(self.preamp_status, "1"),
            " A:"+format(self.atten_status, "1"),
            " PTT:"+self.colored(115,195,110,format(self.ptt_state, "1")),
            #" Menu:"+format(self.in_menu, "1"),   #  this toggles 0/1 when in menus, and/or when there is spectrum flowing not sure which
            #" Src:0x"+format(self.payload_ID, "04x"),
            #" T:%(t)0.1f°F  H:%(h)0.1f%%  CPU:%(c)s°C" % {"t": TempF, "h": Humidity, "c": cpu}, # sub in 'temp' for deg C
            flush=True)


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


    def frequency(self, freqA, freqB):  # 0xd8 0x00 is normal tuning update, 0x18 on band changes
        # Duplex used split status byte for VFO swap, just has vfoB set different with offset
        # split is updated in message ID 0xD4.  Here we can also pick it up and not wait for 
        # someone to press the split button to generate the d4 event.
        # Returns the payload hex converted to int.
        # This need to have the band offset applied next
        __vfoa = freqA
        #print("(Freq) VFO A = ", __vfoa)
        __vfob = freqB
         #print("(Freq) VFO B = ", __vfob)

        if (radio_address == IC905 and (self.vfoa_band == 3 or self.vfoa_band == 4 or self.vfoa_band == 5)):
            self.atten_status = 0
            self.preamp_status = 0

        if (self.split_status != self.__split_status_last):
            self.write_split(self.split_status)
            self.__split_status_last = self.split_status

        # Look for band changes
        if (__vfoa != self.__freq_lastA or __vfob != self.__freq_lastB):
            # Search the Freq_table to see what band these values lie in
            for __band_num in Freq_table:
                if (__vfoa >= Freq_table[__band_num]['lower_edge'] and
                    __vfoa <= Freq_table[__band_num]['upper_edge'] ):
                    # Found a band match, print out the goods
                    self.__offset = Freq_table[__band_num]['offset']
                    self.selected_vfo = __vfoa + self.__offset
                    self.vfoa_band = __band_num
                    self.bandname = Freq_table[__band_num]['bandname']

                if (__vfob >= Freq_table[__band_num]['lower_edge'] and
                    __vfob <= Freq_table[__band_num]['upper_edge'] ):
                    # Found a band match, print out the goods
                    self.__offset = Freq_table[__band_num]['offset']
                    self.unselected_vfo = __vfob + self.__offset
                    self.vfob_band = __band_num

            self.p_status("FREQ ") # print out our state
            #print("  Lower edge = ", Freq_table[self.vfoa_band]['lower_edge'] + self.__offset,
            #      "  Upper edge = ", Freq_table[self.vfoa_band]['upper_edge'] + self.__offset,
            #      "  Offset = ", self.__offset,
            #      "  Bandname = ", self.bandname)

            #  set band outputs on band changes
            if (self.vfoa_band != self.__vfoa_band_last):
                io.band_io_output(self.vfoa_band)
                self.__vfoa_band_last = self.vfoa_band
                self.write_band(self.vfoa_band)  # update the status file on change
                #self.bandname = Freq_table[self.vfoa_band]['bandname']
            self.__freq_lastA = __vfoa
            self.__freq_lastB = __vfob
        else:
            #self.p_status("FREQ ") # print out our state
            pass

        return self.vfoa_band


    # PTT sequence
    def ptt(self, PTT):
        # watch for PTT value changes
        if (self.vfoa_band != ""):   # block PTT until we know what band we are on
            #print("PTT called")    
            self.ptt_state = PTT
            #self.__ptt_state_last = 255

            if (self.ptt_state != self.__ptt_state_last):
                #print("PTT state =", self.ptt_state)
                if (self.ptt_state == 1):  # do not TX if the band is still unknown (such as at startup)
                    #print("PTT TX VFO A Band = ", self.bandname, "VFO B Band = ", self.vfob_band, "  ptt_state is TX ", self.ptt_state, " Msg ID ", hex(self.payload_copy[0x0001]))
                    if (self.split_status == 1): # swap selected and unselected when split is on during TX
                        self.vfoa_band_split_Tx = self.vfoa_band  # back up the original VFOa band
                        self.selected_vfo_split_Tx = self.selected_vfo  # back up original VFOa
                        self.selected_vfo = self.unselected_vfo  # during TX assign b to a
                        self.vfoa_band = self.vfob_band
                        #print("PTT TX1 VFO A Band = ", self.bandname, "VFO B Band = ", self.vfob_band,  " ptt_state is TX ", self.ptt_state)

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
                    #print("PTT-RX VFO A Band =", self.bandname, "VFO B Band =", self.vfob_band, "VFOA BAND SPLIT TX =", self.vfoa_band_split_Tx, "SELECTED VFO SPLIT TX =", self.selected_vfo, " ptt_state is RX ", self.ptt_state) #, " Msg ID ", hex(self.payload_copy[0x0001]))
                    if (self.split_status == 1): # swap selected and unselected when split is on during TX
                        self.vfoa_band = self.vfoa_band_split_Tx
                        self.selected_vfo = self.selected_vfo_split_Tx
                        #print("PTT-RX1 VFO A Band = ", self.bandname, "VFO B Band = ", self.vfob_band,  " ptt_state is RX ", self.ptt_state)

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
        print(rd(buffer))
        print("(case_default) Unknown message,ID:",flush=True)
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

def setTime(hr, min, sec, mday, month, yr):  # modifed to match arduino TimeLib versionq}:
#void setTime(int yr, int month, int mday, int hr, int minute, int sec, int isDst)  // orignal example
    setenv("TZ", "UTC", 1)
    tzset()

    tm.tm_year = yr - 1900   # Set date
    tm.tm_mon = month-1
    tm.tm_mday = mday
    tm.tm_hour = hr      # Set time
    tm.tm_min = min
    tm.tm_sec = sec
    #tm.tm_isdst = isDst  # 1 or 0
    tm.tm_isdst = 0  # 1 or 0  // setting to 0 for UTC only use
    #t = mktime(&tm)
    t = mktime(tm)
    #ESP_LOGI(TAG, "Setting time: %s", asctime(&tm));
    #char *myDate0 = ctime(&t);
    #ESP_LOGI(TAG, "0-myDate: %s", myDate0);

    #
    # The algorithm is fairly straightforward. The scaling array provides divisors to divide up the space into the required number of sections,
    # which is 18 for for the field, 10 for the square, 24 for the subsquare, 10 for the extended square, then 24, then 10. 
    # The limit is 6 pairs which is 2 more than is used even in the most detailed versions (8 characters in all). 
    # The divisor is also used in the fmod function to narrow down the range to the next highest order square that we�re dividing up.
    # The scaling array could be precalculated, but I figure the optimizing compiler would take care of that. 
    # I also thought the values could be scaled up at the beginning of the function, then use integer arithmetic to do the conversion. 
    # It might be a bit faster.

    #To run it �./geo lat long�, e.g. �./geo 43.999 -79.495� which yields FN03gx09
    #  Parse the GPS NMEA ASCII GPGGA string for the time, latitude and longitude 
# ---------------------------------------------------------------------------------------------------------

def positionToMaidenhead(m):
    pairs=4
    scaling = [(360.0),
               (360.0/18.0),
               ((360.0/18.0)/10.0),
               (((360.0/18.0)/10.0)/24.0),
               ((((360.0/18.0)/10.0)/24.0)/10.0),
               (((((360.0/18.0)/10.0)/24.0)/10.0)/24.0),
               ((((((360.0/18.0)/10.0)/24.0)/10.0)/24.0)/10.0)]
    i = 0
    index = 0
    global p_longitude
    global p_latitude

    for i in range(pairs):
        index = math.floor(math.fmod((180.0+p_longitude), scaling[i])/scaling[i+1])
        if (i&1):
            m[i*2] = 0x30+index
        elif (i&2):
            m[i*2] = 0x61+index 
        else: 
            m[i*2] = 0x41+index
        
        index = math.floor(math.fmod((90.0+p_latitude), (scaling[i]/2))/(scaling[i+1]/2))
        if (i&1):
            m[i*2+1] = 0x30+index 
        elif (i&2):
            m[i*2+1] = 0x61+index 
        else:
            m[i*2+1] = 0x41+index
        
    m[pairs*2]=0
    return m  # success


#  Convert Alt and Lon to MH now */
def Convert_to_MH():
    global p_longitude
    global p_latitude
    global Longitude
    global Latitude
    global Grid_Square
    m = [0] * 9
    
    # if(GPS_Status == GPS_STATUS_LOCK_INVALID || msg_Complete == 0)   
    #     return 1;  /* if we are here with invalid data then exit.  LAt and LOnwill have text which cannot be computered of cour     */
    #  Get from GPS Serial input later
    p_latitude=float(Latitude)   # convert string to float
    p_longitude=float(Longitude)
    
    positionToMaidenhead(m)
    if m:
        Grid_Square = ''.join(chr(value) for value in m)
        #print("Grid_Square", Grid_Square)
        return 1  # Success
    else:
        Grid_Square = "Invalid"
        return 0  # fail -  Can use later to skip displaying anything when have invalid or no GPS input  


def bcdByte(x):
    f = 0
    mul = 1
    f += (x & 0x0f) * mul
    mul *= 10
    f += (x >> 4) * mul
    mul *= 10
    return f        
    #y = (((x & 0xf0) >> 4) * 10) + (x & 0x0f)
    #return y


def case_default(self, cmd, buffer):
    return
    #print("CI-V Command ID:", cmd, buffer)
    #self.hexdump(buffer)
    #print("default")
    pass


def CIV_Action(cmd_num:int, data_start_idx:int, data_len:int, msg_len:int, rd_buffer):
    global cmds
    global radio_address
    global ALL_RADIOS
    global IC705
    global IC905
    global IC9700
    global RADIO_ADDR
    global use_wired_PTT
    global TX_last
    global Longitude
    global Latitude
    global Grid_Square
    global p_longitude
    global p_latitude
    global hr_off
    global min_off
    global shift_dir
    global UTC
    global mode_idx
    global filt
    global MODES_NUM
    global datamode
    global modeList
    global ModeStr
    global FilStr
    
    #print("CIV_Action: Entry - cmd_num = %x data_len= %d  cmd = %X  data_start_idx = %d  data_len = %d  rd_buffer:%X %X %X %X %X %X %X %X %x %X %X", cmd_num, data_len, cmd_List[cmd_num][2],
    #         data_start_idx, data_len, rd_buffer[0], rd_buffer[1], rd_buffer[2], rd_buffer[3], rd_buffer[4], rd_buffer[5], rd_buffer[6],
    #         rd_buffer[7],rd_buffer[8], rd_buffer[9], rd_buffer[10], radio_address)

    match (cmd_num):

        case cmds.CIV_C_F25A.value | cmds.CIV_C_F25B.value | cmds.CIV_C_F_READ.value | cmds.CIV_C_F_SEND.value | cmds.CIV_C_F1_SEND.value:
            if  ((data_len == 5 or (radio_address == IC905 and data_len == 6)) and (rd_buffer[4] == 0 or rd_buffer[4] == 3 or rd_buffer[4] == 5 or rd_buffer[4] == 0x25)):
                #print("cmd:%d  message:%s" % (cmd_num, [hex(num) for num in rd_buffer][0:msg_len]))   #self.case_default(cmd_num, rd_buffer)     # anything we have not seen yet comes to here
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
                #print("CIV_Action:  Freq:", f, flush = True);
                #read_Frequency(f, data_len);
                if rd_buffer[5] == 0:
                    bd.selected_vfo = f
                if rd_buffer[4] == 0x25 and rd_buffer[5] == 1:
                    bd.unselected_vfo = f
                    #print("VFOB", bd.unselected_vfo)
                bd.frequency(bd.selected_vfo, bd.unselected_vfo)   #  0 is vfoa, 1 is vfo b
        
        case cmds.CIV_C_TX.value:  # Used to request RX TX status from radio
            #if (1):
            if not (data_len != 1 or (not(rd_buffer[4] == 0x1C or rd_buffer[5] == 0x00))):                
                if rd_buffer[6] == 1:
                    PTT = True 
                else: 
                    PTT = False
                #print("PTT=", PTT)
                if (TX_last != PTT):
                    if not use_wired_PTT:        # normally the wired input will pass thru the PTT from radio hardware PTT. 
                        #PTT_Output(band, PTT);    # If that is not available, then use the radio polled TX state .
                        #print("CIV_Action: TX Status = %d" % (PTT), flush=True)
                        TX_last = PTT
                        bd.ptt(PTT)
                        # Call PTT output here rather than in teh main loop to avoid any loop delay time.

        case cmds.CIV_C_SPLIT_READ.value:
            bd.split_status = rd_buffer[data_start_idx]
            bd.write_split(bd.split_status)
            #print("CIV_Action: CI-V Returned Split status:", bd.split_status);     

        case cmds.CIV_C_PREAMP_READ.value:
            bd.preamp_status = rd_buffer[data_start_idx]
            #print("CIV_Action: CI-V Returned Preamp status:", bd.preamp_status);
            #displayAttn()
            #displayPreamp()        
                
        case cmds.CIV_C_ATTN_READ.value:
            bd.atten_status = rd_buffer[data_start_idx]
            #print("CIV_Action: CI-V Returned Attenuator status:", bd.atten_status);
            #displayAttn()
            #displayPreamp()

        case cmds.CIV_C_MOD_READ.value | cmds.CIV_C_MOD1_SEND.value | cmds.CIV_C_MOD_SEND.value:
            # command CIV_C_MODE_READ received
            #print("Mode Read: cmd:%d  message:%s" % (cmd_num, [hex(num) for num in rd_buffer][0:msg_len]), flush=True) 
            # get extended mode since this does not update data mode.  It does occur on band changes which has uses            
            ser.write(sendCatRequest(cmds.CIV_C_F26A.value, 0, 0))   # mode, filter, datamode
            time.sleep(0.1)  #give the serial port sometime to receive the data
            read_port()
            return

            #  do not need the below, just use extended mode
            radio_mode = rd_buffer[data_start_idx]
            # look up the bcd value in our modelist table to see what radio mode it is 
            for i in range(MODES_NUM):
                if (modeList[i][0] == radio_mode):  # match bcd value to table mode_num value to get out mode index that we store
                    mode_idx = i  # now know our decimal index
                    break
            filt = rd_buffer[data_start_idx+1]
            print("CIV_Action: ModeNum: %d   Mode: %s   DATA: %s   Filt: %s" % (mode_idx, modeList[mode_idx][1], ModeStr[datamode], FilStr[filt]), flush=True)

        case cmds.CIV_C_UTC_READ_9700.value | cmds.CIV_C_UTC_READ_905.value | cmds.CIV_C_UTC_READ_705.value:
            #print("processCatMessages: UTC Offset, Len = %d" % (data_len))
            #               pos     sub cmd  len        1    2     3          term
            # FE.FE.E0.AC.  1A.05.  01.81.   3         07.  00.   01.         FD
            # FE.FE.E0.AC.  1A.05.           datalen   hr   min   1=minus 0=+
            #                                     offset time 00 00 to 14 00    Shift dir 00 + and 01 is -
            # when using datafield, add 1 to prog guide index to account for first byte used as length counter - so 3 is 4 here.
            hr_off = bcdByte(rd_buffer[data_start_idx+0])
            min_off = bcdByte(rd_buffer[data_start_idx+1])
            shift_dir = bcdByte(rd_buffer[data_start_idx+2])

            if (shift_dir):
                hr_off = hr_off * -1  # invert  - used by UTC set function
                min_off = min_off * -1 # invert  - used by UTC set function

            print("UTC Offset: %d:%d" % ( hr_off, min_off))

            # get current time and correct or set time zone offset
            #setTime(_hr,_min,_sec,_day,_month,_yr);  // apply offset in time set function

        case cmds.CIV_C_F26A.value | cmds.CIV_C_F26B.value | cmds.CIV_C_F26.value:  #case CIV_C_F26_SEND:
            #print("Extended Mode Read: cmd:%d  message:%s" % (cmd_num, [hex(num) for num in rd_buffer][0:msg_len])) 
            if (rd_buffer[data_start_idx] == 0x00):   # 0 is selected VFO field 
                # Store the mode, filt an datamode value per band      
                radio_mode = rd_buffer[data_start_idx+1]  # hex value
                if (rd_buffer[data_start_idx+2] == 1):
                    radio_data = 1 
                else:
                    radio_data = 0   # 0-1  limit to 0 or 1 answer
                if (rd_buffer[data_start_idx+3] < 4):
                    radio_filter = rd_buffer[data_start_idx+3]
                else:
                    radio_filter = 1;  # 1-3, make 1 if a bad value
                i = 0
                filt = radio_filter
                datamode = radio_data
                #print("CIV_Action: Extended Mode Info, Mode: %d  DataMode %d  Filt: %d" % (radio_mode, radio_data, radio_filter), flush=True)
                i = 0
                for i in range(MODES_NUM):
                    if (modeList[i][0] == radio_mode and modeList[i][3] == radio_data): # convert to our list index and also validate number received
                        mode_idx = i  # mode is in HEX
                        break  # found our match, exit loop          
                if (i >= MODES_NUM):   # if i reaches the top of the list then a match was not found, skip out
                    print("CIV_Action: ERROR: Extended Mode Info, Invalid Mode: %X" % (radio_mode), flush=True)
                else:  # we have a vaid match, finish up.
                    datamode = radio_data  # data on/off  0-1
                    filt = radio_filter    # filter setting 1-3
                    #print("CIV_Action: Extended Mode Info, ModeNum: %d   Mode: %s   DATA: %s   Filt: %s" % (mode_idx, modeList[mode_idx][1], ModeStr[datamode], FilStr[filt]), flush=True)
                bd.p_status("MODE ")
            # else the changes are for the unselected VFO

        case cmds.CIV_C_MY_POSIT_READ.value:
            #print("Process time, date, location data_len = ", data_len, rd_buffer)
            if (rd_buffer[data_start_idx] == 0xFF):
                print("***Error: Position/Time Not available, GPS connected? - Data = %X" % (rd_buffer[data_start_idx]), flush=True)
                return
            # if (data_len == 23) then altitude is invalid and not sent out, skip
            skip_altitude = False
            if (data_len == 23):
                skip_altitude = True   # skip 4 bytes when looking for time and date.
            elif (data_len != 27):
                return
                
            #print(" Position, Len = %d" % (data_len))            
            #               pos                  1  2  3  4   5     6  7  8  9  10   11       12 13 14 15   16 17   18 19 20   21 22  23  24  25 26 27  term
            # FE.FE.E0.AC.  23.00.  datalen 27  47.46.92.50.  01.   01.22.01.98.70.  00.      00.15.59.00.  01.05.  00.00.07.  20.24. 07. 20. 23.32.45. FD
            #                                   47,46,92,40,  01,   01,22,01,99,60,  00,                    00,58,  00,01,09,  20,24, 08, 28, 11,07,41  FD
            #                                                                                 00,15,64,00,  00,72,  00,00,20
            #                               lat 4746.9250   01(N)     12201.9870    00(-1) long  155.900m alt  105deg   0.7km/h   2024   07  20  23:32:45 UTC
            # when using datafield, add 1 to prog guide index to account for first byte used as length counter - so 27 is 28 here.
        
            # Extract and Process Lat and Long for Maidenhead Grid Square stored in global Grid_Square[]
            # char GPS_Msg[NMEA_MAX_LENGTH] = {}; // {"4746.92382,N,12201.98606,W\0"};
            # uint8_t i = 0;
            k = 0
            lati_deg = 0
            longi_deg = 0
            lati_min = 0
            longi_min = 0
            lati = 0
            longi = 0
            temp_str = ""

            # reformat latitude
            # 47.46.93.10.  01.  
            lati_deg  = bcdByte(rd_buffer[data_start_idx+k])         #  47 deg
            k += 1
            lati_min  = bcdByte(rd_buffer[data_start_idx+k])         #  46. min
            k += 1
            lati_min += bcdByte(rd_buffer[data_start_idx+k]) /100    #    .93
            k += 1
            lati_min += bcdByte(rd_buffer[data_start_idx+k]) /10000  #    .0010  last nibble is always 0
            k += 1
            lati_min /= 60
            lati = lati_deg+lati_min
            # if 1 then North, else south will be negative
            sign = bcdByte(rd_buffer[data_start_idx+k])  # use N/S to set neg or pos
            if not sign:
                Latitude = "-{:.4f}".format(lati)      #the ftoa function does not handle neg numbers */
            else:
                Latitude = " {:.4f}".format(lati)
            k += 1
            #print("CIV_Action: Latitude Converted to dd mm ss format: Deg=%f  min=%f  lat=%f  string=%s" % (lati_deg, lati_min, lati, Latitude))

            # Longitude  01.22.01.98.70.  00.
            longi_deg  = bcdByte(rd_buffer[data_start_idx+k])*100    # 100  first nibble is always 0
            k += 1
            longi_deg += bcdByte(rd_buffer[data_start_idx+k])        #  22 deg
            k += 1
            longi_min  = bcdByte(rd_buffer[data_start_idx+k])        #  01 min
            k += 1
            longi_min += bcdByte(rd_buffer[data_start_idx+k]) /100   #    .98
            k += 1
            longi_min += bcdByte(rd_buffer[data_start_idx+k]) /10000 #    .0070
            k += 1
            longi_min /= 60  # convert minutes to mm.mmm
            longi = longi_deg+longi_min  # make 1 number
            #if 1 then E, else W will be negative
            l_temp = bcdByte(rd_buffer[data_start_idx+k])
            if not l_temp:
                Longitude = "-{:.4f}".format(longi)
            else:
                Longitude = " {:.4f}".format(longi)
            k += 1
            #print("CIV_Action: Longitude Converted to dd mm ss format:  deg=%f   min=%f   long=%f  string=%s  %s" % (longi_deg, longi_min, longi, Latitude, Longitude))

            # if using NMEA string then proved a formnatted string like belw and convert to minutes  
            #strcpy(GPS_Msg, "4746.92382,N,12201.98606,W\0"};   // test string
            #ConvertToMinutes(GPS_Msg);       
            # Here I directly converted to what Convert_to_MH wants
            #print("Lat Long", Latitude, Longitude)
            Convert_to_MH();
            print("CIV_Action: GPS Converted: Lat = %s  Long = %s  Grid Square is %s" % (Latitude, Longitude, Grid_Square), flush=True)
            #//ESP_LOGI(TAG, "CIV_Action: ** Time from Radio is: ");

            _hr = _min = _sec = _month = _day = _yr = 1
            d_index = 0
            if (skip_altitude):
                d_index = 4

            _hr = bcdByte(rd_buffer[data_start_idx+24-d_index]) #Serial.print(_hr); ESP_LOGI(TAG, ":")
            _min = bcdByte(rd_buffer[data_start_idx+25-d_index]) #Serial.print(_min);ESP_LOGI(TAG, ":")
            _sec = bcdByte(rd_buffer[data_start_idx+26-d_index]) #Serial.print(_sec);ESP_LOGI(TAG, " ")
            
            _month = bcdByte(rd_buffer[data_start_idx+22-d_index]) #Serial.print(_month);ESP_LOGI(TAG, ".")
            _day = bcdByte(rd_buffer[data_start_idx+23-d_index]) #Serial.print(_day); ESP_LOGI(TAG, ".")
            _yr = bcdByte(rd_buffer[data_start_idx+21-d_index]) #Serial.print(_yr); // yr can be 4 or 2 digits  2024 or 24                    
            
            #print("Raw Time: %d:%d:%d  %d/%d/20%d UTC=%d" % (_hr,_min,_sec,_month,_day,_yr,UTC))
            #print("UTC=", UTC, hr_off, min_off)
            #if not UTC:
            #    #setTime(_hr+hr_off,_min+min_off,_sec,_day,_month,_yr)  # correct to local time                      
            #    print("Local Time: %d:%d:%d  %d/%d/20%d" % (32-_hr+hr_off,_min+min_off,_sec,_month,_day,_yr))
            #else:
            #setTime(_hr,_min,_sec,_day,_month,_yr)  # display UTC time
            print("UTC Time: %d:%d:%d  %d/%d/20%d" % (_hr,_min,_sec,_month,_day,_yr))

        case _ : print("Unhandled message cmd:%d  message:%s" % (cmd_num, [hex(num) for num in rd_buffer][0:msg_len]), flush=True)   #self.case_default(cmd_num, rd_buffer)     # anything we have not seen yet comes to here

#  __________________________________________________________________
#
#   Packet Capture and Filtering
#  __________________________________________________________________
#

# ----------------------------------------
#      Send CAT Request
# ----------------------------------------
def sendCatRequest(cmd_num, Data, Data_len):  # first byte in Data is length
    global START_BYTE
    global radio_address
    global CONTROLLER_ADDRESS
    global cmd_List
    buf_size = 50
    #print("sendCatRequest: Start Send")

    msg_len = 0
    req1 = [ START_BYTE, START_BYTE, radio_address, CONTROLLER_ADDRESS ]
    req2 = [0] * buf_size
    req = req1+req2

    for msg_len in (n + 1 for n in range(cmd_List[cmd_num][1])):     # copy in 1 or more command bytes
        req[msg_len + 3] = cmd_List[cmd_num][msg_len + 1]

    msg_len += 3  # Tee up to add data if any

    j = 0;
    if (Data_len != 0):              # copy in 1 or more data bytes, if any
        for j in Data_len:  #pick up with value i
            req[msg_len] = Data[j]
            msg_len += 1

    msg_len += 1
    req[msg_len ] = STOP_BYTE
    msg_len += 1
    req[msg_len] = 0  # null terminate for printing or conversion to String type

    #print("sendCatRequest --> Tx Raw Msg: %s," % (req))
    #print("sendCatRequest:  msg_len = %d  END" % (msg_len))

    if (msg_len < buf_size - 1):  # ensure our data is not longer than our buffer
        send_str = req[0:msg_len]
        #print("sendCatRequest: ***Send CI-V Msg: %s" % (send_str))
        loop_ct = 0
        #while (sending and loop_ct < 5):  # In case an overlapping send from IRQ call comes in, wait here until the first one is done.
        #    #vTaskDelay(pdMS_TO_TICKS(10));
        #    loop_ct += 1
        loop_ct = 0
        #print("Send_CatRequest: *** Send Cat Request Msg - result = %d  END TX MSG, msg_len = %d", ser.write(req)  #, msg_len+1, 1000), msg_len)
        #hex_array = [hex(num) for num in send_str][0:msg_len]
        #  This is the main debugging print
        #print("sendCatRequest: ***Send TX data = ", hex_array)
        #ser.write(send_str)
        #time.sleep(0.2)  #give the serial port sometime to receive the data
        #vTaskDelay(pdMS_TO_TICKS(10));
        return serial.to_bytes(send_str)
    else:
        print("sendCatRequest: Buffer overflow")


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
                print("Get_Radio_address: Radio found at ", hex(radio_address), flush = True)
                retry_Count = 0
                init_done = 1

    if (radio_address == IC705):
        # copy the Bands_705 structure content to the Bands structure
        #Freq_table.clear()
        Freq_table = copy.deepcopy(Freq_table_705)
        print("Activated IC-705 Freq Table")

    if (radio_address == IC905 or radio_address == IC9700):
        # for all other radios (at least the 905 and 9700)
        # copy the Bands_905 structure content to the Bands structure
        #Freq_table.clear()
        Freq_table = copy.deepcopy(Freq_table_905)
        print("Activated IC-905 Freq Table")

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

def processCatMessages():
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
            if (c != 0xfd):
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

        #print("processCatMessages %s  length: %d" % (read_buffer, data_len))

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
                #if (read_buffer[3] == radio_address) {
                if (read_buffer[3] != CONTROLLER_ADDRESS):
                    #hex_array = [hex(num) for num in read_buffer][0:msg_len]
                    #print("processCatMessages %s  length: %d" % (hex_array, data_len))
                    #if (read_buffer[2] != CONTROLLER_ADDRESS or read_buffer[2] == BROADCAST_ADDRESS):
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
                        #print("processCatMessages: Call CIV_Action CMD = ", cmd_num)
                        CIV_Action(cmd_num, data_start_idx, data_len, msg_len, read_buffer)

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


def read_port():
    global ser
    #try:
    response = ser.readline()
    if response != b'':
        #print("read data: ", response, flush=True)
        for c in response:
            add(c)    ## usb loop task will pull this data out
            if (c == 0xfd):  ## end of a complete message, process it.
                processCatMessages()
    #except Exception (t):
    #    print("port error", e)


loop_ctr = 0
def poll_radio(bypass):
    #try:  
    global loop_ctr
    global ser
    
    if loop_ctr == 60 or bypass:
        #ser.write(sendCatRequest(cmds.CIV_C_SCOPE_OFF.value, 0, 0))   # turn off scope data ouput to reduce bandwidth
        time.sleep(0.1)  #give the serial port sometime to receive the data
        read_port()
            
        #print("Get UTC offset")
        if radio_address == IC705:
            ser.write(sendCatRequest(cmds.CIV_C_UTC_READ_705.value, 0, 0))
        if radio_address == IC905:
            ser.write(sendCatRequest(cmds.CIV_C_UTC_READ_905.value, 0, 0))
        if radio_address == IC9700:
            ser.write(sendCatRequest(cmds.CIV_C_UTC_READ_9700.value, 0, 0))
        time.sleep(0.1)  #give the serial port sometime to receive the data
        read_port()
        ser.write(sendCatRequest(cmds.CIV_C_MY_POSIT_READ.value, 0, 0))
        time.sleep(0.1)  #give the serial port sometime to receive the data
        read_port()
        
        #ser.write(sendCatRequest(cmds.CIV_C_F26A.value, 0, 0))
        #time.sleep(0.1)  #give the serial port sometime to receive the data
        #read_port()
        
        #ser.write(sendCatRequest(cmds.CIV_C_MOD_SEND.value, 0, 0))
        #time.sleep(0.1)  #give the serial port sometime to receive the data
        #read_port()

    if loop_ctr == 20 or bypass:
        ser.write(sendCatRequest(cmds.CIV_C_PREAMP_READ.value, 0, 0))
        time.sleep(0.1)  #give the serial port sometime to receive the data
        read_port()
            
        ser.write(sendCatRequest(cmds.CIV_C_ATTN_READ.value, 0, 0))
        time.sleep(0.1)  #give the serial port sometime to receive the data
        read_port()
        
    if loop_ctr == 5 or bypass:
        # Add mode and filter here
        # also could use cmds.CIV_C_F26A.value | cmds.CIV_C_F26B.value
        #ser.write(sendCatRequest(cmds.CIV_C_F26A.value, 0, 0))   # mode, filter, datamode
        #time.sleep(0.1)  #give the serial port sometime to receive the data
        #read_port()
        
        ser.write(sendCatRequest(cmds.CIV_C_F25A.value, 0, 0))  # selected VFO freq
        time.sleep(0.1)  #give the serial port sometime to receive the data
        read_port()
        
        ser.write(sendCatRequest(cmds.CIV_C_F25B.value, 0, 0))  # unselcted vfo freq
        time.sleep(0.1)  #give the serial port sometime to receive the data
        read_port()
        
        ser.write(sendCatRequest(cmds.CIV_C_SPLIT_READ.value, 0, 0))   # split mode
        time.sleep(0.1)  #give the serial port sometime to receive the data
        read_port()

    
    ser.write(sendCatRequest(cmds.CIV_C_TX.value, 0, 0))
    time.sleep(0.1)  #give the serial port sometime to receive the data
    read_port()
        
    loop_ctr += 1
    if loop_ctr > 600:
        loop_ctr = 0
    
    #except Exception (e):
    #    print("error communicating...: " + str(e))


def serial_sniffer(args):
    global ser
    #initialization and open the port

    #possible timeout values:
    #    1. None: wait forever, block call
    #    2. 0: non-blocking mode, return immediately
    #    3. x, x is bigger than 0, float allowed, timeout block call
    ser.port = "/home/k7mdl/rig-pty1"
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
  
            ser.write(sendCatRequest(cmds.CIV_C_PREAMP_READ.value, 0, 0))
            time.sleep(0.1)  #give the serial port sometime to receive the data
            read_port()
            
            ser.write(sendCatRequest(cmds.CIV_C_ATTN_READ.value, 0, 0))
            time.sleep(0.1)  #give the serial port sometime to receive the data
            read_port()
            
            ser.write(sendCatRequest(cmds.CIV_C_SPLIT_READ.value, 0, 0))
            time.sleep(0.1)  #give the serial port sometime to receive the data
            read_port()
             
            ser.write(sendCatRequest(cmds.CIV_C_F26A.value, 0, 0))
            time.sleep(0.1)  #give the serial port sometime to receive the data
            read_port() 
            
            ser.write(sendCatRequest(cmds.CIV_C_F25A.value, 0, 0))
            time.sleep(0.1)  #give the serial port sometime to receive the data
            read_port()
            
            ser.write(sendCatRequest(cmds.CIV_C_F25B.value, 0, 0))
            time.sleep(0.1)  #give the serial port sometime to receive the data
            read_port()
                  
            #ser.write(sendCatRequest(cmds.CIV_C_SCOPE_OFF.value, 0, 0))   # turn off scope data ouput to reduce bandwidth
            time.sleep(0.1)  #give the serial port sometime to receive the data
            read_port()
            
            ser.write(sendCatRequest(cmds.CIV_C_F25A.value, 0, 0))
            time.sleep(0.1)  #give the serial port sometime to receive the data
            read_port()
            
            poll_radio(True) # iniitalize
            poll = RepeatedTimer(1, poll_radio, False)  # call every 1 sec
            print("Polling timer thread enabled:", flush=True)
            
            print("Main loop, waiting for RX events")
            while (1):
                #endCatRequest(cmds.CIV_C_SCOPE_OFF.value, 0, 0)
                time.sleep(0.2)  #give the serial port sometime to receive the data
                read_port()

        else:
            print("cannot open serial port ")

    except Exception (e1):
        print("error communicating...: " + str(e1))

    except KeyboardInterrupt:
        bd.write_split(bd.split_status)
        bd.wcrite_band(bd.vfoa_band)
        print('Done')
        dht.stop()
        poll_long.stop()
        poll_short.stop()
        poll_shorter.stop()
        poll_PTT.stop()
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
    Freq_table = Freq_table_905

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
        bd.write_band("0")
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
    
    ser = serial.Serial()
    
    # Start the main program
    dc = DecoderThread(serial_sniffer(sys.argv))   # option to run main program in a thread
    #serial_sniffer(sys.argv)

    #  Program never returns here
    io = None
    bd = None
    mh = None

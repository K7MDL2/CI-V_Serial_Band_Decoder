#!/usr/bin/python

#------------------------------------------------------------------
#
#  CI-V Serial Band Decoder for Raspberry Pi.  
#  Used with wfView virual sewrial port to act as a LAN to Serial bridge
#    permititng remote install of the Pi and its relays.
#
#   March 2025 K7MDL
#
#------------------------------------------------------------------

import serial, time, os, sys, traceback, copy, math
import numpy as np
from RPi import GPIO
import subprocess as sub
from threading import Timer
from typing import Callable
from datetime import datetime as dtime
from enum import Enum, auto
from CIV import *
from serial import SerialException

# These are usually not changed
main_TX = True  #  IC-9700 only - always use the main band for determining relay outputs - False will switch the band relays to the Sub band
use_wired_PTT = 0  # 0 = poll for TX status, 1 use GPIO input
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
RADIO_ADDR = ALL_RADIOS  # default ALL_RADIOS
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
active_band = 0  # 0 = main band 1 = sub on IC-9700
loop_ctr = 0
gpio_ptt_in_pin = 0
gpio_ptt_in_pin_invert = 0
key_value_pairs = {}
radio_model = 0
radio_band = 0
valid_address = False

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
            print("Band PTT"+format(i, '2d'), format(i, '06b'), "band_pin:", format(band_pin, '3'), " ptt_pin", ptt_pin)
            GPIO.setup(band_pin, GPIO.OUT, initial=band_invert)
            GPIO.setup(ptt_pin,  GPIO.OUT, initial=ptt_invert)
        GPIO.setup(gpio_ptt_in_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        print("GPIO pin mode setup complete", flush=True)


    def PTT_In(self):
        global PTT
        if use_wired_PTT:
            #ptt_state = GPIO.wait_for_edge(gpio_ptt_in_pin, BOTH, bouncetime=20, timeout=40)
            PTT = GPIO.input(gpio_ptt_in_pin)  # get ptt input state
            if gpio_ptt_in_pin_invert:
                PTT = PTT ^1   # invert if needed
                #print(" Inverted PTT Input:", PTT)
            else:
                #print("PTT Input: ", PTT)
                pass
            bd.ptt(PTT)


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

class BandDecoder(OutputHandler):
# We are inheriting from OutputHandler class so we can access its functions
#   and variables as if they were our own.
    bandname = ""
    
    # main band vfos
    vfoa_band = ""
    vfob_band = ""
    selected_vfo = 0
    unselected_vfo = 255
    CIV_selected_vfo = 0
    CIV_unselected_vfo = 0
    
    # sub band vfos
    selected_vfo_rx = 0
    CIV_selected_vfo_rx = 0
    vfoa_band_split_Tx = ""    
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
    TX_Delay = False
    global datamode
    global filt
    global mode_idx
    
    def __init__(self):
        self.__offset = 0
        self.__freq_lastA = 0
        self.__freq_lastB = 0
        self.__vfoa_band_last = 255
        self.__vfob_band_last = 255
        self.__ptt_state_last = 255
        self.PTT_hang_time = 0.2
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
        global Freq_table
        # Initialize the band and VFOs to the last saved values, lost likely will be correct.
        # Alternative is to do nothng and block PTT
        #__band_num = key_value_pairs['RADIO_BAND']
        __band_num = '0'
        __offset = Freq_table[__band_num]['offset']
        self.bandname = Freq_table[__band_num]['bandname']
        self.vfoa_band = self.vfob_band = self.vfoa_band_split_Tx = __band_num
        self.selected_vfo = self.unselected_vfo = self.selected_vfo_split_Tx = Freq_table[__band_num]['lower_edge'] + __offset +1
        self.ptt_state = 0
        print("Read_band = ", self.bandname, __band_num)


    def read_model(self, key_value_pairs, reload):
        global Freq_table
        global radio_address
        global radio_model
        #global radio_address_received
        
        # Initialize the Freq Table for a chosen radio model.
        rmkey = key_value_pairs['RADIO_MODEL']
        #print("Radio Model Key =", rmkey)
        if rmkey == 'IC705':
            radio_model = IC705
        if rmkey == 'IC905':
            radio_model = IC905
        if rmkey == 'IC9700':
            radio_model = IC9700
        if rmkey == "":
            radio_model = IC705
        if not reload:
            print("Radio Model = 0x%X" % (radio_model))

        print("Before: radio model:%X  radio address:%X" % (radio_model, radio_address))
        
        # Set to new model and data tables
        if radio_address == IC705:
            Freq_table = copy.deepcopy(Freq_table_705)
            print("Frequencies loaded for Radio Model 705 at address 0x%X" % (radio_address))
            radio_model = IC705
    
        if radio_address == IC905:
            Freq_table = copy.deepcopy(Freq_table_905)
            print("Frequencies loaded for Radio Model 905 at address 0x%X" % (radio_address))
            radio_model = IC905

        if radio_address == IC9700:
            Freq_table = copy.deepcopy(Freq_table_905)
            print("Frequencies loaded for Radio Model 9700 at address 0x%X" % (radio_address))
            radio_model = IC9700
        
        #print("After:  radio model:%X  radio address:%X" % (radio_model, radio_address))
        #print(Freq_table)

    def read_patterns(self, key_value_pairs):
        global Freq_table
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
        global IO_table
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
        global gpio_ptt_in_pin
        global gpio_ptt_in_pin_invert
        global use_wired_PTT
        global IO_table
        global main_TX
        
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
        
        gpio_ptt_in_pin = int(key_value_pairs['GPIO_PTT_IN_PIN'])
        gpio_ptt_in_pin_invert = self.str_to_bool(key_value_pairs['GPIO_PTT_IN_PIN_INVERT'])
        print("PTT Input Pin: ", gpio_ptt_in_pin, " Invert:", gpio_ptt_in_pin_invert)
        
        use_wired_PTT = int(key_value_pairs['WIRED_PTT'])
        print("Wired PTT Mode =",use_wired_PTT)
        
        main_TX = int(key_value_pairs['MAIN_TX'])
        print("Use Main Band only for TX on IC-9700 =", main_TX)
        

    # reload paramters after detection of different radio address.  Skip DHT, radio model
    def init_band(self, key_value_pairs, reload):
        if not reload:
            self.read_DHT(key_value_pairs)
        self.read_model(key_value_pairs, reload)
        if not reload:
            self.read_band(radio_band)
            self.read_patterns(key_value_pairs)
            self.read_band_pins(key_value_pairs)
            self.read_ptt_pins(key_value_pairs)
            io.gpio_config()
            self.vfoa_band = self.frequency(self.selected_vfo, self.unselected_vfo, self.selected_vfo_rx)  # 0 is vfoa
            self.ptt(PTT)


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
        if dht11_enable:
            (temp, hum, temp_F) = self.read_temps()
        else:
            temp = hum= temp_F = 0
        cpu = self.get_cpu_temp()
        tim = dtime.now()
        temp_str = (tim.strftime("%m/%d/%Y %H:%M:%S%Z")+"  Temperature: %(f)0.1f°F  %(t)0.1f°C  Humidity: %(h)0.1f%%  CPU: %(c)s°C  DHT11:%(e)d" % {"t": temp, "h": hum, "f": temp_F, 'c': cpu, 'e': int(dht11_enable)})
        print(self.colored(100,120,255,"(TEMP )"), temp_str, flush=True)
        self.write_temps(temp_str+"\n")


    #
    #    formatVFO()
    #
    def formatVFO(self, vfo):
        vfo_str = [] * 20
        #if (ModeOffset < -1 || ModeOffset > 1)
        #vfo += ModeOffset;  // Account for pitch offset when in CW mode, not others
        MHz = (vfo / 1000000 % 1000000)
        Hz = (vfo % 1000) / 10
        KHz = ((vfo % 1000000) - Hz) / 1000
        #vfo_str = format("%d.%03d.%01d" % (MHz, KHz, Hz/10))
        vfo_str = format("%d,%03d.%03d" % (MHz, KHz, Hz))
        return vfo_str


    def p_status(self, TAG):
        if not valid_address:
            return
        cpu = self.get_cpu_temp()
        #tim = dtime.now()
        
        if self.split_status == 17:
            splits = 'D'  # duplex
        elif self.split_status == 1:
            splits = '1'  # split on
        else: 
            splits = '0'   # split off
            
        sel = self.formatVFO(self.selected_vfo)
        unsel = self.formatVFO(self.unselected_vfo)
            
        print(self.colored(175,210,240,"("+TAG+")"),
            #tim.strftime("%m/%d/%y %H:%M:%S%Z"),
            "VFOA Band: "+self.colored(255,225,165,format(self.bandname,"5")),
            " A: "+self.colored(255,255,255,format(sel, "13")),
            " B: "+self.colored(215,215,215,format(unsel, "13")),
            " Split: "+self.colored(225,255,90,splits)+" ",
            " M: "+modeList[mode_idx][1],
            " F: "+FilStr[filt]+" ",
            " D: "+format(ModeStr[datamode], "10"),
            " P: "+format(self.preamp_status, "1"),
            " A: "+format(self.atten_status, "1"),
            " PTT: "+self.colored(115,195,110,format(self.ptt_state, "1")),
            " Sub: "+format(active_band, "1"),
            #" Menu: "+format(self.in_menu, "1"),   #  this toggles 0/1 when in menus, and/or when there is spectrum flowing not sure which
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


    def frequency(self, freqA, freqB, freqA_Sub):  
        # 0xd8 0x00 is normal tuning update, 0x18 on band changes
        # Duplex used split status byte for VFO swap, just has vfoB set different with offset
        # split is updated in message ID 0xD4.  Here we can also pick it up and not wait for 
        # someone to press the split button to generate the d4 event.
        # Returns the payload hex converted to int.
        # This need to have the band offset applied next
        
        if PTT or self.TX_Delay:
            return
        
        __vfob = freqB
        #print("(Freq) VFOA Main:", freqA, " VFOB Main:", freqB, " VFOA Sub:", freqA_Sub)
        if active_band:
            __vfoa = freqA_Sub
            __vfob = freqA
        else:
            __vfoa = freqA
            __vfoB = freqB
        #print("(Freq) VFOA = ", __vfoa, " VFOB = ", __vfob)
        
        if (radio_model == IC905 and (self.vfoa_band == 3 or self.vfoa_band == 4 or self.vfoa_band == 5)):
            self.atten_status = 0
            self.preamp_status = 0
            
        if radio_model == IC9700 and self.vfoa_band > '2':
            self.vfoa_band = 2  #  Only 3 bands valid for 9700 out of 6 in the table.

        #if (self.split_status != self.__split_status_last):
        #    self.write_split(self.split_status)
        #    self.__split_status_last = self.split_status
        
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
                if (radio_model == IC9700 and main_TX):
                    io.band_io_output(self.vfob_band)
                else:
                    io.band_io_output(self.vfoa_band)
                self.__vfoa_band_last = self.vfoa_band
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
            if (self.ptt_state != self.__ptt_state_last):
                #print("PTT state =", self.ptt_state)
                if (self.ptt_state == 1):  # do not TX if the band is still unknown (such as at startup)
                    #print("PTT-TX VFOA Band:", self.vfoa_band, self.selected_vfo, "VFO B Band:", self.vfob_band, self.unselected_vfo, "SPLIT:", self.split_status, "SUB:", active_band, "ptt_state is TX ")
                    #  Split and Duplex for IC9700 is complicated by the sub RX and whether to switch IO band or not on TX
                    if (radio_model != IC9700 and self.split_status != 0) or \
                        (radio_model == IC9700 and (active_band or self.split_status != 0) and not main_TX): # swap selected and unselected when split is on during TX
                        if radio_model == IC9700:
                            self.vfoa_band_split_Tx = self.vfoa_band  # back up the original VFOa band
                            self.selected_vfo_split_Tx = self.selected_vfo  # back up original VFOa  
                            self.__vfob_band_last = self.vfob_band
                            self.selected_vfo = self.unselected_vfo
                            self.vfoa_band = self.vfob_band
                            self.bandname = Freq_table[self.vfoa_band]['bandname']                         
                        
                        #print("PTT-TX SPLIT VFOA Band:", self.vfoa_band, self.selected_vfo, "VFOB Band:", self.vfob_band, "VFOA BAND SPLIT TX:", self.vfoa_band_split_Tx, "SELECTED VFO SPLIT TX:", self.selected_vfo_split_Tx, " Main_TX:", main_TX, "Actve Band:", active_band)
                        # skip the band switch and delay if on the same band
                        if (radio_model != IC9700 and self.split_status == 1 and self.__vfob_band_last != self.vfob_band) or \
                            (radio_model == IC9700 and self.split_status != 1 and active_band and not main_TX):
                            self.TX_Delay = True
                            self.p_status("SPLtx")
                            if radio_model == IC9700:    
                                io.band_io_output(self.vfoa_band)
                            time.sleep(self.PTT_hang_time)
                            print("Delay:",self.PTT_hang_time,"sec", flush=True)
                        else:
                            self.p_status(" DUP ")  # Duplex mode
                    else:
                        if radio_model == IC9700 and main_TX:
                            self.bandname = Freq_table[self.vfob_band]["bandname"]
                            self.vfoa_band = self.vfob_band
                            self.selected_vfo = self.unselected_vfo
                        self.p_status(" PTT ")  # not split or duplex
                    io.ptt_io_output(self.vfoa_band, self.ptt_state)

                else:   #(self.ptt_state == 0):                    
                    #print("PTT-RX VFOA Band:", self.vfoa_band, self.selected_vfo, "VFO B Band:", self.vfob_band, self.unselected_vfo, "SPLIT:", self.split_status, "SUB:", active_band, "ptt_state is RX ")
                    if (radio_model != IC9700 and self.split_status != 0) or (radio_model == IC9700 and (active_band or self.split_status != 0) and not main_TX): # swap selected and unselected when split is on during TX    
                        if radio_model == IC9700:
                            self.vfoa_band = self.vfoa_band_split_Tx
                            self.selected_vfo = self.selected_vfo_split_Tx
                            self.bandname = Freq_table[self.vfoa_band]['bandname']               
                        #print("PTT-RX SPLIT VFOA Band:", self.vfoa_band, self.selected_vfo, " VFO B Band:", self.vfob_band, self.unselected_vfo, " SPLIT:", self.split_status, " SUB:", active_band, " MainTX:", main_TX, " ptt_state is TX ")
                        # skip the band switch and delay if on the same band
                        if (radio_model != IC9700 and self.split_status == 1 or self.vfoa_band != self.vfoa_band) or \
                            (radio_model == IC9700 and (active_band == 1 or self.split_status == 0) and not main_TX):
                            self.p_status("SplRx")
                            io.ptt_io_output(self.vfoa_band, self.ptt_state)
                            time.sleep(self.PTT_hang_time)
                            print("Delay:",self.PTT_hang_time,"sec", flush=True)
                            if radio_model == IC9700:
                                io.band_io_output(self.vfoa_band)
                            self.TX_Delay = False
                        else:
                            self.p_status(" DUP ")
                            io.ptt_io_output(self.vfoa_band, self.ptt_state)
                    else:
                        if radio_model == IC9700 and main_TX:
                            self.bandname = Freq_table[self.vfob_band]["bandname"]
                        self.p_status(" PTT ")
                        io.ptt_io_output(self.vfoa_band, self.ptt_state)                        
                        if radio_model == IC9700 and main_TX:
                            self.selected_vfo = self.CIV_selected_vfo_rx
                            self.unselected_vfo = self.CIV_unselected_vfo
                            self.bandname = Freq_table[self.__vfoa_band_last]["bandname"]
                        self.p_status("FREQ ")
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


    def colored(self, r, g, b, text):
        return f"\033[38;2;{r};{g};{b}m{text}\033[0m"

#
#   End of class BandDecoder
#

#--------------------------------------------------------------------
#  Read config file
#--------------------------------------------------------------------

def read_config(config_file):
    global key_value_pairs

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
    except Exception as e:
            print(f"An error occurred: {e}")


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


def case_default(cmd, buffer):
    print("case_default: CI-V Command ID:", cmd)
    bd.hexdump(buffer)


def CIV_Action(cmd_num:int, data_start_idx:int, data_len:int, msg_len:int, rd_buffer):
    global radio_address
    global radio_model
    global radio_address_received
    global ALL_RADIOS
    global TX_last
    global Longitude
    global Latitude
    global Grid_Square
    global p_longitude
    global p_latitude
    global hr_off
    global min_off
    global shift_dir
    global mode_idx
    global filt
    global datamode
    global active_band  # for 9700 main=0 sub=1 used to swap 
    global CIV_selected_vfo
    global CIV_unselected_vfo
    global CIV_selected_vfo_rx
    
    #print("CIV_Action: Entry - cmd_num = %x data_len= %d  cmd = %X  data_start_idx = %d  data_len = %d  rd_buffer:%X %X %X %X %X %X %X %X %x %X %X" % (cmd_num, data_len, cmd_List[cmd_num][2],
    #         data_start_idx, data_len, rd_buffer[0], rd_buffer[1], rd_buffer[2], rd_buffer[3], rd_buffer[4], rd_buffer[5], rd_buffer[6],
    #         rd_buffer[7],rd_buffer[8], rd_buffer[9], rd_buffer[10], radio_address))

    match (cmd_num):

        case cmds.CIV_C_F25A.value | cmds.CIV_C_F25B.value | cmds.CIV_C_F_READ.value | cmds.CIV_C_F_SEND.value | cmds.CIV_C_F1_SEND.value | cmds.CIV_C_TX_FREQ.value:
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

                if rd_buffer[4] == 0x25 and rd_buffer[5] == 1:
                        bd.CIV_unselected_vfo = f
                        #print("VFOB Main", bd.CIV_unselected_vfo)
                elif rd_buffer[4] == 0x25 and rd_buffer[5] == 0:
                        bd.CIV_selected_vfo = f
                        #rint("VFOA Main", bd.CIV_selected_vfo)
                else:
                    if active_band:
                        bd.CIV_selected_vfo_rx = f
                        #print("VFOA Sub", bd.CIV_selected_vfo_rx)
                    else:
                        bd.CIV_selected_vfo = f
                        #print("VFOA Main", bd.CIV_selected_vfo)
                        #bd.CIV_selected_vfo_rx = 0
                bd.frequency(bd.CIV_selected_vfo, bd.CIV_unselected_vfo, bd.CIV_selected_vfo_rx)   #  0 is vfoa, 1 is vfo b   
        
        case cmds.CIV_C_TX.value:  # Used to request RX TX status from radio
            global PTT
            if not use_wired_PTT:
            #if not (data_len != 1 or (not(rd_buffer[4] == 0x1C or rd_buffer[5] == 0x00))):                
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
                        
        case cmds.CIV_C_TRX_ID.value:
            radio_address_received = radio_address = rd_buffer[data_start_idx]
            print("CI-V address: 0x%X" % (radio_address))
            Get_Radio_address(radio_address)

        case cmds.CIV_C_SPLIT_READ.value:
            bd.split_status = rd_buffer[data_start_idx]
            #print("CI-V Split status:", bd.split_status)    

        case cmds.CIV_C_PREAMP_READ.value:
            bd.preamp_status = rd_buffer[data_start_idx]
            #print("CIV_Action: CI-V Returned Preamp status:", bd.preamp_status)
            
        case cmds.CIV_C_ATTN_READ.value:
            bd.atten_status = rd_buffer[data_start_idx]
            #print("CIV_Action: CI-V Returned Attenuator status:", bd.atten_status)
        
        case cmds.CIV_R_MAINSUBBAND.value:
            active_band = rd_buffer[data_start_idx]
            #if active_band:
            #    print("CIV_Action: Sub Band Active:", bd.vfob_band)
            #else:
            #    print("CIV_Action: Main Band Active:", bd.vfoa_band)
                
        case cmds.CIV_C_MOD_READ.value | cmds.CIV_C_MOD1_SEND.value | cmds.CIV_C_MOD_SEND.value:
            # command CIV_C_MODE_READ received
            #print("Mode Read: cmd:%d  message:%s" % (cmd_num, [hex(num) for num in rd_buffer][0:msg_len]), flush=True) 
            # get extended mode since this does not update data mode.  It does occur on band changes which has uses
            if radio_model == IC9700:
                sendCatRequest(cmds.CIV_R_MAINSUBBAND.value, 0, 0)  # selected VFO freq
            if not PTT:
                sendCatRequest(cmds.CIV_C_F26A.value, 0, 0)   # mode, filter, datamode
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
                print("Position/Time Not available, GPS connected? - Data = %X" % (rd_buffer[data_start_idx]), flush=True)
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
            print("GPS Lat = %s  Long = %s  Grid Square is %s" % (Latitude, Longitude, Grid_Square), flush=True)
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

        case cmds.CIV_R_GOOD.value: pass
        case cmds.CIV_R_NO_GOOD.value: pass

        case _ : print("Unhandled message cmd:%d  message:%s" % (cmd_num, [hex(num) for num in rd_buffer][0:msg_len]), flush=True)   #self.case_default(cmd_num, rd_buffer)     # anything we have not seen yet comes to here


# ----------------------------------------
#      Send CAT Request
# ----------------------------------------

def sendCatRequest(cmd_num, Data, Data_len):  # first byte in Data is length
    buf_size = 70
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

    #print("sendCatRequest --> Tx Raw Msg: %s  msg_len = %d  END" % (req[0:msg_len], msg_len))

    if (msg_len < buf_size - 1):  # ensure our data is not longer than our buffer
        send_str = req[0:msg_len]
        #  This is the main debugging print
        #hex_array = [hex(num) for num in send_str][0:msg_len]
        #print("sendCatRequest: ***Send TX data = ", hex_array)
        ser_write(serial.to_bytes(send_str))
    else:
        print("sendCatRequest: Buffer overflow")


def Get_Radio_address(address):
    retry_Count = 0
    global radio_address_received
    global radio_address
    global radio_model
    global valid_address
    
    #print("Get Address", radio_address, radio_address_received, radio_model)
    # filter incorrect contents best we can
    # radio_model initially comes from the config file.and is used until there is a valid address
    # or one comes from a different model.  You could have more than 1 address for a model.
    print("Reload Data Tables, reconfigure for 0x%X" % (radio_address), flush=True)
    valid_address = True
    bd.init_band(key_value_pairs, True)


def remove():
    global ring_tail
    if (ring_head != ring_tail):
        c = ring_data[ring_tail]
        ring_tail = (ring_tail + 1) % RING_SIZE
        return c
    else:
        return -1


def processCatMessages():
    global radio_address_received
    cmd_num = 255
    match = 0
    data_start_idx = 0
    read_buffer = [0] * 40
    c = 0
    j = 0
    i = 0

    # Extract the data from the ring buffer
    while True:
        c = remove()
        if c != -1:        
            if (c != 0xfd):
                read_buffer[j] = (c)
                j += 1
            else:
                read_buffer[j] = (c)
                j += 1
                break  # end of message, we are done for now.  IRQ will only call us for 1 message at a time

    data_len = j;
    #print("String copied, now parse it")

    if (data_len):
        msg_len = data_len
        #print("processCatMessages %s  length: %d" % (read_buffer[0:data_len], data_len), flush=True)
        cmd_num = 255
        match = 0
        if (msg_len > 0):
            if (read_buffer[0] == (START_BYTE) and read_buffer[1] == (START_BYTE) and read_buffer[3] != CONTROLLER_ADDRESS):
                #hex_array = [hex(num) for num in read_buffer][0:msg_len]
                #print("processCatMessages %s  length: %d" % (hex_array, data_len))
                if (1):
                #if (read_buffer[3] != CONTROLLER_ADDRESS):
                    radio_address_received = read_buffer[3]
                    #if (radio_address_received != radio_address and radio_model != radio_address_received):
                    #    Get_Radio_address()
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
                        #print("processCatMessages: Call CIV_Action CMD = %d  CMD 0x%X" % (cmd_num, read_buffer[4]))
                        CIV_Action(cmd_num, data_start_idx, data_len, msg_len, read_buffer)

# -------------------------------------------------------------------

# Ring buffers for Serial RX
def add(c):
    global ring_head
    global ring_data

    next_head = (ring_head + 1) % RING_SIZE
    if (next_head != ring_tail):
        # there is room
        ring_data[ring_head] = c
        ring_head = next_head
        return 0
    else:
        print("Err: No room left in RX queue")
        # no room left in the buffer
        return -1


def read_port():
    try:
        if ser.isOpen():
            while ser.in_waiting > 0:
                response = ser.readline()
                if response != b'':
                    #print("read data: ", response, flush=True)
                    #hex_array = [hex(num) for num in response]
                    #print("read_port: %s" % (hex_array))
                    for ch in response:
                        c = (ch)
                        add(c)    ## usb loop task will pull this data out
                        if (c == 0xfd):  ## end of a complete message, process it.
                            processCatMessages()
                
        else:
            print("read_port: Port Not open")
            
    except SerialException as e:
        print("read_port: port error", e)
        

def ser_write(buffer):
    if init_done:
        try:         
            if ser.isOpen(): 
                hex_array = [hex(num) for num in buffer]
                #print("ser_write buffer: %s" % (hex_array))
                ser.write(buffer)
                time.sleep(0.1)  #give the serial port sometime to receive the data
                read_port()
                
        except IndexError as exc:
            print("*** print_exception:")
            traceback.print_exception(exc, limit=6, file=sys.stdout)
        except serial.SerialException as s:
            print("ser_write: SerialException: error communicating...:", s)
        except Exception as e:
            #raise Serial.SerialException
            print("ser_write: error communicating...:",e)   
        #    if not ser.isOpen(): 
        #        init_done = False
           
           
def poll_radio(bypass):
    global loop_ctr
    
    test = False   # block most polling for testing

    if not init_done:
        return
    loop_time = 60
    if (loop_ctr >= loop_time) or bypass and not test:
        #print("***L60")
        #sendCatRequest(cmds.CIV_C_SCOPE_OFF.value, 0, 0)   # turn off scope data ouput to reduce bandwidth
        
        if not PTT:
            #print("Get UTC offset")
            if radio_model == IC705:
                sendCatRequest(cmds.CIV_C_UTC_READ_705.value, 0, 0)
            if radio_model == IC905:
                sendCatRequest(cmds.CIV_C_UTC_READ_905.value, 0, 0)
            if radio_model == IC9700:
                sendCatRequest(cmds.CIV_C_UTC_READ_9700.value, 0, 0)
            sendCatRequest(cmds.CIV_C_MY_POSIT_READ.value, 0, 0)
     
    if loop_ctr % 5 == 0 or bypass and not test:  # every 5 seconds
        #print("L", loop_ctr)
        if not PTT:
            if radio_model == IC9700:
                sendCatRequest(cmds.CIV_R_MAINSUBBAND.value, 0, 0)  # selected VFO freq
            sendCatRequest(cmds.CIV_C_PREAMP_READ.value, 0, 0)
            sendCatRequest(cmds.CIV_C_ATTN_READ.value, 0, 0)
    
    # These run every 1 second
    sendCatRequest(cmds.CIV_C_TX.value, 0, 0)
    if not PTT:
        sendCatRequest(cmds.CIV_C_F25A.value, 0, 0)  # selected VFO freq
        sendCatRequest(cmds.CIV_C_F25B.value, 0, 0)  # unselected vfo freq
        sendCatRequest(cmds.CIV_C_F_READ.value, 0, 0)  # unselected vfo freq
        sendCatRequest(cmds.CIV_C_SPLIT_READ.value, 0, 0)   # split mode
    
    loop_ctr += 1
    if loop_ctr > loop_time:
        loop_ctr = 0
   
   
def serial_init():
    global ser
    global init_done
    s_path = '/home/k7mdl/rig-pty1'

    while not init_done:
        try:
            if os.path.islink(s_path):
                ser = serial.Serial(
                    #initialization and open the port
                    #possible timeout values:
                    #    1. None: wait forever, block call
                    #    2. 0: non-blocking mode, return immediately
                    #    3. x, x is bigger than 0, float allowed, timeout block call
                    port = s_path,
                    baudrate = 19200,
                    bytesize = serial.EIGHTBITS, #number of bits per bytes
                    parity = serial.PARITY_NONE, #set parity check: no parity
                    stopbits = serial.STOPBITS_ONE, #number of stop bits
                    #timeout = None,          #block read
                    timeout = 0,           #non-block read
                    #timeout = 0.01,              #timeout block read
                    xonxoff = False,     #disable software flow control
                    rtscts = False,     #disable hardware (RTS/CTS) flow control, for Virtual turn off
                    dsrdtr = False,       #disable hardware (DSR/DTR) flow control, for vrtual turn off
                    writeTimeout = 1,     #timeout for write
                    inter_byte_timeout = 0.1
                    )
                if ser.isOpen():
                    init_done = True
                    #print ("serial_init: Serial port is open, flushing Serial port")
                    ser.flushInput() #flush input buffer, discarding all its contents
                    ser.flushOutput()#flush output buffer, aborting current output 
                                    #and discard all that is in buffer
            else:
                print("serial_init: Cannot open serial port ")
                time.sleep(5) 
                init_done = False

        except SerialException as e:
            print("serial_init: Serial Exception", e)
            if os.path.islink(s_path):
                os.unlink(s_path)
            init_done = False
            time.sleep(5) 

        except Exception as e:
            print("serial_init: error communicating...:",e)
            if os.path.islink(s_path):
                os.unlink(s_path)
            init_done = False


def serial_sniffer(args):
    global poll
    global init_done
    active_band_last = 0
    global radio_address

    if init_done:
        print("Starting Program")
        try:
            while not init_done:
                time.sleep(5)
                serial_init()
                print("Waiting for serial port to open")
            if ser.isOpen():
                
                while not valid_address:
                    sendCatRequest(cmds.CIV_C_TRX_ID.value, 0, 0)
                    # Try each address until one responds
                sendCatRequest(cmds.CIV_C_PREAMP_READ.value, 0, 0)
                sendCatRequest(cmds.CIV_C_ATTN_READ.value, 0, 0)
                sendCatRequest(cmds.CIV_C_SPLIT_READ.value, 0, 0)
                sendCatRequest(cmds.CIV_C_F26A.value, 0, 0)
                if radio_model == IC9700:
                    sendCatRequest(cmds.CIV_R_MAINSUBBAND.value, 0, 0)  # selected VFO freq
                sendCatRequest(cmds.CIV_C_F_SEND.value, 0, 0)  # selected VFO freq
                sendCatRequest(cmds.CIV_C_F25A.value, 0, 0)
                sendCatRequest(cmds.CIV_C_F25B.value, 0, 0)
                sendCatRequest(cmds.CIV_C_F_READ.value, 0, 0)  # selected VFO freq
                #sendCatRequest(cmds.CIV_C_SCOPE_OFF.value, 0, 0)   # turn off scope data ouput to reduce bandwidth

                poll_radio(True) # initalize
                poll = RepeatedTimer(1, poll_radio, False)  # call every 1 sec
                print("Polling timer thread enabled:", flush=True)
                
                print("Main loop, waiting for RX events")
                while (1):
                    io.PTT_In()
                    read_port()
                    if (radio_model == IC9700 and active_band != active_band_last):
                        sendCatRequest(cmds.CIV_C_F_READ.value, 0, 0)  # selected VFO freq
                        active_band_last = active_band
            else:
                print("Serial Port is not open")
                init_done = False
                        
        except KeyboardInterrupt:
            print('Done')
            dht.stop()
            poll.stop()
            ser.close()
            #com.stop()
            #dc.stop()

        finally:
            GPIO.cleanup()
            sys.exit()


if __name__ == '__main__':
    io = OutputHandler()  # instantiate our classes
    bd = BandDecoder()
    Freq_table = Freq_table_705

    print("CI-V Serial Band Decoder - K7MDL Mar 2025")
    tim = dtime.now()
    print("Startup at", tim.strftime("%m/%d/%Y %H:%M:%S%Z"), flush=True)

    # read in config, split and band files
    config_file = os.path.expanduser("~/Decoder.config")
    if os.path.exists(config_file):
        key_value_pairs = read_config(config_file)

    bd.init_band(key_value_pairs, False)

    # Update the temperature log
    bd.write_temps("Program Startup\n")

    dht = RepeatedTimer(dht11_poll_time, bd.temps)
    print("DHT11 enabled:", dht11_enable, "  DHT11 Poll time:",dht11_poll_time, flush=True)    
    
    serial_init()
    #com = DecoderThread(read_port(ser))
    # Create and start the thread
    #read_thread = threading.Thread(target=read_port, args=())
    #read_thread.daemon = True  # This makes sure the thread will exit when the main program exits
    #read_thread.start()

    # Start the main program
    #dc = DecoderThread(serial_sniffer(sys.argv))   # option to run main program in a thread
    serial_sniffer(sys.argv)

    #  Program never returns here
    io = None
    bd = None
    mh = None

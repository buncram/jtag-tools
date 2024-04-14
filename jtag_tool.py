#first release march 4th - 0.0.1
#!/usr/bin/python3


import os

if os.name == 'posix' and os.uname()[1] == 'raspberrypi':
    try:
        import RPi.GPIO as GPIO
    except RuntimeError:
        print("Error importing RPi.GPIO! Did you run as root?")
    USE_GPIO = True
else:
    USE_GPIO = False

import csv
import argparse
import time
import logging
import sys
import re


from enum import Enum
from cffi import FFI

from pathlib import Path
Path("./logs/").mkdir(parents=True, exist_ok=True)

TIMEOUT_S = 5  # default timeout, in seconds
BANK_RRAM0 = 0
BANK_RRAM1 = 1
BANK_RRAMS = 2
BANK_IPT   = 3

# Tracks a group of tests. Groups are defined as some consecutive number of JTAG operations
# that are not a different "test" *and* do not require any wait or check condition.
#
# A given CP test may have several groups within it.
#
# A single CP test is defined as a named test that starts with an @@@ comment.
class JtagGroup():
    def __init__(self):
        # Test data is structured in a dictionary so we can export to JSON easily if needed.
        self.test = {
            'start' : -1,
            'end' : -1,
            'legs' : [],
            'wait' : '',
            'check': '',
            'debug': 0,
        }
        self.dr_vals = {} # dictionary of test serial numbers to expected DR values
        self.dbg_state = {} # dictionary of test serial numbers to expected DR values

    def has_start(self):
        if self.test['start'] < 0:
            return False
        else:
            return True
    def set_start(self, num):
        self.test['start'] = num
    def get_start(self):
        return self.test['start']
    def has_end(self):
        if self.test['end'] < 0:
            return False
        else:
            return True
    def set_end(self, num):
        self.test['end'] = num
    def get_end(self):
        return self.test['end']
    def append_leg(self, leg):
        self.test['legs'] += [leg]
    def get_legs(self):
        return self.test['legs']
    # A "wait" can only happen at the *end* of a JtagGroup's list of JTAG commands
    # Thus each group can only have one wait set, and it is always executed after
    # the last JTAG command has completed.
    def has_wait(self):
        return self.test['wait'] != ''
    def set_wait(self, condition):
        self.test['wait'] = condition
    def get_wait(self):
        return self.test['wait']
    # A "check" only applies to the *first* command in a JtagGroup's list of JTAG commands.
    # Thus each group can only have at most one check condition, and it is always executed
    # immediately after the first JTAG DR command.
    def has_check(self):
        return self.test['check'] != ''
    def get_check(self):
        return self.test['check']
    def set_check(self, condition):
        self.test['check'] = condition
    def add_dr_val(self, serial_no, dr_val_as_int):
        self.dr_vals[serial_no] = dr_val_as_int
    def lookup_dr_val(self, serial_no):
        return self.dr_vals[serial_no]
    def set_debug(self):
        self.test['debug'] = 1
    def clear_debug(self):
        self.test['debug'] = 0
    def get_debug(self):
        return self.test['debug']
    def add_dbg_state(self, serial_no, dr_val_as_int):
        self.dbg_state[serial_no] = dr_val_as_int
    def lookup_dbg_state(self, serial_no):
        return self.dbg_state[serial_no]

# CpTest is a structure for collecting several groups together into a single named test.
class CpTest():
    def __init__(self, test_name, bank=0):
        self.name = test_name
        self.groups = []
        self.bank = bank

    def append_group(self, group):
        self.groups += [group]

TCK_pin = 4
TMS_pin = 17
TDI_pin = 27  # TDI on FPGA, out for this script
TDO_pin = 22  # TDO on FPGA, in for this script
PRG_pin = 24
# rram0
TDI_R0_pin = 27
TDO_R0_pin = 22
TMS_R0_pin = 17
# rram1
TDI_R1_pin = 9
TDO_R1_pin = 11
TMS_R1_pin = 10
# ipt
TDI_IPT_pin = 6
TDO_IPT_pin = 13
TMS_IPT_pin = 5

DBG_pin = 14
tex_mode = False

if USE_GPIO:
    ffi = FFI()
    ffi.cdef("""
    typedef struct pindefs pindefs;
    struct pindefs {
    uint32_t tck;
    uint32_t tms_r0;
    uint32_t tdi_r0;
    uint32_t tdo_r0;
    uint32_t tms_r1;
    uint32_t tdi_r1;
    uint32_t tdo_r1;
    uint32_t trst;
    uint32_t dbg;
    };

    volatile uint32_t *pi_mmio_init(uint32_t base);
    char jtag_pins(int tdi, int tms, pindefs pins, volatile uint32_t *gpio);
    void jtag_cycle_clk(pindefs pins, volatile uint32_t *gpio, int cycles);
    void set_dbg(pindefs pins, volatile uint32_t *gpio);
    void clear_dbg(pindefs pins, volatile uint32_t *gpio);
    int jtag_prog(char *bitstream, pindefs pins, volatile uint32_t *gpio);
    void jtag_prog_rbk(char *bitstream, pindefs pins, volatile uint32_t *gpio, char *readback);

    int gpioInitialise(void);
    void startClock(void);
    void stopClock(void);
    """)

    found_libs = list(Path('.').glob('*.so'))
    if len(found_libs) == 0:
        print('CFFI module not found, attempting to rebuild...')
        ffi.set_source("gpioffi", '#include "gpio-ffi.h"', sources=["gpio-ffi.c"])
        ffi.compile()
        found_libs = list(Path('.').glob('*.so'))

    assert len(found_libs) == 1, "CFFI object file structure is wrong. Please clear any *.so and *.o files and re-run."

    keepalive = []
    gpioffi = ffi.dlopen('./' + found_libs[0].name)

    pins = ffi.new("struct pindefs *")
    pins.tck = TCK_pin
    pins.tms_r0 = TMS_pin
    pins.tdi_r0 = TDI_pin
    pins.tdo_r0 = TDO_pin
    pins.trst = PRG_pin
    pins.dbg =  DBG_pin
    keepalive.append(pins)

    # maxbuf - maximum length, in bits, of a bitstream that can be handled by this script
    maxbuf = 20 * 1024 * 1024
    ffistr = ffi.new("char[]", bytes(maxbuf))
    keepalive.append(ffistr)
    ffiret = ffi.new("char[]", bytes(maxbuf))
    keepalive.append(ffiret)

class JtagLeg(Enum):
    DR = 0
    IR = 1
    RS = 2 # reset
    DL = 3 # long delay
    ID = 4 # idle in run-test
    IRP = 5  # IR with pause
    IRD = 6  # transition to IR directly
    DRC = 7  # DR for config: MSB-to-LSB order, and use fast protocols
    DRR = 8  # DR for recovery: print out the value returned in non-debug modes
    DRS = 9  # DR for SPI: MSB-to-LSB order, use fast protocols, but also readback data

class JtagState(Enum):
    TEST_LOGIC_RESET = 0
    RUN_TEST_IDLE = 1
    SELECT_SCAN = 2
    CAPTURE = 3
    SHIFT = 4
    EXIT1 = 5
    PAUSE = 6
    EXIT2 = 7
    UPDATE = 8

state = JtagState.RUN_TEST_IDLE
cur_leg = []
jtag_legs = []
tdo_vect_r0 = ''
tdo_vect_r1 = ''
tdo_stash = ''
jtag_results_r0 = []
jtag_results_r1 = []
do_pause = False
gpio_pointer = 0
compat = False
readout = False
readdata = 0
use_key = False
nky_key = ''
nky_iv = ''
nky_hmac =''
use_fuzzer = False

from math import log
def bytes_needed(n):
    if n == 0:
        return 1
    return int(log(n, 256))+1

def int_to_binstr(n):
    return bin(n)[2:].zfill(bytes_needed(n)*8)

def int_to_binstr_bitwidth(n, bitwidth):
    return bin(n)[2:].zfill(bitwidth)

def phy_sync(tdi, tms):
    global USE_GPIO
    if USE_GPIO:
        global TCK_pin, TMS_pin, TDI_pin, TDO_pin, pins, gpio_pointer, gpioffi

        if compat:
            tdo = GPIO.input(TDO_pin) # grab the TDO value before the clock changes

            GPIO.output( (TCK_pin, TDI_pin, TMS_pin), (0, tdi, tms) )
            GPIO.output( (TCK_pin, TDI_pin, TMS_pin), (1, tdi, tms) )
            GPIO.output( (TCK_pin, TDI_pin, TMS_pin), (0, tdi, tms) )
        else:
            tdo = gpioffi.jtag_pins(tdi, tms, pins[0], gpio_pointer)
    else:
        logging.debug(f"tdi: {tdi}, tms: {tms}")
        tdo = 0

    return tdo

def reset_fpga():
    global PRG_pin
    global USE_GPIO

    if USE_GPIO:
        GPIO.output(PRG_pin, 0)
        time.sleep(0.1)
        GPIO.output(PRG_pin, 1)
        print("resetting JTAG...")
    else:
        logging.debug("FPGA reset")

def decode_ir(ir):
    if not tex_mode:
        if ir == 0b100110:
            return 'EXTEST'
        elif ir == 0b111100:
            return 'EXTEST_PULSE'
        elif ir == 0b111101:
            return 'EXTEST_TRAIN'
        elif ir == 0b000001:
            return 'SAMPLE'
        elif ir == 0b000010:
            return 'USER1'
        elif ir == 0b000011:
            return 'USER2'
        elif ir == 0b100010:
            return 'USER3'
        elif ir == 0b100011:
            return 'USER4'
        elif ir == 0b000100:
            return 'CFG_OUT'
        elif ir == 0b000101:
            return 'CFG_IN'
        elif ir == 0b001001:
            return 'IDCODE'
        elif ir == 0b001010:
            return 'HIGHZ_IO'
        elif ir == 0b001011:
            return 'JPROGRAM'
        elif ir == 0b001100:
            return 'JSTART'
        elif ir == 0b001101:
            return 'JSHUTDOWN'
        elif ir == 0b110111:
            return 'XADC_DRP'
        elif ir == 0b010000:
            return 'ISC_ENABLE'
        elif ir == 0b010001:
            return 'ISC_PROGRAM'
        elif ir == 0b010010:
            return 'XSC_PROGRAM_KEY'
        elif ir == 0b010111:
            return 'XSC_DNA'
        elif ir == 0b110010:
            return 'FUSE_DNA'
        elif ir == 0b010100:
            return 'ISC_NOOP'
        elif ir == 0b010110:
            return 'ISC_DISABLE'
        elif ir == 0b111111:
            return 'BYPASS'
        elif ir == 0b110001:
            return 'FUSE_KEY'
        elif ir == 0b110011:
            return 'FUSE_USER'
        elif ir == 0b110100:
            return 'FUSE_CNTL'
        else:
            return ''  # unknown just leave blank for now
    else:
        ir = ir >> 2 # discard two lower bits, they set a mode
        if ir == 0x1:
            return 'R_BIST_IP_CONFIG'
        elif ir == 0x2:
            return 'R_BIST_TRC_DATA'
        elif ir == 0x3:
            return 'R_BIST_PATTERN'
        elif ir == 0x4:
            return 'R_BIST_ADR_CTRL'
        elif ir == 0x5:
            return 'R_BIST_TRC_DATA_RW_BIT_EABLE'
        elif ir == 0x6:
            return 'R_BIST_CMD'
        elif ir == 0x7:
            return 'R_BIST_PIN_CTRL'
        elif ir == 0x8:
            return 'R_BIST_ADR_CTRL_SETTING_1'
        elif ir == 0x9:
            return 'R_BIST_GRPSEL'
        elif ir == 0xb:
            return 'R_BIST_STATUS'
        elif ir == 0xc:
            return 'R_BIST_STATUS_EN'
        elif ir == 0xd:
            return 'R_BIST_STATUS_CLR'
        elif ir == 0xe:
            return 'R_BIST_STATUS_ARRYINFO_IP0'
        elif ir == 0xf:
            return 'R_BIST_STATUS_EN_ARRYINFO_PI0'
        elif ir == 0x10:
            return 'R_BIST_LOOP_CTRL'
        elif ir == 0x11:
            return 'R_BIST_VERSION'
        elif ir == 0x12:
            return 'R_BIST_FPC_SETTING'
        elif ir == 0x13:
            return 'R_BIST_TRIM_COMPARE'
        elif ir == 0x14:
            return 'R_BIST_DBG_SETTING'
        elif ir == 0x15:
            return 'R_BIST_DBG_BIST_DOUT'
        elif ir == 0x16:
            return 'R_BIST_ADR_CTRL_SETTING_0'
        elif ir == 0x18:
            return 'R_BIST_REPAIR_SECTOR_SEL'
        elif ir == 0x19:
            return 'R_BIST_COL_REPAIR_SEC'
        elif ir == 0x1a:
            return 'R_BIST_ROW_REPAIR_SEC_TOPBOT_BANK'
        elif ir == 0x20:
            return 'R_BIST_MULTI_XSIZE_PROGRAM'
        elif ir == 0x21:
            return 'R_BIST_MULTI_XSIZE_PROGRAM_1'
        elif ir == 0x22:
            return 'R_BIST_MULTI_XSIZE_PROGRAM_2'
        elif ir == 0x23:
            return 'R_BIST_COUTER_SETTING'
        elif ir == 0x34:
            return 'R_BIST_BLG_TARGET_STATUS_EN'
        elif ir == 0x35:
            return 'R_BIST_BLG_TARGET_STATUS'
        elif ir == 0x36:
            return 'R_BIST_BLG_DUMMY_STATUS_EN'
        elif ir == 0x37:
            return 'R_BIST_BLG_DUMMY_STATUS'
        elif ir == 0x3c:
            return 'R_BIST_BLG_IP_CONFIG'
        else:
            return ''  # unknown just leave blank for now

def debug_spew(cur_leg):
    if not((cur_leg[0] == JtagLeg.DRC) or (cur_leg[0] == JtagLeg.DRS)):
        logging.debug("start: %s (%s) / %s", str(cur_leg), str(decode_ir(int(cur_leg[1],2))), str(cur_leg[2]) )
    else:
        logging.debug("start: %s config data of length %s", cur_leg[0], str(len(cur_leg[1])))

# take a trace and attempt to extract IR, DR values
# assume: at the start of each 'trace' we are coming from TEST-LOGIC-RESET
def jtag_step():
    global state
    global cur_leg
    global jtag_legs
    global jtag_results_r0
    global jtag_results_r1
    global tdo_vect_r0, tdo_vect_r1, tdo_stash
    global do_pause
    global TCK_pin, TMS_pin, TDI_pin, TDO_pin
    global keepalive
    global compat
    global readout
    global readdata
    global USE_GPIO
    if USE_GPIO:
        global gpio_pointer, gpioffi, pins

    # logging.debug(state)
    if state == JtagState.TEST_LOGIC_RESET:
        phy_sync(0, 0)
        state = JtagState.RUN_TEST_IDLE

    elif state == JtagState.RUN_TEST_IDLE:
        if len(cur_leg):
            # logging.debug(cur_leg[0])
            if cur_leg[0] == JtagLeg.DR or cur_leg[0] == JtagLeg.DRC or cur_leg[0] == JtagLeg.DRR or cur_leg[0] == JtagLeg.DRS:
                phy_sync(0, 1)
                if cur_leg[0] == JtagLeg.DRR or cur_leg[0] == JtagLeg.DRS:
                    readout = True
                else:
                    readout = False
                state = JtagState.SELECT_SCAN
            elif cur_leg[0] == JtagLeg.IR or cur_leg[0] == JtagLeg.IRD:
                phy_sync(0, 1)
                phy_sync(0, 1)
                do_pause = False
                state = JtagState.SELECT_SCAN
            elif cur_leg[0] == JtagLeg.IRP:
                phy_sync(0, 1)
                phy_sync(0, 1)
                do_pause = True
                state = JtagState.SELECT_SCAN
            elif cur_leg[0] == JtagLeg.RS:
                logging.debug("tms reset")
                phy_sync(0, 1)
                phy_sync(0, 1)
                phy_sync(0, 1)
                phy_sync(0, 1)
                phy_sync(0, 1)
                phy_sync(0, 1)
                phy_sync(0, 1)
                phy_sync(0, 1)
                phy_sync(0, 1)
                phy_sync(0, 1)
                phy_sync(0, 1)
                phy_sync(0, 1)
                if len(jtag_legs) > 0:
                    cur_leg = jtag_legs.pop(0)
                debug_spew(cur_leg)
                state = JtagState.TEST_LOGIC_RESET
            elif cur_leg[0] == JtagLeg.DL:
                time.sleep(0.005) # 5ms delay
                cur_leg = jtag_legs.pop(0)
                debug_spew(cur_leg)
            elif cur_leg[0] == JtagLeg.ID:
                phy_sync(0, 0)
                cur_leg = jtag_legs.pop(0)
                debug_spew(cur_leg)
        else:
            if len(jtag_legs):
                cur_leg = jtag_legs.pop(0)
                debug_spew(cur_leg)
            else:
                phy_sync(0, 0)
            state = JtagState.RUN_TEST_IDLE

    elif state == JtagState.SELECT_SCAN:
        phy_sync(0, 0)
        state = JtagState.CAPTURE

    elif state == JtagState.CAPTURE:
        phy_sync(0, 0)
        tdo_vect_r0 = ''  # prep the tdo_vect_r0 to receive data
        tdo_vect_r1 = ''
        state = JtagState.SHIFT

    elif state == JtagState.SHIFT:
        if cur_leg[0] == JtagLeg.DRC or cur_leg[0] == JtagLeg.DRS:
            if cur_leg[0] == JtagLeg.DRC: # duplicate code because we want speed (eliminating TDO readback is significant speedup)
                if USE_GPIO:
                    if compat:
                        GPIO.output((TCK_pin, TDI_pin), (0, 1))
                        for bit in cur_leg[1][:-1]:
                            if bit == '1':
                                GPIO.output((TCK_pin, TDI_pin), (1, 1))
                                GPIO.output((TCK_pin, TDI_pin), (0, 1))
                            else:
                                GPIO.output((TCK_pin, TDI_pin), (1, 0))
                                GPIO.output((TCK_pin, TDI_pin), (0, 0))
                    else:
                        bytestr = bytes(cur_leg[1][:-1], 'utf-8')
                        ffi = FFI()
                        ffistr = ffi.new("char[]", bytestr)
                        keepalive.append(ffistr)  # need to make sure the lifetime of the string is long enough for the call
                        gpioffi.jtag_prog(ffistr, pins[0], gpio_pointer)
                        GPIO.output(TCK_pin, 0)  # restore this to 0, as jtag_prog() leaves TCK high when done
                else:
                    bytestr = bytes(cur_leg[1][:-1], 'utf-8')
                    logging.debug(f"tdi <- {bytestr.hex()}")

            else:  # jtagleg is DRS -- duplicate code, as TDO readback slows things down significantly
                if USE_GPIO:
                    if compat:
                        GPIO.output((TCK_pin, TDI_pin), (0, 1))
                        for bit in cur_leg[1][:-1]:
                            if bit == '1':
                                GPIO.output( (TCK_pin, TDI_pin), (1, 1) )
                                GPIO.output( (TCK_pin, TDI_pin), (0, 1) )
                            else:
                                GPIO.output( (TCK_pin, TDI_pin), (1, 0) )
                                GPIO.output( (TCK_pin, TDI_pin), (0, 0) )
                        tdo = GPIO.input(TDO_pin)
                        if tdo == 1 :
                            tdo_vect_r0 = '1' + tdo_vect_r0
                        else:
                            tdo_vect_r0 = '0' + tdo_vect_r0
                    else:
                        bytestr = bytes(cur_leg[1][:-1], 'utf-8')
                        tdo_temp = '0'*len(cur_leg[1][:-1]) # initialize space for tdo_vect_r0
                        retstr = bytes(tdo_temp, 'utf-8')
                        ffi = FFI()
                        ffistr = ffi.new("char[]", bytestr)
                        ffiret = ffi.new("char[]", retstr)
                        keepalive.append(ffistr) # need to make sure the lifetime of the string is long enough for the call
                        keepalive.append(ffiret)
                        gpioffi.jtag_prog_rbk(ffistr, pins[0], gpio_pointer, ffiret)
                        tdo_vect_r0 = ffi.string(ffiret).decode('utf-8')
                else:
                    bytestr = bytes(cur_leg[1][:-1], 'utf-8')
                    logging.debug(f"tdi <- {bytestr.hex()}")

            state = JtagState.SHIFT

            if cur_leg[-1:] == '1':
                tdi = 1
            else:
                tdi = 0
            cur_leg = ''
            tdo = phy_sync(tdi, 1)
            if tdo == 1:
                tdo_vect_r0 = '1' + tdo_vect_r0
            else:
                tdo_vect_r0 = '0' + tdo_vect_r0

            if (tdo & 0x10) == 1:
                tdo_vect_r1 = '1' + tdo_vect_r1
            else:
                tdo_vect_r1 = '0' + tdo_vect_r1

            state = JtagState.EXIT1
            logging.debug('leaving config')

        else:
            if len(cur_leg[1]) > 1:
                if cur_leg[1][-1] == '1':
                    tdi = 1
                else:
                    tdi = 0
                cur_leg[1] = cur_leg[1][:-1]
                tdo = phy_sync(tdi, 0)
                if tdo[0] == 1 or tdo[0] == 17:
                    tdo_vect_r0 = '1' + tdo_vect_r0
                else:
                    tdo_vect_r0 = '0' + tdo_vect_r0

                if tdo[0] == 16 or tdo[0] == 17:
                    tdo_vect_r1 = '1' + tdo_vect_r1
                else:
                    tdo_vect_r1 = '0' + tdo_vect_r1

                state = JtagState.SHIFT
            else: # this is the last item
                if cur_leg[1][0] == '1':
                    tdi = 1
                else:
                    tdi = 0
                cur_leg = ''
                tdo = phy_sync(tdi, 1)
                if tdo[0] == 1 or tdo[0] == 17:
                    tdo_vect_r0 = '1' + tdo_vect_r0
                else:
                    tdo_vect_r0 = '0' + tdo_vect_r0

                if tdo[0] == 16 or tdo[0] == 17:
                    tdo_vect_r1 = '1' + tdo_vect_r1
                else:
                    tdo_vect_r1 = '0' + tdo_vect_r1
 
                state = JtagState.EXIT1

    elif state == JtagState.EXIT1:
        tdo_stash = tdo_vect_r0
        if do_pause:
           phy_sync(0, 0)
           state = JtagState.PAUSE
           do_pause = False
        else:
           phy_sync(0, 1)
           state = JtagState.UPDATE

    elif state == JtagState.PAUSE:
        logging.debug("pause")
        # we could put more pauses in here but we haven't seen this needed yet
        phy_sync(0, 1)
        state = JtagState.EXIT2

    elif state == JtagState.EXIT2:
        phy_sync(0, 1)
        state = JtagState.UPDATE

    elif state == JtagState.UPDATE:
        jtag_results_r0.append(int(tdo_vect_r0, 2)) # interpret the vector and save it
        logging.debug("result: %s", str(hex(int(tdo_vect_r0, 2))) )
        if readout:
            #print('readout: 0x{:08x}'.format( int(tdo_vect_r0, 2) ) )
            readdata = int(tdo_vect_r0, 2)
            readout = False
        tdo_vect_r0 = ''

        jtag_results_r1.append(int(tdo_vect_r1, 2)) # interpret the vector and save it
        logging.debug("result: %s", str(hex(int(tdo_vect_r1, 2))) )
#        if readout:
#            #print('readout: 0x{:08x}'.format( int(tdo_vect_r1, 2) ) )
#            readdata = int(tdo_vect_r1, 2)
 #           readout = False
        tdo_vect_r1 = ''

        # handle case of "shortcut" to DR
        if len(jtag_legs):
            if (jtag_legs[0][0] == JtagLeg.DR) or (jtag_legs[0][0] == JtagLeg.IRP) or (jtag_legs[0][0] == JtagLeg.IRD):
                if jtag_legs[0][0] == JtagLeg.IRP or jtag_legs[0][0] == JtagLeg.IRD:
                    phy_sync(0, 1)  # +1 cycle on top of the DR cycle below
                    logging.debug("IR bypassing wait state")
                if jtag_legs[0][0] == JtagLeg.IRP:
                    do_pause = True

                cur_leg = jtag_legs.pop(0)
                debug_spew(cur_leg)
                phy_sync(0,1)
                state = JtagState.SELECT_SCAN
            else:
                phy_sync(0, 0)
                state = JtagState.RUN_TEST_IDLE
        else:
            phy_sync(0, 0)
            state = JtagState.RUN_TEST_IDLE

    else:
        print("Illegal state encountered!")

def jtag_next():
    global state
    global jtag_results_r0
    global jtag_results_r1

    if state == JtagState.TEST_LOGIC_RESET or state == JtagState.RUN_TEST_IDLE:
        if len(jtag_legs):
            # run until out of idle
            while state == JtagState.TEST_LOGIC_RESET or state == JtagState.RUN_TEST_IDLE:
                jtag_step()

            # run to idle
            while state != JtagState.TEST_LOGIC_RESET and state != JtagState.RUN_TEST_IDLE:
                jtag_step()
        else:
            # this should do nothing
            jtag_step()
    else:
        # we're in a leg, run to idle
        while state != JtagState.TEST_LOGIC_RESET and state != JtagState.RUN_TEST_IDLE:
            jtag_step()

def auto_int(x):
    return int(x, 0)

def set_bank(bank):
    global TMS_pin, TDI_pin, TDO_pin
    global TMS_R1_pin, TDI_R1_pin, TDO_R1_pin
    global TMS_R0_pin, TDI_R0_pin, TDO_R0_pin
    global pins, USE_GPIO
    if bank == BANK_RRAMS:
        logging.info("Selecting both RRAMS")
        if USE_GPIO:
            pins.tms_r1 = TMS_R1_pin
            pins.tdi_r1 = TDI_R1_pin
            pins.tdo_r1 = TDO_R1_pin
            pins.tms_r0 = TMS_R0_pin
            pins.tdi_r0 = TDI_R0_pin
            pins.tdo_r0 = TDO_R0_pin
        TMS_pin = TMS_R1_pin
        TDI_pin = TDI_R1_pin
        TDO_pin = TDO_R1_pin
    elif bank == BANK_RRAM1:
        logging.info("Selecting RRAM1")
        if USE_GPIO:
            pins.tms_r1 = TMS_R1_pin
            pins.tdi_r1 = TDI_R1_pin
            pins.tdo_r1 = TDO_R1_pin
            pins.tms_r0 = 0
            pins.tdi_r0 = 0
            pins.tdo_r0 = 0
        TMS_pin = TMS_R1_pin
        TDI_pin = TDI_R1_pin
        TDO_pin = TDO_R1_pin
    elif bank == BANK_IPT:
        logging.info("Selecting IPT")
        if USE_GPIO:
            pins.tms_r1 = 0
            pins.tdi_r1 = 0
            pins.tdo_r1 = 0
            pins.tms_r0 = TMS_IPT_pin
            pins.tdi_r0 = TDI_IPT_pin
            pins.tdo_r0 = TDO_IPT_pin
        TMS_pin = TMS_IPT_pin
        TDI_pin = TDI_IPT_pin
        TDO_pin = TDO_IPT_pin
    else:
        logging.info("Selecting RRAM0")
        if USE_GPIO:
            pins.tms_r1 = 0
            pins.tdi_r1 = 0
            pins.tdo_r1 = 0
            pins.tms_r0 = TMS_R0_pin
            pins.tdi_r0 = TDI_R0_pin
            pins.tdo_r0 = TDO_R0_pin
        TMS_pin = TMS_R0_pin
        TDI_pin = TDI_R0_pin
        TDO_pin = TDO_R0_pin

class TexWriter():
    def __init__(self, obin):
        self.obin = obin
        self.cmd_number = 0

    def write_cmd(self, cmd):
        if 'expected DR data' in cmd:
            self.obin.write(f"#{self.cmd_number}, {cmd}\n")
        else:
            self.obin.write(f"#{self.cmd_number}, {cmd}, expected DR data:(40'h0000000000)\n")
        self.cmd_number += 1
    def write_comment(self, comment):
        self.obin.write(f"// {comment}\n")
    def write_testname(self, name):
        self.obin.write(f"@@@ {name}\n")
    def write_wait_tdo(self):
        self.obin.write("Wait for JTAG-TDO to fall and Check Status\n")
        self.obin.write("Check OUTPUT DR bit[X]\n")
    def set_bank(self, bank):
        self.obin.write(f"!bank {bank}\n")
    def write_rram(self, address, data, bank):
        self.set_bank(bank)
        self.write_testname(f'Write single word {int.from_bytes(data, "little"):032x} w/ECC ON at address 0x{address:x}, bank{bank}')
        self.write_cmd("RW JTAG-REG, IR={6'h03,2'b10}(addr:('h03)), DR data:(40'h0000000000)")
        self.write_cmd("RW JTAG-REG, IR={6'h04,2'b10}(addr:('h04)), DR data:(40'h0000000000)")
        self.write_cmd("RW JTAG-REG, IR={6'h02,2'b10}(addr:('h02)), DR data:(40'h0000000001)")
        self.write_cmd("RW JTAG-REG, IR={6'h09,2'b10}(addr:('h09)), DR data:(40'h0000000410)")
        self.write_cmd("RW JTAG-REG, IR={6'h02,2'b10}(addr:('h02)), DR data:(40'h0000000000)")
        self.write_cmd("RW JTAG-REG, IR={6'h09,2'b10}(addr:('h09)), DR data:(40'h0000000411)")
        self.write_cmd("RW JTAG-REG, IR={6'h06,2'b10}(addr:('h06)), DR data:(40'h0000600080)")
        self.write_cmd("RW JTAG-REG, IR={6'h03,2'b10}(addr:('h03)), DR data:(40'h0000000000)")
        self.write_cmd("RW JTAG-REG, IR={6'h04,2'b10}(addr:('h04)), DR data:(40'h0000000004)")
        self.write_cmd("RW JTAG-REG, IR={6'h02,2'b10}(addr:('h02)), DR data:(40'h0018000000)")
        self.write_cmd("RW JTAG-REG, IR={6'h09,2'b10}(addr:('h09)), DR data:(40'h0000000410)")
        self.write_cmd("RW JTAG-REG, IR={6'h02,2'b10}(addr:('h02)), DR data:(40'h0000000001)")
        self.write_cmd("RW JTAG-REG, IR={6'h09,2'b10}(addr:('h09)), DR data:(40'h0000000411)")
        self.write_cmd("RW JTAG-REG, IR={6'h06,2'b10}(addr:('h06)), DR data:(40'h0000600080)")
        self.write_comment("select main array/INFO/redundancy/INFO1 area for write")
        self.write_cmd("RW JTAG-REG, IR={6'h07,2'b10}(addr:('h07)), DR data:(40'h0000000002)")
        self.write_comment("clear write data buffer")
        self.write_cmd("RW JTAG-REG, IR={6'h06,2'b10}(addr:('h06)), DR data:(40'h0000603480)")
        # write data
        for wr_ptr in range(0, 16, 4):
            self.write_comment(f"User loads data buffer [{(wr_ptr+4) * 8 - 1}:{wr_ptr * 8}]")
            word = int.from_bytes(data[wr_ptr:wr_ptr + 4], 'little')
            self.write_cmd(f"RW JTAG-REG, IR={{6'h02,2'b10}}(addr:('h02)), DR data:(40'h{word:010x})")
            self.write_comment(f"User selects buffer {wr_ptr // 4} to load buffer data")
            self.write_cmd(f"RW JTAG-REG, IR={{6'h09,2'b10}}(addr:('h09)), DR data:(40'h{0x410 + wr_ptr // 4:010x})")
        self.write_comment("User loads data buffer[143:128] = 0x0000 (DR bit[15:0]) = Don't Care w/ ECC ON")
        self.write_cmd("RW JTAG-REG, IR={6'h02,2'b10}(addr:('h02)), DR data:(40'h0000000000)")
        self.write_comment("User selects buffer 4 to load buffer data")
        self.write_cmd("RW JTAG-REG, IR={6'h09,2'b10}(addr:('h09)), DR data:(40'h0000000414)")
        # write addresses
        y_address = (address & 0b11_1110_0000) >> 5
        self.write_comment(f"User inputs WRITE Y address = 0x{y_address:x} (DR bit[23:16]), bank{bank}")
        self.write_cmd(f"RW JTAG-REG, IR={{6'h04,2'b10}}(addr:('h04)), DR data:(40'h{(y_address << 16) | 3:010x})")
        self.write_comment("User issues BIST LOAD command and, starts bist_run")
        self.write_cmd("RW JTAG-REG, IR={6'h06,2'b10}(addr:('h06)), DR data:(40'h0000602c80)")
        x_address = (address & 0b11_1111_1111_1100_0000_0000) >> 10
        self.write_comment(f"User inputs WRITE X address = 0x{x_address:x} (DR bit[15:0]), bank{bank}")
        self.write_cmd(f"RW JTAG-REG, IR={{6'h04,2'b10}}(addr:('h04)), DR data:(40'h{x_address:010x})")
        self.write_comment("User inputs WRITE # of loop=0x0")
        self.write_cmd("RW JTAG-REG, IR={6'h10,2'b10}(addr:('h10)), DR data:(40'h0000000000)")
        self.write_comment("User inputs address and data pattern(ADR_FIX, DATA_FIX, LOOP_0)")
        self.write_cmd("RW JTAG-REG, IR={6'h03,2'b10}(addr:('h03)), DR data:(40'h0000000000)")
        self.write_comment("User sets bist_write_status_ip0_en = 0x1")
        self.write_cmd("RW JTAG-REG, IR={6'h0c,2'b10}(addr:('h0c)), DR data:(40'h0000000040)")
        self.write_comment("User issues bist WRITE command")
        self.write_cmd("RW JTAG-REG, IR={6'h06,2'b10}(addr:('h06)), DR data:(40'h0000605480)")
        self.write_comment("Read out bit[6]=bist_write_status_ip0 (1: fail, 0: pass), bit[0] = bist_busy (1: busy, 0: idle)")
        # check this return address
        self.write_wait_tdo()
        self.write_cmd("RW JTAG-REG, IR={6'h0b,2'b10}(addr:('h0b)), DR data:(40'h000000000a), expected DR data:(40'h000000000a)")

    def verify_rram(self, address, checkdata, bank):
        x_address = (address & 0b11_1111_1111_1100_0000_0000) >> 10
        y_address = (address & 0b11_1110_0000) >> 5
        self.set_bank(bank)
        self.write_testname(f'Verify single word w/ECC ON at address 0x{address:x} bank{bank}')
        self.write_cmd("RW JTAG-REG, IR={6'h03,2'b10}(addr:('h03)), DR data:(40'h0000000000)")
        self.write_cmd("RW JTAG-REG, IR={6'h04,2'b10}(addr:('h04)), DR data:(40'h0000000000)")
        self.write_cmd("RW JTAG-REG, IR={6'h02,2'b10}(addr:('h02)), DR data:(40'h0000000001)")
        self.write_cmd("RW JTAG-REG, IR={6'h09,2'b10}(addr:('h09)), DR data:(40'h0000000410)")
        self.write_cmd("RW JTAG-REG, IR={6'h02,2'b10}(addr:('h02)), DR data:(40'h0000000000)")
        self.write_cmd("RW JTAG-REG, IR={6'h09,2'b10}(addr:('h09)), DR data:(40'h0000000411)")
        self.write_cmd("RW JTAG-REG, IR={6'h06,2'b10}(addr:('h06)), DR data:(40'h0000600080)")
        self.write_cmd("RW JTAG-REG, IR={6'h03,2'b10}(addr:('h03)), DR data:(40'h0000000000)")
        self.write_cmd("RW JTAG-REG, IR={6'h04,2'b10}(addr:('h04)), DR data:(40'h0000000004)")
        self.write_cmd("RW JTAG-REG, IR={6'h02,2'b10}(addr:('h02)), DR data:(40'h0018000000)")
        self.write_cmd("RW JTAG-REG, IR={6'h09,2'b10}(addr:('h09)), DR data:(40'h0000000410)")
        self.write_cmd("RW JTAG-REG, IR={6'h02,2'b10}(addr:('h02)), DR data:(40'h0000000001)")
        self.write_cmd("RW JTAG-REG, IR={6'h09,2'b10}(addr:('h09)), DR data:(40'h0000000411)")
        self.write_cmd("RW JTAG-REG, IR={6'h06,2'b10}(addr:('h06)), DR data:(40'h0000600080)")
        self.write_comment("select main array/INFO/redundancy/INFO1 area for write")
        self.write_cmd("RW JTAG-REG, IR={6'h07,2'b10}(addr:('h07)), DR data:(40'h0000000002)")
        # read addresses
        bank_address = (x_address & 0xFFFF) | ((y_address & 0xFF) << 16)
        self.write_comment(f"User inputs READ address = 0x{bank_address:x} (X address=0x{x_address:x}, Y address=0x{y_address:x}, bank{bank}) (DR bit[23:0])")
        self.write_cmd(f"RW JTAG-REG, IR={{6'h04,2'b10}}(addr:('h04)), DR data:(40'h{bank_address:010x})")
        # issue read command
        self.write_comment(f"User inputs READ # of loop=0x0")
        self.write_cmd(f"RW JTAG-REG, IR={{6'h10,2'b10}}(addr:('h10)), DR data:(40'h0000000000)")
        self.write_comment(f"User inputs address and data pattern, (data_option needs to be 0x6 = DATA_LATCH to output RRAM DOUT to TDO)")
        self.write_cmd(f"RW JTAG-REG, IR={{6'h03,2'b10}}(addr:('h03)), DR data:(40'h0000000060)")
        self.write_comment(f"User sets bist_read_status_ip0_en = 0x1")
        self.write_cmd(f"RW JTAG-REG, IR={{6'h0c,2'b10}}(addr:('h0c)), DR data:(40'h0000000400)")
        self.write_comment(f"User issues bist READ command")
        self.write_cmd(f"RW JTAG-REG, IR={{6'h06,2'b10}}(addr:('h06)), DR data:(40'h000060c880)")
        self.write_comment(f"Read out bit[10]=bist_read_status_ip0 (1: fail, 0: pass), bit[0] = bist_busy (1: busy, 0: idle)")
        self.write_wait_tdo()
        self.write_cmd(f"RW JTAG-REG, IR={{6'h0b,2'b10}}(addr:('h0b)), DR data:(40'h000000000a), expected DR data:(40'h000000000a)")
        # get read data
        for rd_ptr in range(0, 16, 4):
            self.write_comment(f"set debug_bist_grpsel=0x0, R_BIST_debug_bist_grpdata[31:0] = macro_dout[{(rd_ptr+4) * 8 - 1}:{rd_ptr * 8}]")
            checkaddr = 0x2000 | (rd_ptr // 4) << 16
            self.write_cmd(f"RW JTAG-REG, IR={{6'h14,2'b10}}(addr:('h14)), DR data:(40'h{checkaddr:010x})")
            self.write_comment(f"User outpus R_BIST_debug_bist_grpdata[{(rd_ptr+1) * 8 - 1}:{rd_ptr * 8}] to TDO during DRSHIFT")
            checkword = int.from_bytes(checkdata[rd_ptr:rd_ptr + 4], 'little')
            self.write_cmd(f"RW JTAG-REG, IR={{6'h15,2'b10}}(addr:('h15)), DR data:(40'h0000000000), expected DR data:(40'h{checkword:010x})")


def main():
    global USE_GPIO
    global TCK_pin, TMS_pin, TDI_pin, TDO_pin, PRG_pin
    global jtag_legs, jtag_results_r0, jtag_results_r1
    if USE_GPIO:
        global gpio_pointer, pins, gpioffi
    global tex_mode
    global compat
    global use_key, nky_key, nky_iv, nky_hmac, use_fuzzer

    if USE_GPIO:
        GPIO.setwarnings(False)

    parser = argparse.ArgumentParser(description="Drive JTAG via Rpi GPIO")
    parser.add_argument(
        "-f", "--file", required=True, help="a file ending in .jtg, .bin, or .tex. Operation is dispatched based on the extension: manual jtag commands for .jtg, firmware upload for .bin, and CP testing for .tex", type=str
    )
    parser.add_argument(
        "-c", "--compat", default=False, action="store_true", help="Use compatibility mode (warning: about 100x slower than FFI)"
    )
    parser.add_argument(
        "-d", "--debug", help="turn on debugging spew", default=False, action="store_true"
    )
    parser.add_argument(
        '--tdi', type=int, help="Specify RRAM0 TDI GPIO. Defaults to 27", default=27
    )
    parser.add_argument(
        '--tdo', type=int, help="Specify RRAM0 TDO GPIO. Defaults to 22", default=22
    )
    parser.add_argument(
        '--tms', type=int, help="Specify RRAM0 TMS GPIO. Defaults to 17", default=17
    )
    parser.add_argument(
        '--tdi-r1', type=int, help="Specify RRAM1 TDI GPIO. Defaults to 9", default=9
    )
    parser.add_argument(
        '--tdo-r1', type=int, help="Specify RRAM1 TDO GPIO. Defaults to 11", default=11
    )
    parser.add_argument(
        '--tms-r1', type=int, help="Specify RRAM1 TMS GPIO. Defaults to 10", default=10
    )
    parser.add_argument(
        '--tck', type=int, help="Specify TCK GPIO. Defaults to 4", default=4
    )
    parser.add_argument(
        '--prg', type=int, help="Specify PRG (prog) GPIO. Defaults to 24", default=24
    )
    parser.add_argument(
        '--trst', type=int, help="Specify TRST GPIO. Defaults to 24", default=24
    )
    parser.add_argument(
        '--dbg_p', type=int, help="Specify TRST GPIO. Defaults to 24", default=14
    )
    parser.add_argument(
        "-r", "--reset-prog", help="Pull the PROG pin before initiating any commands", default=False, action="store_true"
    )
    parser.add_argument(
        "-b", "--bank", type=int, help="Specify which bank to use. Default to 0", default=0
    )

    args = parser.parse_args()
    if args.debug:
        logging.basicConfig(format="%(asctime)s [%(levelname)s] %(message)s",handlers=[logging.FileHandler("./logs/" + args.file + ".log"),logging.StreamHandler()], level=logging.DEBUG)
    else:
        logging.basicConfig(format="%(asctime)s [%(levelname)s] %(message)s",handlers=[logging.FileHandler("./logs/" + args.file + ".log"),logging.StreamHandler()], level=logging.INFO)

    ifile = args.file
    compat = args.compat

    TCK_pin = args.tck
    TDI_pin = args.tdi
    TDO_pin = args.tdo
    TMS_pin = args.tms
    PRG_pin = args.prg
    TDI_R1_pin = args.tdi_r1
    TDO_R1_pin = args.tdo_r1
    TMS_R1_pin = args.tms_r1
    DBG_pin = args.dbg_p
    bank = args.bank

    assert bank <= BANK_IPT, "a correct bank should be given"

    if USE_GPIO:
        rev = GPIO.RPI_INFO
        print("revision is: " + str(rev["P1_REVISION"]))
        if rev['P1_REVISION'] == 1:
            gpio_pointer = gpioffi.pi_mmio_init(0x20000000)
        elif rev['P1_REVISION'] == 3 or rev['P1_REVISION'] == 2:
            gpio_pointer = gpioffi.pi_mmio_init(0x3F000000)
            gpioffi.gpioInitialise()
            gpioffi.stopClock()
        elif rev['P1_REVISION'] == 4:
            gpio_pointer = gpioffi.pi_mmio_init(0xFE000000)
        else:
            logging.warning("Unknown Raspberry Pi rev, can't set GPIO base")
            compat = True

        GPIO.setmode(GPIO.BCM)

        GPIO.setup((TCK_pin, TMS_R0_pin, TDI_R0_pin), GPIO.OUT)
        GPIO.setup((TCK_pin, TMS_R1_pin, TDI_R1_pin), GPIO.OUT)
        GPIO.setup((TCK_pin, TMS_IPT_pin, TDI_IPT_pin), GPIO.OUT)
        GPIO.setup(TDO_R0_pin, GPIO.IN)
        GPIO.setup(TDO_R1_pin, GPIO.IN)
        GPIO.setup(TDO_IPT_pin, GPIO.IN)
        GPIO.setup(PRG_pin, GPIO.OUT)
        GPIO.setup(DBG_pin, GPIO.OUT)

        GPIO.output(PRG_pin, 1)
        GPIO.output(DBG_pin, 0)

    # convert 'bin' to 'tex'
    if ifile.endswith('.bin'):
        with open(ifile, 'rb') as ibin:
            with open(ifile[:-4] + '.tex', 'w') as obin:
                tex_writer = TexWriter(obin)
                binary = ibin.read()
                assert len(binary) % (256 // 8) == 0, "Input file must be padded to a 32-byte boundary"
                rd_ptr = 0
                # per spec:

                # string MEMFILE = "reram_simcode.bin";
                # fd = $fopen(MEMFILE,"rb");
                # for(i = 0; (i < MEMDEPTH ) ; i = i + 1)
                # begin
                #     r0 = $fread(data0,fd);
                #     r1 = $fread(data1,fd);
                #     r2 = $fread(data2,fd);
                #     r3 = $fread(data3,fd);
                #     reram_source = {swizzle(data3),swizzle(data2),swizzle(data1),swizzle(data0)};
                #     reram_data0[i] = reram_source[127:0]  // wdata for IP0
                #     reram_data1[i] = reram_source[255:128]    // wdata for IP1
                # end
                #
                # Significantly, the spec does *not* declare the endianness of data0, but
                # reading the source code it seems to be:
                #    reg  [63:0]  data0
                #
                # Thus an `$fread(data0,fd)` into `data0` will take the first 64 bits out
                # of the stream and assign it in big-endian order to data0:
                #   0b1100_0000_0000_0000_0000_0000_0000_0000_0000_0000_0000_0000_0000_0000_0000_0000
                # will yield
                #   data0: u64 = 0xC000_0000_0000_0000
                #
                # the `swizzle` function, which is also not documented in the spec,
                # seems to do a swap of data across an 8-byte word:
                #
                # parameter DATAWIDTH = 64;
                # localparam DATA_BYTES = 8;
                # //Function to account for the Big Endianness of $fread
                # function [DATAWIDTH-1:0] swizzle;
                #     input [DATAWIDTH-1:0] data_in;
                #     integer i;
                #     begin
                #         for (i=0; i<DATA_BYTES; i=i+1)
                #         swizzle[i*8 +:8] = data_in[(DATA_BYTES-1-i)*8 +:8];
                #     end
                # endfunction
                #
                # so swizzle[7:0] = data_in[63:56], swizzle[15:8] = data_in[55:48], etc.
                #
                # Thus the formatting of reram_source is:
                #
                # .bin offset  |  description
                # 0x00         |  reram_data0[7:0]
                # 0x01         |  reram_data0[15:8]
                #  ...
                # 0x0F         |  reram_data0[127:120]
                # 0x10         |  reram_data1[7:0]
                #  ...
                # 0x1E         |  reram_data1[127:120]
                #
                # per the data0 = 0xC000_0000_0000_0000 example above, assuming
                # all other bits are 0, reram_data0 would be:
                #   reram_data0: u128 = 0x0000_0000_0000_0000_0000_0000_0000_00C0

                # stride through the input file in 256-bit chunks: 32*8 = 256
                for rd_ptr in range(0, len(binary), 32):
                    # split into the two 128-bit sections
                    raw_data0 = binary[rd_ptr : rd_ptr + 16]
                    raw_data1 = binary[rd_ptr + 16 : rd_ptr + 32]
                    # prep the storage to receive data
                    reram_data0 = bytearray(16)
                    reram_data1 = bytearray(16)
                    # read in the bytes
                    for index, b in enumerate(raw_data0):
                        reram_data0[index] = b
                    for index, b in enumerate(raw_data1):
                        reram_data1[index] = b

                    tex_writer.write_rram(rd_ptr, reram_data0, BANK_RRAM0)
                    tex_writer.write_rram(rd_ptr + 16, reram_data1, BANK_RRAM1)

                    tex_writer.verify_rram(rd_ptr, reram_data0, BANK_RRAM0)
                    tex_writer.verify_rram(rd_ptr + 16, reram_data1, BANK_RRAM1)

    # assume CP test if a .tex file is specified
    elif ifile.endswith('tex'):
        tex_mode = True
        # process tex format
        cmds = 0
        # first "test" is a reset command to the scan chain
        reset_jtag_group = JtagGroup()
        reset_jtag_group.test['legs'] = [[JtagLeg.RS, '0', '0']] # special-case override to insert a reset command
        reset_jtag_cp_test = CpTest('Reset JTAG', bank)
        reset_jtag_cp_test.append_group(reset_jtag_group)

        cp_tests = [reset_jtag_cp_test]
        current_test = None
        current_group = None
        current_serial_no = None
        testname = 'Uninit'
        #bank = BANK_RRAM0 # default
        with open(ifile) as f:
            for line in f:
                if line.startswith('#'):
                    cmd = line.split(', ')
                    if len(cmd) != 5 and len(cmd) != 4:
                        logging.error(f".tex command did not conform to expected format! Ignoring: {line}")
                    else:
                        if bank == BANK_IPT:
                            # extract serial number, and check it
                            new_serial_no = int(cmd[0].lstrip('#'))
                            if current_serial_no is not None:
                                assert new_serial_no == current_serial_no + 1, "Test serial numbers are not uniformly increasing!"
                            current_serial_no = new_serial_no

                            # track start-to-end serials, mostly for debugging
                            if not current_group.has_start():
                                current_group.set_start(current_serial_no)
                            # start == end if there is just one test!
                            # i think we can set end test when we see a =========
                            current_group.set_end(current_serial_no) # we keep updating the "end" test with the last test seen. Not efficient but easy...
                            assert cmd[1] == 'RW JTAG-REG', "Unexpected JTAG command, this is a hard error. Script needs to be updated to handle all commands!"

                            # parse the IR, add it to the JTAG legs
                            ir_regex = re.compile("IR=\((.*)\)")
                            ir_hex = ir_regex.match(cmd[2])[1]
                            current_group.append_leg([JtagLeg.IR, '%0*d' % (5, int(bin(int(ir_hex.lstrip("5'h"), 16))[2:])), ' '])

                            # parse the DR going from host to chip, add it to the JTAG legs
                            dr_regex = re.compile("DR data:\(61'h([0-9A-Fa-f]*)\)") # very narrow rule because we want to catch if the value isn't 61'h
                            dr_hex = dr_regex.match(cmd[3])[1]
                            current_group.append_leg([JtagLeg.DR, '%0*d' % (61, int(bin(int(dr_hex, 16))[2:])), ' ']) # note the hard-coded 40-bit length here!

                            if len(cmd) == 5:
                                # parse the DR coming from chip to host, add it to the dr_vals tracker
                                dre_regex = re.compile("expected DR data:\(61'h([0-9A-Fa-f]*)\)") # very narrow rule because we want to catch if the value isn't 40'h
                                dre_hex = dre_regex.match(cmd[4])[1]
                                current_group.add_dr_val(current_serial_no, int(dre_hex, 16))
                            else:
                                current_group.add_dr_val(current_serial_no, 'x')

                            current_group.add_dbg_state(current_serial_no, current_group.get_debug())
                            # track total commands seen as a sanity check
                            cmds += 1
                        else:
                            # extract serial number, and check it
                            new_serial_no = int(cmd[0].lstrip('#'))
                            if current_serial_no is not None:
                                assert new_serial_no == current_serial_no + 1, "Test serial numbers are not uniformly increasing!"
                            current_serial_no = new_serial_no

                            # track start-to-end serials, mostly for debugging
                            if not current_group.has_start():
                                current_group.set_start(current_serial_no)
                            # start == end if there is just one test!
                            # i think we can set end test when we see a =========
                            current_group.set_end(current_serial_no) # we keep updating the "end" test with the last test seen. Not efficient but easy...
                            assert cmd[1] == 'RW JTAG-REG', "Unexpected JTAG command, this is a hard error. Script needs to be updated to handle all commands!"

                            # parse the IR, add it to the JTAG legs
                            ir_regex = re.compile('IR=\{(.*),(.*)\}')
                            ir_hex = ir_regex.match(cmd[2])[1]
                            # if ir_regex.match(cmd[2])[2] != "2'b10":
                            #     logging.debug(line)
                            # assert ir_regex.match(cmd[2])[2] == "2'b10", "IR second argument was not 2'b10: extend the parser to handle this case"
                            ir_val = int(ir_hex.lstrip("6'h"), base=16)
                            ir_val <<= 2
                            # ir_val |= 0b10 # easy enough to extend the parser to handle other cases, but since we only ever see 2'b10 in practice...
                            ir_val += int(ir_regex.match(cmd[2])[2].lstrip("2'b"), base=2)

                            current_group.append_leg([JtagLeg.IR, '%0*d' % (8, int(bin(ir_val)[2:])), ' ']) # note hard-coded value of 6 for the IR length!

                            # parse the DR going from host to chip, add it to the JTAG legs
                            dr_regex = re.compile("DR data:\(40'h([0-9a-f]*)\)") # very narrow rule because we want to catch if the value isn't 40'h
                            dr_hex = dr_regex.match(cmd[3])[1]
                            current_group.append_leg([JtagLeg.DR, '%0*d' % (40, int(bin(int(dr_hex, 16))[2:])), ' ']) # note the hard-coded 40-bit length here!
                            if len(cmd) == 5:
                                # parse the DR coming from chip to host, add it to the dr_vals tracker
                                dre_regex = re.compile("expected DR data:\(40'h([0-9a-f]*)\)") # very narrow rule because we want to catch if the value isn't 40'h
                                dre_hex = dre_regex.match(cmd[4])[1]
                                current_group.add_dr_val(current_serial_no, int(dre_hex, 16))
                            else:
                                current_group.add_dr_val(current_serial_no, 'x')

                            current_group.add_dbg_state(current_serial_no, current_group.get_debug())
                            # track total commands seen as a sanity check
                            cmds += 1
                elif line.startswith('@@@'):
                    testname = line.strip()
                    if current_test is not None:
                        if current_group is not None:
                            current_test.append_group(current_group)
                        cp_tests += [current_test]
                    current_test = CpTest(testname, bank)
                    current_group = JtagGroup()
                elif line.startswith('Wait'):
                    # set a wait condition, which ends the group.
                    current_group.set_wait(line.strip())
                    current_test.append_group(current_group)
                    # create a new group for future commands
                    current_group = JtagGroup()
                elif line.startswith('Check'):
                    if current_group.has_check(): # if a check already exists, terminate the group and start a new one
                        current_test.append_group(current_group)
                        current_group = JtagGroup()
                    current_group.set_check(line.strip())
                elif line.startswith('!bank'):
                   bank = int(line.strip().split(' ')[1])
                elif line.startswith('!trigger_h'):
                   current_group.set_debug()
                elif line.startswith('!trigger_l'):
                   current_group.clear_debug()
                else:
                    # placeholder for doing stuff with comments, etc.
                    pass

        # capture the last group
        current_test.append_group(current_group)
        cp_tests += [current_test]

        logging.info(f"found {cmds} commands")
        hard_errors = 0
        DR_errors = 0

        for test in cp_tests:
            logging.info(f"Running {test.name}")
            set_bank(test.bank)
            for jtg in test.groups:
                if jtg.has_start():
                    logging.info(f"  Group {jtg.get_start()}-{jtg.get_end()}")
                # gets gross here, because this is assigned to a global variable, jtag_legs...
                jtag_legs = jtg.get_legs()
                test_index = jtg.get_start()

                if jtg.has_check() and len(jtag_legs) == 0:
                    logging.error("Check statement found, but no JTAG legs were defined to run the check on!")
                first_leg = True
                while len(jtag_legs) > 0:
                        if state == JtagState.TEST_LOGIC_RESET or state == JtagState.RUN_TEST_IDLE:
                            if len(jtag_legs):

                                gpioffi.stopClock()

                                if test_index > 0:
                                    if jtg.lookup_dbg_state(test_index):
                                        gpioffi.set_dbg(pins[0], gpio_pointer)
                                    else:
                                        gpioffi.clear_dbg(pins[0], gpio_pointer)

                                if jtg.has_start():
                                    expected_data = jtg.lookup_dr_val(test_index)
                                    test_index += 1
                                else:
                                    expected_data = None
                                logging.debug(f"Running leg {jtag_legs[0]}")
                                if jtag_legs[0][0] == JtagLeg.RS:
                                    jtag_step()
                                else:
                                    # stash a copy of the commands to repeat in case we are in a check loop
                                    check_wait_cmds = []
                                    # can't just use 'copy' because the references are destroyed
                                    check_wait_cmds += [[jtag_legs[0][0], jtag_legs[0][1], jtag_legs[0][2]]]
                                    check_wait_cmds += [[jtag_legs[1][0], jtag_legs[1][1], jtag_legs[1][2]]]

                                    # run until out of idle
                                    while state == JtagState.TEST_LOGIC_RESET or state == JtagState.RUN_TEST_IDLE:
                                        jtag_step()

                                    # run to idle
                                    logging.debug(f"Finishing leg {jtag_legs[0]}")
                                    while state != JtagState.TEST_LOGIC_RESET and state != JtagState.RUN_TEST_IDLE:
                                        jtag_step()

                                gpioffi.startClock()

                                if expected_data is not None:
                                    # at this point, jtag_result should have our result if we were in a DR leg
                                    # print(jtag_results)

                                    if(test.bank==BANK_RRAM0 or test.bank==BANK_RRAMS or test.bank==BANK_IPT):
                                        assert len(jtag_results_r0) == 2, "Consistency error in number of results returned"
                                    if(test.bank==BANK_RRAM1 or test.bank==BANK_RRAMS):
                                        assert len(jtag_results_r1) == 2, "Consistency error in number of results returned on td0 rram1"

                                    if(test.bank==BANK_RRAM0 or test.bank==BANK_RRAMS or test.bank==BANK_IPT):
                                        if expected_data == 'x':
                                            logging.info(f"    Test:{test_index-1} Passed! Ignoring TDO")  
                                        elif jtag_results_r0[1] != expected_data:
                                            logging.error(f"    Test:{test_index-1} Failed! Expected: 0x{expected_data:x} != result: 0x{jtag_results_r0[1]:x}")
                                            DR_errors += 1
                                        else:
                                            logging.info(f"    Test:{test_index-1} Passed! Expected: 0x{expected_data:x} = result: 0x{jtag_results_r0[1]:x}")
                                    if(test.bank==BANK_RRAM1 or test.bank==BANK_RRAMS):
                                        if expected_data == 'x':
                                            logging.info(f"    Test:{test_index-1} Passed! Ignoring TDO") 
                                        elif jtag_results_r1[1] != expected_data:
                                            logging.error(f"    Test:{test_index-1} Failed! Expected: 0x{expected_data:x} != result: 0x{jtag_results_r1[1]:x}")
                                            DR_errors += 1
                                        else:
                                            logging.info(f"    Test:{test_index-1} Passed! Expected: 0x{expected_data:x} = result: 0x{jtag_results_r1[1]:x}")
                                    if first_leg:
                                        if jtg.has_check():
                                            check_statement = jtg.get_check()
                                            if '[X]' not in check_statement:
                                                # all check statements want to confirm that the busy bit is not set (bit 1)
                                                start_time = time.time()
                                                if(test.bank==BANK_RRAM0 or test.bank==BANK_RRAMS or test.bank==BANK_IPT):
                                                    while (jtag_results_r0[1] & 1) != 0:
                                                        logging.warning(f"    BIST is still busy, trying again")
                                                        if time.time() - start_time > TIMEOUT_S:
                                                            logging.error("    TIMEOUT FAILURE waiting for BIST to go idle. Test failed.")
                                                            hard_errors += 1
                                                            break
                                                        # can't point to the reference in check_wait_cmds because the references are destroyed
                                                        jtag_legs.insert(0, [check_wait_cmds[1][0], check_wait_cmds[1][1], check_wait_cmds[1][2]]) # insert the DR
                                                        jtag_legs.insert(0, [check_wait_cmds[0][0], check_wait_cmds[0][1], check_wait_cmds[0][2]]) # insert the IR
                                                        # reset jtag_results
                                                        jtag_results_r0 = []
                                                        # rerun the poll loop
                                                        while state == JtagState.TEST_LOGIC_RESET or state == JtagState.RUN_TEST_IDLE:
                                                            jtag_step()
                                                        while state != JtagState.TEST_LOGIC_RESET and state != JtagState.RUN_TEST_IDLE:
                                                            jtag_step()
                                                    logging.info(f"    Check Test:{test_index-1} Passed! Expected: 0x{expected_data:x} = result: 0x{jtag_results_r0[1]:x}")
                                                elif(test.bank==BANK_RRAM1):
                                                    while (jtag_results_r1[1] & 1) != 0:
                                                        logging.warning(f"    BIST is still busy, trying again")
                                                        if time.time() - start_time > TIMEOUT_S:
                                                            logging.error("    TIMEOUT FAILURE waiting for BIST to go idle. Test failed.")
                                                            hard_errors += 1
                                                            break
                                                        # can't point to the reference in check_wait_cmds because the references are destroyed
                                                        jtag_legs.insert(0, [check_wait_cmds[1][0], check_wait_cmds[1][1], check_wait_cmds[1][2]]) # insert the DR
                                                        jtag_legs.insert(0, [check_wait_cmds[0][0], check_wait_cmds[0][1], check_wait_cmds[0][2]]) # insert the IR
                                                        # reset jtag_results
                                                        jtag_results_r1 = []
                                                        # rerun the poll loop
                                                        while state == JtagState.TEST_LOGIC_RESET or state == JtagState.RUN_TEST_IDLE:
                                                            jtag_step()
                                                        while state != JtagState.TEST_LOGIC_RESET and state != JtagState.RUN_TEST_IDLE:
                                                            jtag_step()
                                                    logging.info(f"    Check Test:{test_index-1} Passed! Expected: 0x{expected_data:x} = result: 0x{jtag_results_r1[1]:x}")
                                            else:
                                                if(test.bank==BANK_RRAM0 or test.bank==BANK_RRAMS or test.bank==BANK_IPT):
                                                    if jtag_results_r0[1] != expected_data:
                                                        logging.error(f"    All-bit check of DR return value failed.")
                                                        hard_errors += 1
                                                if(test.bank==BANK_RRAM1 or test.bank==BANK_RRAMS):
                                                    if jtag_results_r1[1] != expected_data:
                                                        logging.error(f"    All-bit check of DR return value failed.")
                                                        hard_errors += 1                                                       

                                        first_leg = False

                                    jtag_results_r0 = []
                                    jtag_results_r1 = []


                            else:
                                # this should do nothing
                                jtag_step()
                        else:
                            # we're in a leg, run to idle
                            while state != JtagState.TEST_LOGIC_RESET and state != JtagState.RUN_TEST_IDLE:
                                jtag_step()

                # no more legs, check and see if we had a wait or check condition!
                if jtg.has_wait():
                    # TODO: implement the wait routine. This is just a placeholder that is a guess
                    # assert 'JTAG-TDO to fall' in jtg.get_wait(), "The wait condition did not match our expectation"
                    if 'JTAG-TDO to fall' in jtg.get_wait():
                        start_time = time.time()
                        if USE_GPIO:
                            while GPIO.input(TDO_pin) != 0:
                                if time.time() - start_time > TIMEOUT_S:
                                    logging.error("    TIMEOUT FAILURE waiting for JTAG-TDO to fall. Test failed.")
                                    hard_errors += 1
                                    break
                        else:
                            logging.debug("Would wait for TDO_pin to go high")
                    elif 'seconds' in jtg.get_wait():

                        start_time = time.time()

                        wait = jtg.get_wait().split(' ')
                        assert 3 ==len(wait), "the wait condition did not match our expectations"

                        print("waiting for " + wait[1] + " seconds")
                        stop_time = start_time + int(wait[1])
                        if USE_GPIO:
                            time.sleep(int(wait[1]))
                        else:
                            logging.debug("Would wait for TDO_pin to go high")
                    elif 'for keypress' in jtg.get_wait():

                        start_time = time.time()

                        wait = jtg.get_wait().split(' ')
                        assert 3 ==len(wait), "the wait condition did not match our expectations"

                        input("press enter to continue...")
                    else:
                        assert "no matching wait"

        logging.info(f"Test finished with {hard_errors} Check[x] or timeout errors")
        logging.info(f"Test finished with {DR_errors} expected DR errors")

    elif ifile.endswith('jtg'):
        # CSV file format
        # chain, width, value:
        # IR, 6, 0b110110
        # DR, 64, 0x0
        with open(ifile) as csvfile:
            reader = csv.reader(csvfile, delimiter=',')

            for row in reader:
                if len(row) < 3:
                    continue
                chain = str(row[0]).lower().strip()
                if chain[0] == '#':
                    continue
                length = int(row[1])
                if str(row[2]).strip()[:2] == '0x':
                    value = int(row[2], 16)
                elif str(row[2]).strip()[:2] == '0b':
                    value = int(row[2], 2)
                else:
                    value = int(row[2])

                if (chain != 'dr') & (chain != 'ir') & (chain != 'rs') & (chain != 'dl') & \
                (chain != 'id') & (chain != 'irp') & (chain != 'ird') & (chain != 'drc') & \
                (chain != 'drr') & (chain != 'drs'):
                    print('unknown chain type ', chain, ' aborting!')
                    if USE_GPIO:
                        GPIO.cleanup()
                    exit(1)

                # logging.debug('found JTAG chain ', chain, ' with len ', str(length), ' and data ', hex(value))
                if chain == 'rs':
                    jtag_legs.append([JtagLeg.RS, '0', '0'])
                elif chain == 'dl':
                    jtag_legs.append([JtagLeg.DL, '0', '0'])
                elif chain == 'id':
                    jtag_legs.append([JtagLeg.ID, '0', '0'])

                else:
                    if chain == 'dr':
                        code = JtagLeg.DR
                    elif chain == 'drc':
                        code = JtagLeg.DRC
                    elif chain == 'drr':
                        code = JtagLeg.DRR
                    elif chain == 'drs':
                        code = JtagLeg.DRS
                    elif chain == 'ir':
                        code = JtagLeg.IR
                    elif chain == 'ird':
                        code = JtagLeg.IRD
                    else:
                        code = JtagLeg.IRP
                    if len(row) > 3:
                        jtag_legs.append([code, '%0*d' % (length, int(bin(value)[2:])), row[3]])
                    else:
                        jtag_legs.append([code, '%0*d' % (length, int(bin(value)[2:])), ' '])
        # logging.debug(jtag_legs)
    else:
        logging.error("Input file should end with one of .jtg, .tex, or .bin")
        logging.error("  - .jtg files contain manual JTAG debugging commands")
        logging.error("  - .tex files contain CP testing commands in TSMC format")
        logging.error("  - .bin files are a binary image for uploading to RRAM")

    if args.reset_prog:
        reset_fpga()
    while len(jtag_legs):
        # time.sleep(0.002) # give 2 ms between each command
        jtag_next()

    if USE_GPIO:
        GPIO.cleanup((TCK_pin, TMS_R0_pin, TDI_R0_pin, TMS_R1_pin, TDI_R1_pin, TMS_IPT_pin, TDI_IPT_pin, TDO_R0_pin, TDO_R1_pin, TDO_IPT_pin, DBG_pin))
        GPIO.setup((TCK_pin, PRG_pin), GPIO.IN)
    exit(0)

from typing import Any, Iterable, Mapping, Optional, Set, Union
def int_to_bytes(x: int) -> bytes:
    if x != 0:
        return x.to_bytes((x.bit_length() + 7) // 8, 'big')
    else:
        return bytes(1)  # a length 1 bytes with value of 0

if __name__ == "__main__":
    main()

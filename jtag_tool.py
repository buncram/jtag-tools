#!/usr/bin/python3

try:
    import RPi.GPIO as GPIO
except RuntimeError:
    print("Error importing RPi.GPIO! Did you run as root?")

import csv
import argparse
import time
import logging
import sys
import re

from enum import Enum
from cffi import FFI

from pathlib import Path

TIMEOUT_S = 5  # default timeout, in seconds

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
        }
        self.dr_vals = {} # dictionary of test serial numbers to expected DR values

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
    # Thus each group can only have at most one check conditio, and it is always executed
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

# CpTest is a structure for collecting several groups together into a single named test.
class CpTest():
    def __init__(self, test_name):
        self.name = test_name
        self.groups = []

    def append_group(self, group):
        self.groups += [group]

ffi = FFI()
ffi.cdef("""
typedef struct pindefs pindefs;
struct pindefs {
  uint32_t tck;
  uint32_t tms;
  uint32_t tdi;
  uint32_t tdo;
  uint32_t trst;
};

volatile uint32_t *pi_mmio_init(uint32_t base);
int jtag_pins(int tdi, int tms, pindefs pins, volatile uint32_t *gpio);
int jtag_prog(char *bitstream, pindefs pins, volatile uint32_t *gpio);
void jtag_prog_rbk(char *bitstream, pindefs pins, volatile uint32_t *gpio, char *readback);
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

TCK_pin = 4
TMS_pin = 17
TDI_pin = 27  # TDI on FPGA, out for this script
TDO_pin = 22  # TDO on FPGA, in for this script
PRG_pin = 24
tex_mode = False

pins = ffi.new("struct pindefs *")
pins.tck = TCK_pin
pins.tms = TMS_pin
pins.tdi = TDI_pin
pins.tdo = TDO_pin
pins.trst = PRG_pin
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
tdo_vect = ''
tdo_stash = ''
jtag_results = []
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
    global TCK_pin, TMS_pin, TDI_pin, TDO_pin, pins, gpio_pointer, gpioffi

    if compat:
        tdo = GPIO.input(TDO_pin) # grab the TDO value before the clock changes

        GPIO.output( (TCK_pin, TDI_pin, TMS_pin), (0, tdi, tms) )
        GPIO.output( (TCK_pin, TDI_pin, TMS_pin), (1, tdi, tms) )
        GPIO.output( (TCK_pin, TDI_pin, TMS_pin), (0, tdi, tms) )
    else:
        tdo = gpioffi.jtag_pins(tdi, tms, pins[0], gpio_pointer)

    return tdo

def reset_fpga():
    global PRG_pin

    GPIO.output(PRG_pin, 0)
    time.sleep(0.1)
    GPIO.output(PRG_pin, 1)


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
    global jtag_results
    global tdo_vect, tdo_stash
    global do_pause
    global TCK_pin, TMS_pin, TDI_pin, TDO_pin
    global gpio_pointer, gpioffi, pins
    global keepalive
    global compat
    global readout
    global readdata

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
        tdo_vect = ''  # prep the tdo_vect to receive data
        state = JtagState.SHIFT

    elif state == JtagState.SHIFT:
        if cur_leg[0] == JtagLeg.DRC or cur_leg[0] == JtagLeg.DRS:
            if cur_leg[0] == JtagLeg.DRC: # duplicate code because we want speed (eliminating TDO readback is significant speedup)
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
            else:  # jtagleg is DRS -- duplicate code, as TDO readback slows things down significantly
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
                        tdo_vect = '1' + tdo_vect
                    else:
                        tdo_vect = '0' + tdo_vect
                else:
                    bytestr = bytes(cur_leg[1][:-1], 'utf-8')
                    tdo_temp = '0'*len(cur_leg[1][:-1]) # initialize space for tdo_vect
                    retstr = bytes(tdo_temp, 'utf-8')
                    ffi = FFI()
                    ffistr = ffi.new("char[]", bytestr)
                    ffiret = ffi.new("char[]", retstr)
                    keepalive.append(ffistr) # need to make sure the lifetime of the string is long enough for the call
                    keepalive.append(ffiret)
                    gpioffi.jtag_prog_rbk(ffistr, pins[0], gpio_pointer, ffiret)
                    tdo_vect = ffi.string(ffiret).decode('utf-8')

            state = JtagState.SHIFT

            if cur_leg[-1:] == '1':
                tdi = 1
            else:
                tdi = 0
            cur_leg = ''
            tdo = phy_sync(tdi, 1)
            if tdo == 1:
                tdo_vect = '1' + tdo_vect
            else:
                tdo_vect = '0' + tdo_vect
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
                if tdo == 1:
                    tdo_vect = '1' + tdo_vect
                else:
                    tdo_vect = '0' + tdo_vect
                state = JtagState.SHIFT
            else: # this is the last item
                if cur_leg[1][0] == '1':
                    tdi = 1
                else:
                    tdi = 0
                cur_leg = ''
                tdo = phy_sync(tdi, 1)
                if tdo == 1:
                    tdo_vect = '1' + tdo_vect
                else:
                    tdo_vect = '0' + tdo_vect
                state = JtagState.EXIT1

    elif state == JtagState.EXIT1:
        tdo_stash = tdo_vect
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
        jtag_results.append(int(tdo_vect, 2)) # interpret the vector and save it
        logging.debug("result: %s", str(hex(int(tdo_vect, 2))) )
        if readout:
            #print('readout: 0x{:08x}'.format( int(tdo_vect, 2) ) )
            readdata = int(tdo_vect, 2)
            readout = False
        tdo_vect = ''

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
    global jtag_results

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

def main():
    global TCK_pin, TMS_pin, TDI_pin, TDO_pin, PRG_pin
    global jtag_legs, jtag_results
    global gpio_pointer, pins, gpioffi
    global tex_mode
    global compat
    global use_key, nky_key, nky_iv, nky_hmac, use_fuzzer

    GPIO.setwarnings(False)

    parser = argparse.ArgumentParser(description="Drive JTAG via Rpi GPIO")
    parser.add_argument(
        "-f", "--file", required=True, help="file containing jtag command list or bitstream", type=str
    )
    parser.add_argument(
        "-c", "--compat", default=False, action="store_true", help="Use compatibility mode (warning: about 100x slower than FFI)"
    )
    parser.add_argument(
        "-d", "--debug", help="turn on debugging spew", default=False, action="store_true"
    )
    parser.add_argument(
        '--tdi', type=int, help="Specify TDI GPIO. Defaults to 27", default=27
    )
    parser.add_argument(
        '--tdo', type=int, help="Specify TDO GPIO. Defaults to 22", default=22
    )
    parser.add_argument(
        '--tms', type=int, help="Specify TMS GPIO. Defaults to 17", default=17
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
        "-r", "--reset-prog", help="Pull the PROG pin before initiating any commands", default=False, action="store_true"
    )
    args = parser.parse_args()
    if args.debug:
        logging.basicConfig(stream=sys.stdout, level=logging.DEBUG)
    else:
        logging.basicConfig(stream=sys.stdout, level=logging.INFO)

    ifile = args.file
    compat = args.compat

    TCK_pin = args.tck
    TDI_pin = args.tdi
    TDO_pin = args.tdo
    TMS_pin = args.tms
    PRG_pin = args.prg

    rev = GPIO.RPI_INFO
    if rev['P1_REVISION'] == 1:
        gpio_pointer = gpioffi.pi_mmio_init(0x20000000)
    elif rev['P1_REVISION'] == 3 or rev['P1_REVISION'] == 2:
        gpio_pointer = gpioffi.pi_mmio_init(0x3F000000)
    elif rev['P1_REVISION'] == 4:
        gpio_pointer = gpioffi.pi_mmio_init(0xFE000000)
    else:
        logging.warning("Unknown Raspberry Pi rev, can't set GPIO base")
        compat = True

    GPIO.setmode(GPIO.BCM)

    GPIO.setup((TCK_pin, TMS_pin, TDI_pin), GPIO.OUT)
    GPIO.setup(TDO_pin, GPIO.IN)
    if args.reset_prog:
        GPIO.setup(PRG_pin, GPIO.OUT)

    if ifile.endswith('tex'):
        tex_mode = True
        # process tex format
        cmds = 0
        # first "test" is a reset command to the scan chain
        reset_jtag_group = JtagGroup()
        reset_jtag_group.test['legs'] = [[JtagLeg.RS, '0', '0']] # special-case override to insert a reset command
        reset_jtag_cp_test = CpTest('Reset JTAG')
        reset_jtag_cp_test.append_group(reset_jtag_group)

        cp_tests = [reset_jtag_cp_test]
        current_test = None
        current_group = None
        current_serial_no = None
        testname = 'Uninit'
        with open(ifile) as f:
            for line in f:
                if line.startswith('#'):
                    cmd = line.split(', ')
                    if len(cmd) != 5:
                        logging.error(f".tex command did not conform to expected format! Ignoring: {line}")
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
                        current_group.set_end(current_serial_no) # we keep updating the "end" test with the last test seen. Not efficient but easy...
                        assert cmd[1] == 'RW JTAG-REG', "Unexpected JTAG command, this is a hard error. Script needs to be updated to handle all commands!"

                        # parse the IR, add it to the JTAG legs
                        ir_regex = re.compile('IR=\{(.*),(.*)\}')
                        ir_hex = ir_regex.match(cmd[2])[1]
                        if ir_regex.match(cmd[2])[2] != "2'b10":
                            logging.debug(line)
                        assert ir_regex.match(cmd[2])[2] == "2'b10", "IR second argument was not 2'b10: extend the parser to handle this case"
                        ir_val = int(ir_hex.lstrip("6'h"), base=16)
                        ir_val <<= 2
                        ir_val |= 0b10 # easy enough to extend the parser to handle other cases, but since we only ever see 2'b10 in practice...
                        current_group.append_leg([JtagLeg.IR, '%0*d' % (8, int(bin(ir_val)[2:])), ' ']) # note hard-coded value of 6 for the IR length!

                        # parse the DR going from host to chip, add it to the JTAG legs
                        dr_regex = re.compile("DR data:\(40'h([0-9a-f]*)\)") # very narrow rule because we want to catch if the value isn't 40'h
                        dr_hex = dr_regex.match(cmd[3])[1]
                        current_group.append_leg([JtagLeg.DR, '%0*d' % (40, int(bin(int(dr_hex, 16))[2:])), ' ']) # note the hard-coded 40-bit length here!

                        # parse the DR coming from chip to host, add it to the dr_vals tracker
                        dre_regex = re.compile("expected DR data:\(40'h([0-9a-f]*)\)") # very narrow rule because we want to catch if the value isn't 40'h
                        dre_hex = dre_regex.match(cmd[4])[1]
                        current_group.add_dr_val(current_serial_no, int(dre_hex, 16))

                        # track total commands seen as a sanity check
                        cmds += 1
                elif line.startswith('@@@'):
                    testname = line.strip()
                    if current_test is not None:
                        if current_group is not None:
                            current_test.append_group(current_group)
                        cp_tests += [current_test]
                    current_test = CpTest(testname)
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
                else:
                    # placeholder for doing stuff with comments, etc.
                    pass

        # capture the last group
        current_test.append_group(current_group)
        cp_tests += [current_test]

        logging.info(f"found {cmds} commands")
        hard_errors = 0

        for test in cp_tests:
            logging.info(f"Running {test.name}")
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

                                if expected_data is not None:
                                    # at this point, jtag_result should have our result if we were in a DR leg
                                    # print(jtag_results)
                                    assert len(jtag_results) == 2, "Consistency error in number of results returned"
                                    if jtag_results[1] != expected_data:
                                        logging.warning(f"    Expected data 0x{expected_data:x} != result 0x{jtag_results[1]:x}")
                                    if first_leg:
                                        if jtg.has_check():
                                            check_statement = jtg.get_check()
                                            if '[X]' not in check_statement:
                                                # all check statements want to confirm that the busy bit is not set (bit 1)
                                                start_time = time.time()
                                                while (jtag_results[1] & 1) != 0:
                                                    logging.warning(f"    BIST is still busy, trying again")
                                                    if time.time() - start_time > TIMEOUT_S:
                                                        logging.error("    TIMEOUT FAILURE waiting for BIST to go idle. Test failed.")
                                                        hard_errors += 1
                                                        break
                                                    # can't point to the reference in check_wait_cmds because the references are destroyed
                                                    jtag_legs.insert(0, [check_wait_cmds[1][0], check_wait_cmds[1][1], check_wait_cmds[1][2]]) # insert the DR
                                                    jtag_legs.insert(0, [check_wait_cmds[0][0], check_wait_cmds[0][1], check_wait_cmds[0][2]]) # insert the IR
                                                    # reset jtag_results
                                                    jtag_results = []
                                                    # rerun the poll loop
                                                    while state == JtagState.TEST_LOGIC_RESET or state == JtagState.RUN_TEST_IDLE:
                                                        jtag_step()
                                                    while state != JtagState.TEST_LOGIC_RESET and state != JtagState.RUN_TEST_IDLE:
                                                        jtag_step()
                                            else:
                                                if jtag_results[1] != expected_data:
                                                    logging.error(f"    All-bit check of DR return value failed.")
                                                    hard_errors += 1
                                        first_leg = False
                                    jtag_results = []
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
                    assert 'JTAG-TDO to fall' in jtg.get_wait(), "The wait condition did not match our expectation"
                    start_time = time.time()
                    while GPIO.input(TDO_pin) != 0:
                        if time.time() - start_time > TIMEOUT_S:
                            logging.error("    TIMEOUT FAILURE waiting for JTAG-TDO to fall. Test failed.")
                            hard_errors += 1
                            break
                        else:
                            time.sleep(0.01)

        logging.info(f"CP test finished with {hard_errors} hard errors")

    else:
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

    if args.reset_prog:
        reset_fpga()
    while len(jtag_legs):
        # time.sleep(0.002) # give 2 ms between each command
        jtag_next()

#        while len(jtag_results):
#            result = jtag_result.pop()
            # printout happens in situ

    GPIO.cleanup()
    exit(0)

from typing import Any, Iterable, Mapping, Optional, Set, Union
def int_to_bytes(x: int) -> bytes:
    if x != 0:
        return x.to_bytes((x.bit_length() + 7) // 8, 'big')
    else:
        return bytes(1)  # a length 1 bytes with value of 0

if __name__ == "__main__":
    main()

#!/usr/bin/python3

try:
    import RPi.GPIO as GPIO
except RuntimeError:
    print("Error importing RPi.GPIO! Did you run as root?")

import csv
import argparse
import time
import subprocess
import logging
import sys
import binascii
from Crypto.Cipher import AES

from enum import Enum

from cffi import FFI
try:
    from gpioffi.lib import pi_mmio_init
except ImportError as err:
    print('Module not found ({}), attempting to rebuild...'.format(err))
    subprocess.call(['python3', 'build.py'])
    print('Please try the command again.')
    exit(1)

from gpioffi.lib import jtag_pins
from gpioffi.lib import jtag_prog
from gpioffi.lib import jtag_prog_rbk

keepalive = []
ffi = FFI()
# maxbuf - maximum length, in bits, of a bitstream that can be handled by this script
maxbuf = 20 * 1024 * 1024
ffistr = ffi.new("char[]", bytes(maxbuf))
keepalive.append(ffistr)
ffiret = ffi.new("char[]", bytes(maxbuf))
keepalive.append(ffiret)

TCK_pin = 4
TMS_pin = 17
TDI_pin = 27  # TDI on FPGA, out for this script
TDO_pin = 22  # TDO on FPGA, in for this script
PRG_pin = 24

pins = ffi.new("pindefs *")
pins.tck = TCK_pin
pins.tms = TMS_pin
pins.tdi = TDI_pin
pins.tdo = TDO_pin
pins.trst = PRG_pin
keepalive.append(pins)

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
    global TCK_pin, TMS_pin, TDI_pin, TDO_pin

    if compat:
        tdo = GPIO.input(TDO_pin) # grab the TDO value before the clock changes

        GPIO.output( (TCK_pin, TDI_pin, TMS_pin), (0, tdi, tms) )
        GPIO.output( (TCK_pin, TDI_pin, TMS_pin), (1, tdi, tms) )
        GPIO.output( (TCK_pin, TDI_pin, TMS_pin), (0, tdi, tms) )
    else:
        tdo = jtag_pins(tdi, tms, pins, gpio_pointer)

    return tdo

def reset_fpga():
    global PRG_pin

    GPIO.output(PRG_pin, 0)
    time.sleep(0.1)
    GPIO.output(PRG_pin, 1)


def decode_ir(ir):
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
    global gpio_pointer
    global pins
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
                    jtag_prog(ffistr, pins, gpio_pointer)
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
                    jtag_prog_rbk(ffistr, pins, gpio_pointer, ffiret)
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
    global gpio_pointer
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
        "-r", "--reset-prog", help="Pull the PROG pin before initiating any commands", default=False, action="store_true"
    )
    args = parser.parse_args()
    if args.debug:
       logging.basicConfig(stream=sys.stdout, level=logging.DEBUG)

    ifile = args.file
    compat = args.compat

    if TCK_pin != args.tck:
        compat = True
        TCK_pin = args.tck
    if TDI_pin != args.tdi:
        compat = True
        TDI_pin = args.tdi
    if TDO_pin != args.tdo:
        compat = True
        TDO_pin = args.tdo
    if TMS_pin != args.tms:
        compat = True
        TMS_pin = args.tms
    if PRG_pin != args.prg:
        PRG_pin = args.prg
        # prog not in FFI, so no need for compat if it changes

    if compat == True and args.compat == False:
        print("Compatibility mode triggered because one of tck/tdi/tdo/tms pins do not match the CFFI bindings")
        print("To fix this, edit gpio-ffi.c and change the #define's to match your bindings, and update the ")
        print("global defaults to the pins in this file, specified just after the imports.")

    rev = GPIO.RPI_INFO
    if rev['P1_REVISION'] == 1:
        gpio_pointer = pi_mmio_init(0x20000000)
    elif rev['P1_REVISION'] == 3 or rev['P1_REVISION'] == 2:
        gpio_pointer = pi_mmio_init(0x3F000000)
    elif rev['P1_REVISION'] == 4:
        gpio_pointer = pi_mmio_init(0xFE000000)
    else:
        print("Unknown Raspberry Pi rev, can't set GPIO base")
        compat = True

    GPIO.setmode(GPIO.BCM)

    GPIO.setup((TCK_pin, TMS_pin, TDI_pin), GPIO.OUT)
    GPIO.setup(TDO_pin, GPIO.IN)
    if args.reset_prog:
        GPIO.setup(PRG_pin, GPIO.OUT)

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

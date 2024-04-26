import re
from jtag_tool import JtagLeg

class Xlat():
    def __init__(self):
        self.checkval = []

    def write_cmd(self, arg, c_backend=True):
        oarg = arg
        arg = f"#1, {arg}"

        cmd = arg.split(', ')
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

        # parse the DR going from host to chip, add it to the JTAG legs
        dr_regex = re.compile("DR data:\(40'h([0-9a-f]*)\)") # very narrow rule because we want to catch if the value isn't 40'h
        dr_hex = dr_regex.match(cmd[3])[1]

        if c_backend:
            print(f"   # {oarg}")
            print(f"   (0x{ir_val:02x}, 0x{int(dr_hex, 16):010x}),")
        else:
            print(f"   # {oarg}")
            print(f"   [JtagLeg.IR, '{'%0*d' % (8, int(bin(ir_val)[2:]))}', ' '], ", end='')
            print(f"[JtagLeg.DR, '{'%0*d' % (40, int(bin(int(dr_hex, 16))[2:]))}', ' '], ")

        if len(cmd) == 5:
            # parse the DR coming from chip to host, add it to the dr_vals tracker
            dre_regex = re.compile("expected DR data:\(40'h([0-9a-f]*)\)") # very narrow rule because we want to catch if the value isn't 40'h
            dre_hex = dre_regex.match(cmd[4])[1]
            self.checkval += [int(dre_hex, 16)]
        else:
            self.checkval += [None]

    def write_comment(self, arg):
        print(f"   # {arg}")

    def print_checkvals(self):
        print('checkvals = [')
        for val in self.checkval:
            print(f'    {val}, ')
        print(']')
        checkvals = []

    def gen1(self):
        print('jtag_legs = [')
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
        self.write_cmd("RW JTAG-REG, IR={6'h07,2'b10}(addr:('h07)), DR data:(40'h0000000000)")
        self.write_cmd("RW JTAG-REG, IR={6'h07,2'b10}(addr:('h07)), DR data:(40'h0000000000)")
        self.write_comment("clear write data buffer")
        self.write_cmd("RW JTAG-REG, IR={6'h06,2'b10}(addr:('h06)), DR data:(40'h0000603480)")
        print(']')
        self.print_checkvals()

    def gen2(self):
        print('jtag_legs = [')
        self.write_comment("User inputs WRITE # of loop=0x0")
        self.write_cmd("RW JTAG-REG, IR={6'h10,2'b10}(addr:('h10)), DR data:(40'h0000000000)")
        self.write_comment("User inputs address and data pattern(ADR_FIX, DATA_FIX, LOOP_0)")
        self.write_cmd("RW JTAG-REG, IR={6'h03,2'b10}(addr:('h03)), DR data:(40'h0000000000)")
        self.write_comment("User sets bist_write_status_ip0_en = 0x1")
        self.write_cmd("RW JTAG-REG, IR={6'h0c,2'b10}(addr:('h0c)), DR data:(40'h0000000040)")
        self.write_comment("User issues bist WRITE command")
        self.write_cmd("RW JTAG-REG, IR={6'h06,2'b10}(addr:('h06)), DR data:(40'h0000605480)")
        self.write_comment("Read out bit[6]=bist_write_status_ip0 (1: fail, 0: pass), bit[0] = bist_busy (1: busy, 0: idle)")
        print(']')
        self.print_checkvals()

    def gen3(self):
        self.write_cmd("RW JTAG-REG, IR={6'h0b,2'b10}(addr:('h0b)), DR data:(40'h000000000a)") # , expected DR data:(40'h000000000e) # LSB (busy) bit changes

    def gen4(self):
        self.write_cmd("RW JTAG-REG, IR={6'h0b,2'b10}(addr:('h0b)), DR data:(40'h000000000a)")

    def v1(self):
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
        self.write_cmd("RW JTAG-REG, IR={6'h07,2'b10}(addr:('h07)), DR data:(40'h0000000000)")

    def v2(self):
        self.write_comment(f"User inputs READ # of loop=0x0")
        self.write_cmd(f"RW JTAG-REG, IR={{6'h10,2'b10}}(addr:('h10)), DR data:(40'h0000000000)")
        self.write_comment(f"User inputs address and data pattern, (data_option needs to be 0x6 = DATA_LATCH to output RRAM DOUT to TDO)")
        self.write_cmd(f"RW JTAG-REG, IR={{6'h03,2'b10}}(addr:('h03)), DR data:(40'h0000000060)")
        self.write_comment(f"User sets bist_read_status_ip0_en = 0x1")
        self.write_cmd(f"RW JTAG-REG, IR={{6'h0c,2'b10}}(addr:('h0c)), DR data:(40'h0000000400)")
        self.write_comment(f"User issues bist READ command")
        self.write_cmd(f"RW JTAG-REG, IR={{6'h06,2'b10}}(addr:('h06)), DR data:(40'h000060c880)")
        self.write_comment(f"Read out bit[10]=bist_read_status_ip0 (1: fail, 0: pass), bit[0] = bist_busy (1: busy, 0: idle)")
        # self.write_wait_tdo()
        self.write_cmd(f"RW JTAG-REG, IR={{6'h0b,2'b10}}(addr:('h0b)), DR data:(40'h000000000a)") #, , expected DR data:(40'h000000000a)") # LSB (busy) bit changes

    def v3(self):
        self.write_cmd(f"RW JTAG-REG, IR={{6'h14,2'b10}}(addr:('h14)), DR data:(40'h{0:010x})")
        self.write_cmd(f"RW JTAG-REG, IR={{6'h15,2'b10}}(addr:('h15)), DR data:(40'h0000000000), expected DR data:(40'h{0:010x})")

x = Xlat()
x.v3()

import RPi.GPIO as GPIO
import smbus
import math
import time

# addr 0x10
CFG_ALDO2_EN  = 0x80
CFG_ALDO1_EN  = 0x40
CFG_DCDC5_EN  = 0x20
CFG_DCDC4_EN  = 0x10
CFG_DCDC3_EN  = 0x08
CFG_DCDC2_EN  = 0x04
CFG_DCDC1_EN  = 0x02
CFG_DC5LDO_EN = 0x01

# addr 0x12
CFG_DC1SW_EN  = 0x80
CFG_DLDO4_EN  = 0x40
CFG_DLDO3_EN  = 0x20
CFG_DLDO2_EN  = 0x10
CFG_DLDO1_EN  = 0x08
CFG_ELDO3_EN  = 0x04
CFG_ELDO2_EN  = 0x02
CFG_ELDO1_EN  = 0x01

# addr 0x13
CFG_ALDO3_EN   = 0x80

NTC_DRV_UA   = 40
NTC_BETA = 3750
UT_THRESHOLD_MV = (24704 * NTC_DRV_UA / 800)
OT_THRESHOLD_MV = (2668  * NTC_DRV_UA / 800)

# USER CONFIG

# DLDO input: SW1, 3.4V
# DLDO2: VQFC
DC1_VOLTAGE_MV = 3300
CHG_CUR_MA = 450

def AXP22X_CFG_LDO0733(v):
  return (int(((v) - 700) / 100) & 0x1f)

def AXP22X_CFG_DCDC1634(v):
  return (int(((v) - 1600) / 100) & 0x3f)

def AXP22X_CFG_DCDC0V6_1V86(v):
  return (int(((v) - 600) / 20) & 0x3f)

def AXP22X_CFG_DCDC1V_2V55(v):
  return (int(((v) - 1000) / 50) & 0x1f)


axp223_cfg = {
  0x10: CFG_DCDC1_EN | CFG_DCDC2_EN | CFG_DCDC3_EN | CFG_DCDC4_EN | CFG_DCDC5_EN | CFG_DC5LDO_EN | CFG_ALDO2_EN,
  0x12: CFG_DLDO1_EN | CFG_DLDO2_EN | CFG_ELDO2_EN, #DC1SW, DLDO, ELDO:
  0x13: 0x00, #  0x13, ALDO3: off

  0x15: AXP22X_CFG_LDO0733(3000), # 0x15 V DLDO1 VUSB3
  0x16: AXP22X_CFG_LDO0733(3300), # 0x16 V DLDO2 VQFC
  0x17: AXP22X_CFG_LDO0733(3200), # 0x17 V DLDO3 VSD
  0x18: AXP22X_CFG_LDO0733(3300), # 0x18 V DLDO4 VSCC
  0x19: AXP22X_CFG_LDO0733(700) , # 0x19 V ELDO1, test
  0x1a: AXP22X_CFG_LDO0733(900) , # 0x1a V ELDO2, VCC usb 0V9
  0x1b: AXP22X_CFG_LDO0733(1100), # 0x1b V ELDO3, test
  0x1c: AXP22X_CFG_LDO0733(900) , # 0x1c V DC5LDO, 0.7-1.4

  0x21: AXP22X_CFG_DCDC1634(DC1_VOLTAGE_MV), # DCDC1 3.4
  0x22: AXP22X_CFG_DCDC0V6_1V86(700) , # DCDC2 .85 core supply, do NOT go below 0.7V
  0x23: AXP22X_CFG_DCDC0V6_1V86(1860), # DCDC3 1.86
  0x24: AXP22X_CFG_DCDC0V6_1V86(800) , # DCDC4, unused, test
  0x25: AXP22X_CFG_DCDC1V_2V55(1100) , # DCDC5

  0x27: 0                            , # DCDC2/3 dynamoc voltage scaling
  0x28: AXP22X_CFG_LDO0733(2500)     , # V ALDO1, V25A
  0x29: AXP22X_CFG_LDO0733(2600)     , # V ALDO2, +2V5
  0x2a: AXP22X_CFG_LDO0733(3100)     , # V ALDO3, VCCRR

  0x30: 0x63, # 0x30, TODO: VBUS, IPS: [1:0]2 nolim, 2:100mA, 1:500mA, 0:900mA
  0x31: 0x06, # 0x31, TODO: wakeup, shutdown at 3.2V
  0x32: 0x6B, # 0x32, bat det on, chgled 0.5Hz, auto chgled, pwrok 64ms
  0x33: 0xC0 | int(CHG_CUR_MA / 150 - 2), # 0x33, charge control1, 8x:4.1V, Cx:4.2V, 10%, 450mA (mA / 150 - 2)
  0x34: 0x04, # 0x34, charge control2, 40min, off, ext path on, 6hr
  0x35: 0x00 | int(600 / 150 - 2), # 0x35, charge control3. 450mA TODO: what is Charge loop current limit ?
  0x36: 0x08, # 0x36, PEK, pon 128ms, long 1s, button power off: true, AC power on: false, poff time 4s
  0x37: 0x08, # 0x37, DCDC freq, default 3MHz
  0x38: UT_THRESHOLD_MV / 8, # 0x38, VLTF TODO, unit 8mV
  0x39: OT_THRESHOLD_MV / 8, # 0x39, VHTF TODO
  0x3c: UT_THRESHOLD_MV / 8, # 0x3c, batt discharge UT
  0x3d: OT_THRESHOLD_MV / 8, # 0x3d, batt discharge OT
  0x80: 0x00, # 1F:force PWM mode, 00: PFM/PWM auto
  0x82: 0xE1, # ADC enable, all en
  0x84: 0xC3 | (int((NTC_DRV_UA - 20) / 20) << 4), # ADC sample rate, 800sps, 80uA, batt tmp mon, I src out when chg
  0x85: 0xC0, # TS ADC sample rate, 100sps
  0x8F: 0x05, # OT shutdown
  0xB8: 0xE0, # fuel gauge :on, coulomb counter: on, cal on
  0xE0: int(600/1.456) >> 8, # batt cap, x 1.456 mAh, MSB
  0xE1: int(600/1.456) & 0xff, # batt cap, x 1.456 mAh, LSB
  0xE6: 0x05, # fuel gauge update time: 5s
  0xE9: 0xC0, # fuel gauge calibration interval: 15s
}

bus = smbus.SMBus(1)
AXP223_ADDR = 0x34

def i2cdump(do_print=False):
    buf = []
    for i in range(0, 256):
        d = bus.read_byte_data(AXP223_ADDR, i)
        if do_print:
            if i % 16 == 0:
                print(f'\n{i:02x}: ', end='')
            print(f'{d:02x} ', end='')
        buf += [d]
    if do_print:
        print('')
    return buf

def data13(buf, i):
    return buf[i] << 5 | ( buf[i + 1] & 0x1f)

def data12(buf, i):
    return buf[i] << 4 | ( buf[i + 1] & 0x0f)

def data16(buf, i):
    return (buf[i] << 8) | ( buf[i + 1])

def parsedata(buf):
    t32 = data12(buf, 0x56)
    t32 -= 2510; # What is the offset?, 267.7C in doc is off
    print(f"int temp {t32 // 10}.{abs(t32) % 10}C")

    t32 = data12(buf, 0x58)
    # t = 1. / (np.log(r / r25) / beta + 1/t25k)
    tfloat = 1. / (math.log((((t32 * 8 / 1e3 / (NTC_DRV_UA*1e-6)) / 1e3) / NTC_BETA + 1/(273.15+25)), 10))
    print(f"batt TS: {t32 * 8 / 10} mV, raw 0x{int(t32):x}, {tfloat - 273.15:0.2f}C")

    t32 = data12(buf, 0x78)
    print(f"batt voltage: {t32 * 11 / 10:0.2f}mV")

    # Current reading has unknown scale
    # guessing 1 LSB = 0.73 mA
    t32 = data12(buf, 0x7A)
    print(f"batt chg  mA: {t32 * 73/100:0.2f}")

    t32 = data12(buf, 0x7C)
    print(f"batt dchg mA: {t32 * 73/100}")

    if (0x80 & buf[0xB9]) != 0:
        t16 = buf[0xB9];
        print(f"batt {t16 & 0x7f}%")
        print(f"batt cap {data16(buf, 0xE0) * 1456 / 1000:0.2f} mAh")

def initAXP223(do_print=False):
    for addr, data in axp223_cfg.items():
        data = int(data)
        if do_print:
            print(f'*({addr:02x}) = {data:02x}')
        bus.write_byte_data(AXP223_ADDR, addr, data)

def main():
    TCK_pin = 4
    PRG_pin = 24
    GPIO.setmode(GPIO.BCM)
    GPIO.setup((TCK_pin, PRG_pin), GPIO.IN)
    buf = i2cdump(do_print=False)
    print("AXP223 stats:")
    parsedata(buf)

    if buf[0x21] != AXP22X_CFG_DCDC1634(DC1_VOLTAGE_MV):
        initAXP223(do_print=False)
        time.sleep(1.0)
        print("Post-config dump:")
        buf = i2cdump(do_print=True)
        parsedata(buf)
    else:
        print("Device is already configured")

if __name__ == "__main__":
    main()
    exit(0)

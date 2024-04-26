/* run 'python3 build.py' to generate the ffi bindings before running jtag-gpio.py */
/*

This allows the setting of the three general purpose clocks.

The clocks are named GPCLK0, GPCLK1, and GPCLK2.

The clocks are accessible from the following gpios.

gpio4  GPCLK0 ALT0
gpio5  GPCLK1 ALT0 B+ and compute module only (reserved for system use)
gpio6  GPCLK2 ALT0 B+ and compute module only
gpio20 GPCLK0 ALT5 B+ and compute module only
gpio21 GPCLK1 ALT5 Not available on Rev.2 B (reserved for system use)

gpio32 GPCLK0 ALT0 Compute module only
gpio34 GPCLK0 ALT0 Compute module only
gpio42 GPCLK1 ALT0 Compute module only (reserved for system use)
gpio43 GPCLK2 ALT0 Compute module only
gpio44 GPCLK1 ALT0 Compute module only (reserved for system use)

Clock sources

0     0 Hz                    Ground
1     19.2 MHz / 54 Mhz (Pi4) oscillator
2     0 Hz                    testdebug0
3     0 Hz                    testdebug1
4     0 Hz                    PLLA
5     1000 MHz                PLLC (changes with overclock settings)
6     500 MHz  / 750MHz (Pi4) PLLD
7     216 MHz                 HDMI auxiliary
8-15  0 Hz                    Ground

The integer divider may be 2-4095.
The fractional divider may be 0-4095.

There is no 25MHz cap for using non-zero MASH
(multi-stage noise shaping) values.
*/

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <unistd.h>
#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <arpa/inet.h>

#include "gpio-ffi.h"
#include "time.h"

// #define GPIO_BASE (base + 0x200000)
#define GPIO_LENGTH 4096

volatile uint32_t *pi_mmio_gpio = NULL;

static volatile uint32_t piModel = 1;

static volatile uint32_t piPeriphBase = 0x20000000;
static volatile uint32_t piBusAddr = 0x40000000;

#define SYST_BASE  (piPeriphBase + 0x003000)
#define DMA_BASE   (piPeriphBase + 0x007000)
#define CLK_BASE   (piPeriphBase + 0x101000)
#define GPIO_BASE  (piPeriphBase + 0x200000)
#define UART0_BASE (piPeriphBase + 0x201000)
#define PCM_BASE   (piPeriphBase + 0x203000)
#define SPI0_BASE  (piPeriphBase + 0x204000)
#define I2C0_BASE  (piPeriphBase + 0x205000)
#define PWM_BASE   (piPeriphBase + 0x20C000)
#define BSCS_BASE  (piPeriphBase + 0x214000)
#define UART1_BASE (piPeriphBase + 0x215000)
#define I2C1_BASE  (piPeriphBase + 0x804000)
#define I2C2_BASE  (piPeriphBase + 0x805000)
#define DMA15_BASE (piPeriphBase + 0xE05000)

#define DMA_LEN   0x1000 /* allow access to all channels */
#define CLK_LEN   0xA8
#define GPIO_LEN  0xB4
#define SYST_LEN  0x1C
#define PCM_LEN   0x24
#define PWM_LEN   0x28
#define I2C_LEN   0x1C

#define GPSET0 7
#define GPSET1 8

#define GPCLR0 10
#define GPCLR1 11

#define GPLEV0 13
#define GPLEV1 14

#define GPPUD     37
#define GPPUDCLK0 38
#define GPPUDCLK1 39

#define SYST_CS  0
#define SYST_CLO 1
#define SYST_CHI 2

#define CLK_PASSWD  (0x5A<<24)

#define CLK_CTL_MASH(x)((x)<<9)
#define CLK_CTL_BUSY    (1 <<7)
#define CLK_CTL_KILL    (1 <<5)
#define CLK_CTL_ENAB    (1 <<4)
#define CLK_CTL_SRC(x) ((x)<<0)

#define CLK_SRCS 4

static double cfreq[CLK_SRCS]={500e6, 19.2e6, 216e6, 1000e6};
// static char *clocks[CLK_SRCS]={"PLLD", " OSC", "HDMI", "PLLC"};
static double clk_min_freq = 4687.5;
static double clk_max_freq = 250e6;

#define CLK_CTL_SRC_OSC  1  /* 19.2 MHz or 54 MHz (Pi4)*/
#define CLK_CTL_SRC_PLLC 5  /* 1000 MHz */
#define CLK_CTL_SRC_PLLD 6  /*  500 MHz  or 750 MHz (Pi4)*/
#define CLK_CTL_SRC_HDMI 7  /*  216 MHz */

#define CLK_DIV_DIVI(x) ((x)<<12)
#define CLK_DIV_DIVF(x) ((x)<< 0)

#define CLK_GP0_CTL 28
#define CLK_GP0_DIV 29
#define CLK_GP1_CTL 30
#define CLK_GP1_DIV 31
#define CLK_GP2_CTL 32
#define CLK_GP2_DIV 33

#define CLK_PCM_CTL 38
#define CLK_PCM_DIV 39

#define CLK_PWM_CTL 40
#define CLK_PWM_DIV 41


static volatile uint32_t  *gpioReg = MAP_FAILED;
static volatile uint32_t  *systReg = MAP_FAILED;
static volatile uint32_t  *clkReg  = MAP_FAILED;

#define PI_BANK (gpio>>5)
#define PI_BIT  (1<<(gpio&0x1F))

/* gpio modes. */

#define PI_INPUT  0
#define PI_OUTPUT 1
#define PI_ALT0   4
#define PI_ALT1   5
#define PI_ALT2   6
#define PI_ALT3   7
#define PI_ALT4   3
#define PI_ALT5   2

void gpioSetMode(unsigned gpio, unsigned mode)
{
   int reg, shift;

   reg   =  gpio/10;
   shift = (gpio%10) * 3;

   gpioReg[reg] = (gpioReg[reg] & ~(7<<shift)) | (mode<<shift);
}

int gpioGetMode(unsigned gpio)
{
   int reg, shift;

   reg   =  gpio/10;
   shift = (gpio%10) * 3;

   return (*(gpioReg + reg) >> shift) & 7;
}

unsigned gpioHardwareRevision(void)
{
   static unsigned rev = 0;

   FILE * filp;
   char buf[512];
   char term;

   if (rev) return rev;

   filp = fopen ("/proc/cpuinfo", "r");

   if (filp != NULL)
   {
      while (fgets(buf, sizeof(buf), filp) != NULL)
      {
         if (!strncasecmp("revision\t:", buf, 10))
         {
            if (sscanf(buf+10, "%x%c", &rev, &term) == 2)
            {
               if (term != '\n') rev = 0;
            }
         }
      }
      fclose(filp);
   }

   if (rev == 0)
   {
      /* (some) arm64 operating systems get revision number here  */
      filp = fopen ("/proc/device-tree/system/linux,revision", "r");

      if (filp != NULL)
      {
         uint32_t tmp;
         if (fread(&tmp,1 , 4, filp) == 4)
         {
            /*
               for some reason the value returned by reading
               this /proc entry seems to be big endian,
               convert it.
            */
            rev = ntohl(tmp);
         }
         fclose(filp);
      }
   }

   rev &= 0xFFFFFF; /* mask out warranty bit */

   /* Decode revision code */

   if ((rev & 0x800000) == 0) /* old rev code */
   {
      if ((rev > 0) && (rev < 0x0016)) /* all BCM2835 */
      {
         piPeriphBase = 0x20000000;
         piBusAddr  = 0x40000000;
      }
      else
      {
         rev = 0;
      }
   }
   else /* new rev code */
   {
      switch ((rev >> 12) & 0xF)  /* just interested in BCM model */
      {

         case 0x0:   /* BCM2835 */
            piPeriphBase = 0x20000000;
            piBusAddr  = 0x40000000;
            break;

         case 0x1:   /* BCM2836 */
         case 0x2:   /* BCM2837 */
            piPeriphBase = 0x3F000000;
            piBusAddr  = 0xC0000000;
            break;

         case 0x3:   /* BCM2711 */
            piPeriphBase = 0xFE000000;
            piBusAddr  = 0xC0000000;
            cfreq[0] = 750e6;
            cfreq[1] = 54e6;
            clk_min_freq = 13184.0;
            clk_max_freq = 375e6;
            break;

         default:
            rev = 0;
            break;
      }
   }

   return rev;
}

/* Returns the number of microseconds after system boot. Wraps around
   after 1 hour 11 minutes 35 seconds.
*/

uint32_t gpioTick(void) { return systReg[SYST_CLO]; }


static int initClock(int clock, int source, int divI, int divF, int MASH)
{
   int ctl[] = {CLK_GP0_CTL, CLK_GP2_CTL};
   int div[] = {CLK_GP0_DIV, CLK_GP2_DIV};
   int src[CLK_SRCS] =
      {CLK_CTL_SRC_PLLD,
       CLK_CTL_SRC_OSC,
       CLK_CTL_SRC_HDMI,
       CLK_CTL_SRC_PLLC};

   int clkCtl, clkDiv, clkSrc;

   if ((clock  < 0) || (clock  > 1))    return -1;
   if ((source < 0) || (source > 3 ))   return -2;
   if ((divI   < 2) || (divI   > 4095)) return -3;
   if ((divF   < 0) || (divF   > 4095)) return -4;
   if ((MASH   < 0) || (MASH   > 3))    return -5;

   clkCtl = ctl[clock];
   clkDiv = div[clock];
   clkSrc = src[source];

   clkReg[clkCtl] = CLK_PASSWD | CLK_CTL_KILL;

   /* wait for clock to stop */

   while (clkReg[clkCtl] & CLK_CTL_BUSY)
   {
      usleep(10);
   }

   clkReg[clkDiv] =
      (CLK_PASSWD | CLK_DIV_DIVI(divI) | CLK_DIV_DIVF(divF));

   usleep(10);

   clkReg[clkCtl] =
      (CLK_PASSWD | CLK_CTL_MASH(MASH) | CLK_CTL_SRC(clkSrc));

   usleep(10);

   clkReg[clkCtl] |= (CLK_PASSWD | CLK_CTL_ENAB);
   return 0;
}

int termClock(int clock)
{
   int ctl[] = {CLK_GP0_CTL, CLK_GP2_CTL};

   int clkCtl;

   if ((clock  < 0) || (clock  > 1))    return -1;

   clkCtl = ctl[clock];

   clkReg[clkCtl] = CLK_PASSWD | CLK_CTL_KILL;

   /* wait for clock to stop */

   while (clkReg[clkCtl] & CLK_CTL_BUSY)
   {
      usleep(10);
   }
   return 0;
}

void startClock() //10MHz output
{
  //PLLD(750MHz), GPCLK0,75DIV, 0FR, INT
  initClock(0, 0, 37, 5, 0);
  gpioSetMode(4, 4);
}

void stopClock()
{
    termClock(0);
    gpioSetMode(4, 1);
}



/* Map in registers. */

static uint32_t * initMapMem(int fd, uint32_t addr, uint32_t len)
{
    return (uint32_t *) mmap(0, len,
       PROT_READ|PROT_WRITE|PROT_EXEC,
       MAP_SHARED|MAP_LOCKED,
       fd, addr);
}

int gpioInitialise(void)
{
   int fd;

   gpioHardwareRevision(); /* sets piModel, needed for peripherals address */

   fd = open("/dev/mem", O_RDWR | O_SYNC) ;

   if (fd < 0)
   {
      fprintf(stderr,
         "This program needs root privileges.  Try using sudo\n");
      return -1;
   }

   gpioReg  = initMapMem(fd, GPIO_BASE, GPIO_LEN);
   systReg  = initMapMem(fd, SYST_BASE, SYST_LEN);
   clkReg   = initMapMem(fd, CLK_BASE,  CLK_LEN);

   close(fd);

   if ((gpioReg == MAP_FAILED) ||
       (systReg == MAP_FAILED) ||
       (clkReg == MAP_FAILED))
   {
      fprintf(stderr,
         "Bad, mmap failed\n");
      return -1;
   }
   return 0;
}

//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////

volatile uint32_t *pi_mmio_init(uint32_t base) {
  if (pi_mmio_gpio == NULL) {
    int fd;

    // On older kernels user readable /dev/gpiomem might not exists.
    // Falls back to root-only /dev/mem.
    if( access( "/dev/gpiomem", F_OK ) != -1 ) {
      fd = open("/dev/gpiomem", O_RDWR | O_SYNC);
    } else {
      fd = open("/dev/mem", O_RDWR | O_SYNC);
    }
    if (fd == -1) {
      // Error opening /dev/gpiomem.
      return 0;
    }
    // Map GPIO memory to location in process space.
    pi_mmio_gpio = (uint32_t *)mmap(NULL, GPIO_LENGTH, PROT_READ | PROT_WRITE, MAP_SHARED, fd, GPIO_BASE);
    close(fd);
    if (pi_mmio_gpio == MAP_FAILED) {
      // Don't save the result if the memory mapping failed.
      pi_mmio_gpio = NULL;
      return 0;
    }
  }
  return pi_mmio_gpio;
}

#define GPIO_SET *(gpio+7)  // sets   bits which are 1 ignores bits which are 0
#define GPIO_CLR *(gpio+10) // clears bits which are 1 ignores bits which are 0
#define GPIO_LVL *(gpio+13)

/// Clock tck with TDI, TMS values, and return the value of TDO.
char jtag_pins(int tdi, int tms, pindefs pins, volatile uint32_t *gpio) {

  char out_r0 = 0;
  char out_r1 = 0;

  GPIO_CLR = 1 << pins.tck;

  if(tdi){
    if(pins.tdi_r0 > 0) GPIO_SET = 1 << pins.tdi_r0;
    if(pins.tdi_r1 > 0) GPIO_SET = 1 << pins.tdi_r1;
  }else{
    if(pins.tdi_r0 > 0) GPIO_CLR = 1 << pins.tdi_r0;
    if(pins.tdi_r1 > 0) GPIO_CLR = 1 << pins.tdi_r1;
  }

  if(tms){
    if(pins.tms_r0 > 0) GPIO_SET = 1 << pins.tms_r0;
    if(pins.tms_r1 > 0) GPIO_SET = 1 << pins.tms_r1;
  }else{
    if(pins.tms_r0 > 0) GPIO_CLR = 1 << pins.tms_r0;
    if(pins.tms_r1 > 0) GPIO_CLR = 1 << pins.tms_r1;
  }
  //adding delay (hard to do hard timing on OS)
  volatile int t = 5;
  while(t>0)
    t--;

  GPIO_SET = 1 << pins.tck;

  out_r0 = (GPIO_LVL & (1 << pins.tdo_r0)) ? 1 : 0;
  out_r1 = (GPIO_LVL & (1 << pins.tdo_r1)) ? 1 : 0;

  return out_r0 | (out_r1<<4);
  //return (GPIO_LVL & (1 << pins.tdo)) ? 1 : 0;
}

#define PULSE 200
void jtag_cycle_clk(pindefs pins, volatile uint32_t *gpio, int cycles) {

  // struct timespec start, current = {0};
  volatile uint16_t tiks = 5;

  for(int i = 0; i < cycles; i++){

    GPIO_CLR = 1 << pins.tck;

    while(tiks > 0){
      tiks--;
    }

    GPIO_SET = 1 << pins.tck;

    while(tiks < 20){
      tiks++;
    }

    // clock_gettime(CLOCK_REALTIME, &start);
    // GPIO_CLR = 1 << pins.tck;
    // clock_gettime(CLOCK_REALTIME, &current);

    // while(current.tv_nsec - start.tv_nsec < PULSE){
    //   clock_gettime(CLOCK_REALTIME, &current);
    // }

    // clock_gettime(CLOCK_REALTIME, &start);
    // GPIO_SET = 1 << pins.tck;
    // clock_gettime(CLOCK_REALTIME, &current);

    // while(current.tv_nsec - start.tv_nsec < PULSE){
    //   clock_gettime(CLOCK_REALTIME, &current);
    // }
  }
}

void set_dbg(pindefs pins, volatile uint32_t *gpio) {

  GPIO_SET = 1 << pins.dbg;

}

void clear_dbg(pindefs pins, volatile uint32_t *gpio) {

  GPIO_CLR = 1 << pins.dbg;
}

int jtag_prog(char *bitstream, pindefs pins, volatile uint32_t *gpio) {

  GPIO_CLR = 1 << pins.tms_r0; // TMS is known to be zero for this operation
  int i = 0;
  while(bitstream[i] != '\0') {
    GPIO_CLR = 1 << pins.tck;

    if(bitstream[i] == '1')
      GPIO_SET = 1 << pins.tdi_r0;
    else
      GPIO_CLR = 1 << pins.tdi_r0;

    GPIO_SET = 1 << pins.tck;

    i++;
  }

  return 0; // we ignore TDO for speed
}

void jtag_prog_rbk(char *bitstream, pindefs pins, volatile uint32_t *gpio, char *readback) {

  GPIO_CLR = 1 << pins.tms_r0; // TMS is known to be zero for this operation
  int i = 0;
  GPIO_CLR = 1 << pins.tck;
  while(bitstream[i] != '\0') {
    if(bitstream[i] == '1')
      GPIO_SET = 1 << pins.tdi_r0;
    else
      GPIO_CLR = 1 << pins.tdi_r0;

    GPIO_SET = 1 << pins.tck; // clock needs stretching on the rpi4
    GPIO_SET = 1 << pins.tck;
    GPIO_SET = 1 << pins.tck;
    GPIO_SET = 1 << pins.tck;

    GPIO_CLR = 1 << pins.tck; // meet hold time
    GPIO_CLR = 1 << pins.tck;
    GPIO_CLR = 1 << pins.tck;
    GPIO_CLR = 1 << pins.tck;

    if (GPIO_LVL & (1 << pins.tdo_r0)) {
       readback[i] = '1';
    } else {
       readback[i] = '0';
    }

    i++;
  }
}

/// Clock tck with TDI, TMS values, and return the value of TDO.
void jtag_pins_no_tdo(int tdi, int tms, pindefs pins, volatile uint32_t *gpio) {
  GPIO_CLR = 1 << pins.tck;

  if(tdi){
    if(pins.tdi_r0 > 0) GPIO_SET = 1 << pins.tdi_r0;
    if(pins.tdi_r1 > 0) GPIO_SET = 1 << pins.tdi_r1;
  }else{
    if(pins.tdi_r0 > 0) GPIO_CLR = 1 << pins.tdi_r0;
    if(pins.tdi_r1 > 0) GPIO_CLR = 1 << pins.tdi_r1;
  }

  if(tms){
    if(pins.tms_r0 > 0) GPIO_SET = 1 << pins.tms_r0;
    if(pins.tms_r1 > 0) GPIO_SET = 1 << pins.tms_r1;
  }else{
    if(pins.tms_r0 > 0) GPIO_CLR = 1 << pins.tms_r0;
    if(pins.tms_r1 > 0) GPIO_CLR = 1 << pins.tms_r1;
  }
  //adding delay (hard to do hard timing on OS)
  volatile int t = 5;
  while(t>0)
    t--;

  GPIO_SET = 1 << pins.tck;
}

/// 8-bit IR, fixed-width register shift
/// Enters from Idle
/// leaves in DR-Scan state
uint8_t jtag_ir8_to_dr(uint8_t ir, pindefs pins, volatile uint32_t *gpio, uint8_t bank) {
   uint8_t ret = 0;
   // idle -> select IR-scan
   jtag_pins_no_tdo(0, 1, pins, gpio);
   jtag_pins_no_tdo(0, 1, pins, gpio);
   // select IR-scan -> capture
   jtag_pins_no_tdo(0, 0, pins, gpio);
   // capture -> shift-IR
   jtag_pins_no_tdo(0, 0, pins, gpio);
   // shift loop, from LSB to MSB
   for(int i = 0; i < 8; i++) {
      if (i != 7) {
         jtag_pins_no_tdo(ir & 1, 0, pins, gpio);
      } else {
         // last item gets TMS = 1
         jtag_pins_no_tdo(ir & 1, 1, pins, gpio);
      }
      ir >>= 1;
      ret >>= 1;
      if (bank) {
         ret |= (GPIO_LVL & (1 << pins.tdo_r1)) ? 0x80 : 0;
      } else {
         ret |= (GPIO_LVL & (1 << pins.tdo_r0)) ? 0x80 : 0;
      }
   }
   // Exit1 -> Update-IR
   jtag_pins_no_tdo(0, 1, pins, gpio);
   // Update-IR -> DR-Scan
   jtag_pins_no_tdo(0, 1, pins, gpio);

   // drop TMS in case of spurious clock
   if(pins.tms_r0 > 0) GPIO_CLR = 1 << pins.tms_r0;
   if(pins.tms_r1 > 0) GPIO_CLR = 1 << pins.tms_r1;

   return ret;
}

/// 40-bit DR, fixed-width register shift
/// Assumes entry from DR-scan
/// Leaves in Idle state
/// 32 bit LSB is in dr_io[0]
/// 8 bit MSB is in dr_io[1]
void jtag_dr40_to_idle(uint32_t dr_lsb, uint32_t dr_msb, uint32_t *ret_data, pindefs pins, volatile uint32_t *gpio, uint8_t bank) {
   uint32_t dr_io[2] = {dr_lsb, dr_msb};
   // select DR-scan -> capture
   jtag_pins_no_tdo(0, 0, pins, gpio);
   // capture -> shift-DR
   jtag_pins_no_tdo(0, 0, pins, gpio);
   // shift loop
   uint32_t limit = 32;
   uint32_t ret = 0;
   for (uint32_t j = 0; j < 2; j++) {
      uint32_t dr = dr_io[j];
      if (j == 0) {
         limit = 32;
      } else {
         limit = 8;
      }
      for (uint32_t i = 0; i < limit; i++) {
         if (!((i == 7) && (j == 1))) {
            jtag_pins_no_tdo(dr & 1, 0, pins, gpio);
         } else {
            // last item gets TMS = 1
            jtag_pins_no_tdo(dr & 1, 1, pins, gpio);
         }
         dr >>= 1;
         ret >>= 1;
         if (bank) {
            ret |= (GPIO_LVL & (1 << pins.tdo_r1)) ? 0x80000000 : 0;
         } else {
            ret |= (GPIO_LVL & (1 << pins.tdo_r0)) ? 0x80000000 : 0;
         }
      }
      ret_data[j] = ret;
      ret = 0;
   }
   // Exit1 -> Update-DR
   jtag_pins_no_tdo(0, 1, pins, gpio);
   // Update-DR -> Idle
   jtag_pins_no_tdo(0, 0, pins, gpio);
}

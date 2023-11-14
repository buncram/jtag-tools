/* run 'python3 build.py' to generate the ffi bindings before running jtag-gpio.py */
#include "gpio-ffi.h"

#include <fcntl.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <stdint.h>

//#define BCM2708_PERI_BASE 0x3F000000 // tested on Rpi3
#define GPIO_BASE (base + 0x200000)
#define GPIO_LENGTH 4096

typedef struct jtag_pins jtag_pins;
struct jtag_pins {
  uint32_t tck;
  uint32_t tms;
  uint32_t tdi;
  uint32_t tdo;
  uint32_t trst;
};

volatile uint32_t *pi_mmio_gpio = NULL;

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

#define TCK_PIN 4
#define TMS_PIN 17
#define TDI_PIN 27
#define TDO_PIN 22

/// Clock tck with TDI, TMS values, and return the value of TDO.
int jtag_pins(int tdi, int tms, jtag_pins pins, volatile uint32_t *gpio) {

  GPIO_CLR = 1 << pins.tck;

  if(tdi)
    GPIO_SET = 1 << pins.tdi;
  else
    GPIO_CLR = 1 << pins.tdi;

  if(tms)
    GPIO_SET = 1 << pins.tms;
  else
    GPIO_CLR = 1 << pins.tms;

  GPIO_SET = 1 << pins.tck;

  return (GPIO_LVL & (1 << pins.tdo_pin)) ? 1 : 0;
}

int jtag_prog(char *bitstream, jtag_pins pins, volatile uint32_t *gpio) {

  GPIO_CLR = 1 << pins.tms_pins; // TMS is known to be zero for this operation
  int i = 0;
  while(bitstream[i] != '\0') {
    GPIO_CLR = 1 << pins.tck;

    if(bitstream[i] == '1')
      GPIO_SET = 1 << pins.tdi;
    else
      GPIO_CLR = 1 << pins.tdi;

    GPIO_SET = 1 << pins.tck;

    i++;
  }

  return 0; // we ignore TDO for speed
}

void jtag_prog_rbk(char *bitstream, jtag_pins pins, volatile uint32_t *gpio, char *readback) {

  GPIO_CLR = 1 << pins.tms; // TMS is known to be zero for this operation
  int i = 0;
  GPIO_CLR = 1 << pins.tck;
  while(bitstream[i] != '\0') {
    if(bitstream[i] == '1')
      GPIO_SET = 1 << pins.tdi;
    else
      GPIO_CLR = 1 << pins.tdi;

    GPIO_SET = 1 << pins.tck; // clock needs stretching on the rpi4
    GPIO_SET = 1 << pins.tck;
    GPIO_SET = 1 << pins.tck;
    GPIO_SET = 1 << pins.tck;

    GPIO_CLR = 1 << pins.tck; // meet hold time
    GPIO_CLR = 1 << pins.tck;
    GPIO_CLR = 1 << pins.tck;
    GPIO_CLR = 1 << pins.tck;

    if (GPIO_LVL & (1 << pins.tdo_pin)) {
       readback[i] = '1';
    } else {
       readback[i] = '0';
    }

    i++;
  }
}

#include <sys/types.h>
#include <stdint.h>

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

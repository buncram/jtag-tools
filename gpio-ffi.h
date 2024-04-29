#include <sys/types.h>
#include <stdint.h>

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
uint8_t jtag_ir8_to_dr(uint8_t ir, pindefs pins, volatile uint32_t *gpio, uint8_t bank);
void jtag_dr40_to_idle(uint32_t dr_lsb, uint32_t dr_msb, uint32_t *ret, pindefs pins, volatile uint32_t *gpio, uint8_t bank);
void jtag_dr40_to_idle_unrolled(uint32_t dr_lsb, uint32_t dr_msb, uint32_t *ret, pindefs pins, volatile uint32_t *gpio, uint8_t bank);
void jtag_dr40_to_idle_noret(uint32_t dr_lsb, uint32_t dr_msb, uint32_t *ret, pindefs pins, volatile uint32_t *gpio, uint8_t bank);
void jtag_ir_dr_to_idle(uint8_t ir, uint32_t dr_lsb, uint32_t dr_msb, uint32_t *ret_data, pindefs pins, volatile uint32_t *gpio, uint8_t bank);
void jtag_ir_dr_to_idle_noret(uint8_t ir, uint32_t dr_lsb, uint32_t dr_msb, uint32_t *ret_data, pindefs pins, volatile uint32_t *gpio, uint8_t bank);

int gpioInitialise(void);
void startClock(void);
void stopClock(void);

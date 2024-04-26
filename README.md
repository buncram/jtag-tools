# jtag tools

A collection of scripts for chip bringup, debug, and manufacturing

- `rram_burn.py` can burn code into the chip
- `jtag_tool.py` has drivers for diagnostics, CP test, forming

These scripts talk to the chip through testmode-only JTAG ports. These
pads are not available in a production package, and are normally only
accessible prior to packaging (while the chip is part of a whole wafer,
at probe and sort time). They are not bonded out in a package, thus
access to the pins post-packaging requires a lot of needle probes
and patience, as well as bypassing the one-way door disable fuse with a
FIB or glitch of some sort.

A factory-new chip needs forming and fusing before it can be used. This
is done by running `dchk.sh` to check that the JTAG port is accessible,
and then `run.sh` to do the entire forming process. Note that this requires
3.63V on RRAM and a collection of .tex scripts which are not part of this
repo, along with other analog pre-flight checks that are outside the scope of
this tool.

Once the chip is formed and fused, the RV32 should be enabled to
boot to 0x6000_0000.

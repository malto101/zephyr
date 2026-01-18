# FPGA Test Application for Shrike-lite

This sample demonstrates how to load a bitstream into the Renesas SLG47910V FPGA on the Shrike-lite board.

## Prerequisites

1. A bitstream file (e.g., `blink.bin`) compiled for the SLG47910V FPGA
2. The Shrike-lite board with Zephyr support

## Converting Bitstream to Header File

### Method 1: Using the Conversion Script (Recommended)

1. Place your `blink.bin` file in the sample directory
2. Run the conversion script:
   ```bash
   cd samples/drivers/fpga/fpga_test_shrike
   ./convert_bitstream.sh blink.bin src/blink.h
   ```

   This will automatically:
   - Convert the binary to a C header file
   - Rename variables to `blink_bitstream` and `blink_bitstream_len`
   - Set the correct type (`uint32_t`) for FPGA driver compatibility

### Method 2: Manual Conversion

1. Convert the binary file to a C header file:
   ```bash
   xxd -i blink.bin > src/blink.h
   ```

2. Edit `src/blink.h` to rename the variables:
   - Change the array name from `blink_bin` to `blink_bitstream`
   - Change the length variable from `blink_bin_len` to `blink_bitstream_len`
   - Change the array type from `unsigned char` to `uint32_t`
   - Add `#include <stdint.h>` at the top

   Example modification:
   ```c
   #include <stdint.h>
   
   const uint32_t blink_bitstream[] = { ... };
   const unsigned int blink_bitstream_len = ...;
   ```

## Building

```bash
west build -b shrike_lite samples/drivers/fpga/fpga_test_shrike
```

## Flashing

```bash
west flash
```

## Expected Output

After flashing and running, you should see output like:

```
FPGA Test Application for Shrike-lite
=====================================

FPGA device: FPGA_0
FPGA status: INACTIVE

Powering on FPGA...
FPGA powered on

Loading bitstream (blink.bin)...
Bitstream address: 0x...
Bitstream size: XXXX bytes
Bitstream loaded successfully!
FPGA info: <CRC info>
FPGA status after load: ACTIVE

FPGA configuration complete!
If your bitstream implements LED blinking, you should see it now.
```

## Alternative: Using Shell Commands

You can also use the FPGA shell commands to load bitstreams dynamically:

1. Build with shell support:
   ```bash
   west build -b shrike_lite samples/drivers/fpga/fpga_test_shrike -- -DCONFIG_SHELL=y
   ```

2. Use `devmem load` to upload bitstream to memory
3. Use `fpga load <device> <address> <size>` to configure FPGA

## Troubleshooting

- If FPGA device is not ready: Check device tree configuration
- If bitstream load fails: Verify bitstream is correct format for SLG47910V
- If no output: Check UART connection (default: 115200 baud)

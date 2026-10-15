.. zephyr:board:: lemans_cdsp0

Lemans CDSP0 is the Hexagon v73QA_1 compute DSP in the IQ-9075 platform.

The initial target is a native, single-hardware-thread image. It is linked
into the CDSP0 Linux PAS carveout at ``0x9b500000`` with a size of
``0x1e00000``. The intended deployment path is an authorized Qualcomm MBN/PAS
image loaded by Linux remoteproc. The CDSP has no ordinary UART; console
transport uses a fixed RAM diagnostic region. Authorized Trace32/JTAG loading
remains a bring-up and diagnosis fallback.

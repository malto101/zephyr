.. zephyr:board:: iq9075_evk

Overview
********

The Qualcomm IQ-9075 EVK carries the SA8775P (Lemans) SoC. Zephyr runs on
compute DSP 0 (``iq9075_evk/sa8775p/cdsp0``, Hexagon v73) as a guest of the
Hexagon Virtual Machine (HVM), while Linux runs on the application
processors and starts the DSP through remoteproc.

Supported features
==================

- HVM timer and interrupts
- Hexagon MMU, with uncached mappings for devices and SMEM
- RAM console at 0x9d000000 (64 KiB), readable from Linux
- IPCC mailbox (``qcom,ipcc``) and TCSR mutex (``qcom,tcsr-mutex``)
- SMP2P ready/handover, so remoteproc sees the DSP as booted

Memory map
==========

Linux reserves the cdsp0 PAS carveout at 0x9b500000 (30 MiB):

=============  ==========  ==================
Address        Size        Use
=============  ==========  ==================
0x9b800000     8 MiB       HVM
0x9c000000     16 MiB      Zephyr image
0x9d000000     64 KiB      RAM console
=============  ==========  ==================

Building
********

Build HVM's loadlinux for this DSP (H2K_LOAD_ADDR=0x9b800000,
LINUX_LINK_ADDR=0x9c000000, ARCHV=73) and point ``HEXAGON_H2_LOADLINUX`` at
it. The build then also writes ``zephyr/zephyr_hvm.elf``, HVM and Zephyr
merged into one image entering HVM.

.. zephyr-app-commands::
   :zephyr-app: samples/hello_world
   :board: iq9075_evk/sa8775p/cdsp0
   :goals: build

Flashing
********

Sign ``zephyr_hvm.elf`` for the device (``qtestsign`` on test-fused parts),
then on the target::

   cp cdsp0.mbn /lib/firmware/qcom/sa8775p/cdsp0.mbn
   echo stop  > /sys/class/remoteproc/remoteprocN/state
   echo start > /sys/class/remoteproc/remoteprocN/state

and read the console with ``dd if=/dev/mem bs=4096 skip=$((0x9d000)) count=16``.

#!/usr/bin/env python3
# Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
# SPDX-License-Identifier: Apache-2.0

"""Merge HVM and its Zephyr guest into one ELF for Linux remoteproc.

The output keeps the PT_LOAD segments of both inputs at their physical
addresses and enters HVM. It still needs signing (hash segment) before
TZ PAS accepts it.
"""

import argparse
import struct
import sys

EHDR = struct.Struct('<16sHHIIIIIHHHHHH')
PHDR = struct.Struct('<IIIIIIII')
PT_LOAD = 1


def load_segments(path):
    with open(path, 'rb') as f:
        data = f.read()
    ehdr = EHDR.unpack_from(data)
    if ehdr[0][:4] != b'\x7fELF' or ehdr[0][4] != 1 or ehdr[0][5] != 1:
        sys.exit(f'{path}: not a 32-bit little-endian ELF')
    segs = []
    for i in range(ehdr[10]):
        p = PHDR.unpack_from(data, ehdr[5] + i * ehdr[9])
        if p[0] == PT_LOAD and p[5] != 0:
            segs.append((p, data[p[1]:p[1] + p[4]]))
    return ehdr, segs


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--hvm', required=True, help='HVM loadlinux ELF')
    ap.add_argument('--guest', required=True, help='Zephyr ELF')
    ap.add_argument('-o', '--output', required=True)
    args = ap.parse_args()

    ehdr, hvm = load_segments(args.hvm)
    _, guest = load_segments(args.guest)
    segs = sorted(hvm + guest, key=lambda s: s[0][3])

    for (a, _), (b, _) in zip(segs, segs[1:]):
        if a[3] + a[5] > b[3]:
            sys.exit(f'segments overlap: {a[3]:#x}+{a[5]:#x} and {b[3]:#x}')

    phoff = EHDR.size
    off = phoff + PHDR.size * len(segs)
    phdrs = b''
    body = b''
    for p, content in segs:
        align = max(p[7], 1)
        pad = -(off + len(body)) % align
        body += b'\0' * pad
        phdrs += PHDR.pack(p[0], off + len(body), *p[2:])
        body += content

    out = EHDR.pack(ehdr[0], ehdr[1], ehdr[2], ehdr[3], ehdr[4], phoff, 0, ehdr[7],
                    EHDR.size, PHDR.size, len(segs), 0, 0, 0)
    with open(args.output, 'wb') as f:
        f.write(out + phdrs + body)


if __name__ == '__main__':
    main()

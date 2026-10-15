#!/usr/bin/env python3
# Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
# SPDX-License-Identifier: Apache-2.0

"""Validate the layout contract of a native Lemans ELF image."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from elftools.elf.elffile import ELFFile


DIAG_SIZE = 0x1000


def symbol_value(elf: ELFFile, name: str) -> int:
    for section_name in (".symtab", ".dynsym"):
        section = elf.get_section_by_name(section_name)
        if section is None:
            continue
        for symbol in section.iter_symbols():
            if symbol.name == name:
                return int(symbol["st_value"])
    raise ValueError(f"missing ELF symbol: {name}")


def section_range(elf: ELFFile, name: str) -> tuple[int, int]:
    section = elf.get_section_by_name(name)
    if section is None:
        raise ValueError(f"missing ELF section: {name}")
    start = int(section["sh_addr"])
    return start, start + int(section["sh_size"])


def validate(path: Path) -> list[str]:
    errors: list[str] = []
    with path.open("rb") as stream:
        elf = ELFFile(stream)
        entry = int(elf.header["e_entry"])
        try:
            start = symbol_value(elf, "__start")
            image_ram_end = symbol_value(elf, "_image_ram_end")
            end = symbol_value(elf, "_end")
            diag_start = symbol_value(elf, "__lemans_native_diag_start")
            diag_end = symbol_value(elf, "__lemans_native_diag_end")
        except ValueError as error:
            errors.append(str(error))
            return errors

        if entry != start:
            errors.append(f"entry 0x{entry:x} does not equal __start 0x{start:x}")
        if diag_end - diag_start != DIAG_SIZE:
            errors.append("diagnostic section is not exactly 4 KiB")
        if image_ram_end != end:
            errors.append("_image_ram_end and _end differ")
        if end > diag_start:
            errors.append("ordinary RAM extends into the diagnostic region")

        try:
            section_start, section_end = section_range(elf, "lemans_native_diag")
            if section_start != diag_start or section_end != diag_end:
                errors.append("diagnostic section and symbols disagree")
        except ValueError as error:
            errors.append(str(error))

        load_segments = [segment for segment in elf.iter_segments()
                         if segment["p_type"] == "PT_LOAD"]
        if not load_segments:
            errors.append("ELF has no PT_LOAD segments")
        elif not any(segment["p_vaddr"] <= entry <
                     segment["p_vaddr"] + segment["p_memsz"]
                     for segment in load_segments):
            errors.append("entry is not covered by a PT_LOAD segment")

        diag_load = [segment for segment in load_segments
                     if segment["p_vaddr"] <= diag_start <
                     segment["p_vaddr"] + segment["p_memsz"]]
        if len(diag_load) != 1:
            errors.append("diagnostic region is not covered by exactly one PT_LOAD")

    return errors


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("elf", type=Path)
    args = parser.parse_args()
    errors = validate(args.elf)
    if errors:
        for error in errors:
            print(f"lemans ELF validation: {error}", file=sys.stderr)
        return 1
    print(f"lemans ELF validation passed: {args.elf}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

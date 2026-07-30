#!/usr/bin/env python3
"""
ME7 Launch Control Utility

Usage:
    python launch_me7.py <ecu.bin> <dump.ecu>
    launch.exe <ecu.bin> <dump.ecu>

Optional removal:
    python launch_me7.py --remove <ecu.bin> <dump.ecu>

The program:
- reads BIN and ECU files from command-line arguments,
- installs Launch Control when it is not present,
- immediately enters configuration mode,
- detects its own previous installation,
- stores the original FTOMN value,
- lets the user select Soft Cut (FTOMN 0x05) or Hard Cut (FTOMN 0x00),
- restores the original hook bytes and FTOMN when --remove is used,
- never overwrites the input BIN.

Checksums are NOT calculated by this program.
"""

from __future__ import annotations

import argparse
import re
import struct
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Sequence


APP_NAME = "ME7 Launch Control Utility"
APP_VERSION = "1.0"

SUPPORTED_BIN_SIZES = {512 * 1024, 1024 * 1024}

CONFIG_SEARCH_START = 0x17000
CONFIG_SEARCH_END = 0x18000
CONFIG_AREA_SIZE = 64
CODE_CAVE_SIZE = 256
SETZI_FUNCTION_SIZE = 144
SETZI_COMPATIBLE_CODE_START_1MB = 0x70000

DEFAULT_NLS_COUNTER = 0x384FF0

MARKER = b"ME7LC001"
MARKER_OFFSET = 10
METADATA_VERSION_OFFSET = 18
SOFT_CUT_OFFSET = 19
ORIGINAL_FTOMN_OFFSET = 20
FTOMN_ADDRESS_OFFSET = 22
CODE_CAVE_ADDRESS_OFFSET = 26
HOOK_ADDRESS_OFFSET = 30
ORIGINAL_HOOK_BYTES_OFFSET = 34
NORMAL_IGNITION_RAW_OFFSET = 38
METADATA_SIZE = 40

DEFAULT_SPEED_KMH = 3.0
DEFAULT_LAUNCH_RPM = 4500
DEFAULT_RPM_THRESHOLD = 5500
DEFAULT_ACCELERATOR_PERCENT = 90.0
DEFAULT_IGNITION_CUT_MS = 200
FTOMN_SOFT_CUT = 0x05
FTOMN_HARD_CUT = 0x00

TRIGGER_CLUTCH = "Clutch"
TRIGGER_BRAKE = "Brake"
TRIGGER_REFERENCE_OFFSETS = ((1, 3), (43, 45))


class LaunchError(RuntimeError):
    """Raised when the BIN cannot be safely processed."""


@dataclass
class EcuEntry:
    name: str
    columns: List[str]


@dataclass
class Installation:
    config_address: int
    code_cave_address: int
    hook_address: int
    ftomn_address: int
    original_ftomn: int
    original_hook_bytes: bytes
    soft_cut: bool
    normal_ignition_raw: int
    managed: bool = True


@dataclass
class Configuration:
    speed_kmh: float
    launch_rpm: int
    rpm_threshold: int
    accelerator_percent: float
    ignition_cut_ms: int
    soft_cut: bool = False
    minimum_temperature: int = 75
    trigger: str = TRIGGER_CLUTCH


def info(message: str) -> None:
    print(f"[INFO] {message}")


def ok(message: str) -> None:
    print(f"[ OK ] {message}")


def warn(message: str) -> None:
    print(f"[WARN] {message}")


def fail(message: str) -> None:
    print(f"[ERROR] {message}", file=sys.stderr)


def banner() -> None:
    line = "=" * 64
    print(line)
    print("ME7 Launch Control CMD V2".center(64))
    print("Bosch ME7 / ME7.5 Launch Control Installer".center(64))
    print(line)


def parse_int(value: str) -> int:
    value = value.strip()
    return int(value, 16) if value.lower().startswith("0x") else int(value)


def output_name(input_path: Path, suffix: str = "_mod") -> Path:
    stem = input_path.stem
    extension = input_path.suffix or ".bin"
    candidate = input_path.with_name(f"{stem}{suffix}{extension}")
    counter = 2
    while candidate.exists():
        candidate = input_path.with_name(f"{stem}{suffix}_{counter}{extension}")
        counter += 1
    return candidate


def parse_ecu_file(path: Path) -> Dict[str, EcuEntry]:
    try:
        text = path.read_text(encoding="latin-1")
    except OSError as exc:
        raise LaunchError(f"Cannot read ECU file: {exc}") from exc

    entries: Dict[str, EcuEntry] = {}
    for raw_line in text.splitlines():
        line = raw_line.replace("\r", "")
        stripped = line.lstrip()
        if not stripped or stripped[0] in ";#/[":
            continue

        comments: List[str] = []

        def keep_comment(match: re.Match[str]) -> str:
            comments.append(match.group(1))
            return f"#COMMENT{len(comments) - 1}"

        line = re.sub(r"\{([^}]*)\}", keep_comment, line)
        line = line.replace("\t", "").replace(" ", "")
        columns = line.split(",")
        if len(columns) < 10:
            continue

        for index, comment in enumerate(comments):
            columns = [item.replace(f"#COMMENT{index}", comment) for item in columns]

        name = columns[0].lower()
        entries[name] = EcuEntry(name=name, columns=columns[1:])

    if not entries:
        raise LaunchError(
            "The ECU file contains no usable entries. Create it with ME7Info using: "
            "me7info -n file.bin"
        )
    return entries


def ecu_address(entries: Dict[str, EcuEntry], name: str) -> Optional[int]:
    entry = entries.get(name.lower())
    if entry is None or len(entry.columns) < 2:
        return None

    value = entry.columns[1].strip()
    try:
        return int(value, 16)
    except ValueError:
        return None


def ecu_mask(entries: Dict[str, EcuEntry], name: str) -> Optional[int]:
    entry = entries.get(name.lower())
    if entry is None or len(entry.columns) < 4:
        return None

    value = entry.columns[3].strip().lower()
    if value.startswith("0x"):
        value = value[2:]
    try:
        mask = int(value, 16)
    except ValueError:
        return None

    if mask <= 0 or mask & (mask - 1):
        return None
    return mask.bit_length() - 1


def require_address(entries: Dict[str, EcuEntry], name: str) -> int:
    value = ecu_address(entries, name)
    if value is None:
        raise LaunchError(f"Required ECU variable not found: {name}")
    ok(f"{name} found at 0x{value:06X}")
    return value


def resolve_wped(entries: Dict[str, EcuEntry]) -> int:
    wped = ecu_address(entries, "wped")
    if wped is not None:
        ok(f"wped found at 0x{wped:06X}")
        return wped

    dwped = ecu_address(entries, "dwped")
    if dwped is not None:
        wped = dwped + 2
        ok(f"wped derived from dwped + 2: 0x{wped:06X}")
        return wped

    raise LaunchError("Required ECU variable not found: wped or dwped")


def require_bit(entries: Dict[str, EcuEntry], name: str) -> tuple[int, int]:
    address = ecu_address(entries, name)
    bit = ecu_mask(entries, name)
    if address is None or bit is None:
        raise LaunchError(f"Required ECU bit variable not found or invalid: {name}")
    ok(f"{name} found at 0x{address:06X}.{bit}")
    return address, bit


def is_ecu_address_used(entries: Dict[str, EcuEntry], absolute_address: int) -> Optional[str]:
    wanted = f"0x{absolute_address:X}".lower()
    for name, entry in entries.items():
        if any(item.lower() == wanted for item in entry.columns):
            return name
    return None


def find_hole(
    data: Sequence[int],
    size: int,
    start: int = 0,
    end: Optional[int] = None,
    alignment: int = 16,
) -> Optional[int]:
    if end is None or end >= len(data):
        end = len(data) - 64
    start = max(0, start)

    run_start: Optional[int] = None
    for index in range(start, end):
        if data[index] == 0xFF:
            if run_start is None:
                run_start = index
        else:
            if run_start is not None:
                aligned = (run_start + alignment - 1) & ~(alignment - 1)
                if aligned + size <= index:
                    return aligned
            run_start = None

    if run_start is not None:
        aligned = (run_start + alignment - 1) & ~(alignment - 1)
        if aligned + size <= end:
            return aligned
    return None


def find_free_bool(data: bytes) -> int:
    hex_data = data.hex()

    for safe in (True, False):
        for candidate in range(1, 127):
            token = f"{candidate:x}"
            if len(token) == 1:
                token = "0" + token

            if safe:
                matches = list(re.finditer(rf"9a{token}...0", hex_data))
                if any(match.start() % 2 != 0 for match in matches):
                    continue

            matches = list(re.finditer(rf"8a{token}...0", hex_data))
            if any(match.start() % 2 != 0 for match in matches):
                continue

            return candidate

        if safe:
            warn("No safely unused status flag was found; trying relaxed search.")

    raise LaunchError("Cannot find an unused status flag variable.")


def find_ftomn(data: bytes, memory_layout_kb: int) -> List[int]:
    found: List[int] = []
    limit = len(data)

    for index in range(0, max(0, limit - 26)):
        if (
            data[index] == 0x05
            and data[index + 1] != 0x05
            and data[index + 11] == 0x05
            and data[index + 24] == 0x08
            and data[index + 25] == 0x05
        ):
            found.append(index + 22)

    if not found:
        for index in range(0, max(0, limit - 13)):
            if (
                data[index] == 0x05
                and data[index + 1] != 0x05
                and data[index + 11] == 0x05
                and data[index + 12] == 0x07
            ):
                found.append(index + 11)

    if not found and memory_layout_kb == 512:
        pattern = re.compile(b"\xC2\xF4..\x40\x94\x9D\x02\xC2\xF9", re.DOTALL)
        matches = list(pattern.finditer(data))
        if matches:
            position = matches[-1].start() + 10
            if position + 2 <= len(data):
                raw = data[position : position + 2]
                address = int("1" + raw[::-1].hex(), 16)
                if address < len(data):
                    found.append(address)

    return sorted(set(found))


def find_hook(data: bytes, memory_layout_kb: int) -> int:
    if memory_layout_kb == 1024:
        signature = b"\xD7\x40\x06\x02\x03\xF8"
        locations: List[int] = []
        start = 0
        while True:
            position = data.find(signature, start)
            if position < 0:
                break
            if position >= 4:
                locations.append(position - 4)
            start = position + 1
        if not locations:
            raise LaunchError("Cannot find the Launch Control hook signature.")
        return locations[-1]

    pattern = re.compile(b"\xF0\x49\xF7\xF8..\xF3\xF8", re.DOTALL)
    matches = list(pattern.finditer(data))
    if not matches:
        raise LaunchError("Cannot find the Launch Control hook signature.")
    return matches[-1].start() + 6


def encode_calls_address(code_cave: int) -> bytes:
    address = code_cave + 0x800000
    text = f"{address:06x}"
    return bytes.fromhex(text[0:2] + text[4:6] + text[2:4])


def word_little(value: int) -> bytes:
    return struct.pack("<H", value & 0xFFFF)


def flash_operand(value: int) -> bytes:
    return word_little(value)


def ram_operand_from_absolute(value: int) -> bytes:
    return word_little(value + 0x8000)


def direct_operand_from_hex_address(value: int) -> bytes:
    return word_little(value)


def bit_register_offset(address: int) -> int:
    relative = address - 0xFD00
    if relative < 0 or relative % 2:
        raise LaunchError(f"Invalid bit-register address: 0x{address:06X}")
    result = relative // 2
    if not 0 <= result <= 0xFF:
        raise LaunchError(f"Bit-register address is outside supported range: 0x{address:06X}")
    return result


def bit_nibble(bit: int) -> int:
    if not 0 <= bit <= 15:
        raise LaunchError(f"Invalid bit number: {bit}")
    return bit << 4


def trigger_reference_bytes(address: int, bit: int) -> tuple[int, int]:
    """Return the C167 bit-register operand bytes for a trigger input."""
    return bit_register_offset(address), bit_nibble(bit)


def detect_launch_trigger(
    data: bytes | bytearray,
    function_address: int,
    clutch_address: int,
    clutch_bit: int,
    brake_address: int,
    brake_bit: int,
) -> str:
    """Detect whether the installed Launch Control uses Clutch or Brake."""
    if function_address < 0 or function_address + SETZI_FUNCTION_SIZE > len(data):
        raise LaunchError("The Launch Control function is outside the BIN file.")

    clutch_reference = trigger_reference_bytes(clutch_address, clutch_bit)
    brake_reference = trigger_reference_bytes(brake_address, brake_bit)

    references = []
    for address_offset, bit_offset in TRIGGER_REFERENCE_OFFSETS:
        references.append(
            (
                data[function_address + address_offset],
                data[function_address + bit_offset],
            )
        )

    if all(reference == clutch_reference for reference in references):
        return TRIGGER_CLUTCH
    if all(reference == brake_reference for reference in references):
        return TRIGGER_BRAKE

    raise LaunchError(
        "The current Launch Control trigger could not be identified safely. "
        "The trigger references in the installed function are inconsistent."
    )


def write_launch_trigger(
    data: bytearray,
    function_address: int,
    trigger: str,
    clutch_address: int,
    clutch_bit: int,
    brake_address: int,
    brake_bit: int,
) -> None:
    """Change the installed Launch Control trigger without rebuilding the patch."""
    if function_address < 0 or function_address + SETZI_FUNCTION_SIZE > len(data):
        raise LaunchError("The Launch Control function is outside the BIN file.")

    # These are the two trigger bit-test instructions in the Setzi function.
    if data[function_address] not in (0x9A, 0x8A):
        raise LaunchError("Unexpected first trigger instruction in Launch Control code.")
    if data[function_address + 42] not in (0x9A, 0x8A):
        raise LaunchError("Unexpected second trigger instruction in Launch Control code.")

    if trigger == TRIGGER_CLUTCH:
        address, bit = clutch_address, clutch_bit
    elif trigger == TRIGGER_BRAKE:
        address, bit = brake_address, brake_bit
    else:
        raise LaunchError(f"Unsupported trigger selection: {trigger}")

    address_byte, bit_byte = trigger_reference_bytes(address, bit)
    for address_offset, bit_offset in TRIGGER_REFERENCE_OFFSETS:
        data[function_address + address_offset] = address_byte
        data[function_address + bit_offset] = bit_byte

    verified = detect_launch_trigger(
        data,
        function_address,
        clutch_address,
        clutch_bit,
        brake_address,
        brake_bit,
    )
    if verified != trigger:
        raise LaunchError(
            f"Trigger verification failed: expected {trigger}, detected {verified}."
        )


def nls_counter_operand(absolute_address: int) -> bytes:
    relative = absolute_address - 0x380000
    if not 0 <= relative <= 0xFFFF:
        raise LaunchError(
            f"NLS counter 0x{absolute_address:06X} is outside the supported 0x38xxxx range."
        )
    return struct.pack("<H", relative)


def append(buffer: bytearray, *parts: int | bytes) -> None:
    for part in parts:
        if isinstance(part, int):
            buffer.append(part & 0xFF)
        else:
            buffer.extend(part)


def build_launch_code(
    *,
    config_address: int,
    tsrldyn: int,
    vfil_w: int,
    nmot_w: int,
    wped: int,
    clutch_address: int,
    clutch_bit: int,
    brake_address: int,
    brake_bit: int,
    nls_counter_address: int,
    original_hook_bytes_2_3: bytes,
) -> bytes:
    """
    Build the C167 machine code used by the original PHP utility.

    The temperature precondition from the PHP modification is deliberately
    omitted. This version starts directly with the LC/NLS conditions.
    """
    code = bytearray()
    nls = nls_counter_operand(nls_counter_address)

    append(
        code,
        0x9A,
        bit_register_offset(clutch_address),
        0x13,
        bit_nibble(clutch_bit),
        0xF2,
        0xF4,
        ram_operand_from_absolute(vfil_w),
        0xD7,
        0x00,
        0x81,
        0x00,
        0xF2,
        0xF9,
        flash_operand(config_address),
    )

    append(
        code,
        0x40,
        0x49,
        0x9D,
        0x0B,
        0xF2,
        0xF4,
        direct_operand_from_hex_address(nmot_w),
        0xD7,
        0x00,
        0x81,
        0x00,
        0xF2,
        0xF9,
        flash_operand(config_address + 2),
    )

    append(
        code,
        0x40,
        0x49,
        0xFD,
        0x03,
        0xF7,
        0x8E,
        ram_operand_from_absolute(tsrldyn),
        0x0D,
        0x2F,
        0x9A,
        bit_register_offset(clutch_address),
        0x29,
        bit_nibble(clutch_bit),
        0x8A,
        bit_register_offset(brake_address),
    )

    append(
        code,
        0x22,
        bit_nibble(brake_bit),
        0xF2,
        0xF4,
        direct_operand_from_hex_address(nmot_w),
        0xD7,
        0x00,
        0x81,
        0x00,
        0xF2,
        0xF9,
        flash_operand(config_address + 6),
        0x40,
        0x49,
    )

    append(
        code,
        0xFD,
        0x1A,
        0xC2,
        0xF4,
        ram_operand_from_absolute(wped),
        0xD7,
        0x00,
        0x81,
        0x00,
        0xC2,
        0xF9,
        flash_operand(config_address + 8),
        0x40,
        0x49,
    )

    append(
        code,
        0xFD,
        0x12,
        0xD7,
        0x00,
        0x38,
        0x00,
        0xF2,
        0xF4,
        nls,
        0xD7,
        0x00,
        0x81,
        0x00,
        0xF2,
        0xF9,
    )

    append(
        code,
        flash_operand(config_address + 4),
        0x40,
        0x49,
        0x9D,
        0x11,
        0xF7,
        0x8E,
        ram_operand_from_absolute(tsrldyn),
        0x08,
        0x41,
        0xD7,
        0x00,
        0x38,
    )

    append(
        code,
        0x00,
        0xF7,
        0xF8,
        nls,
        0x0D,
        0x09,
        0xD7,
        0x00,
        0x38,
        0x00,
        0xF6,
        0x8F,
        nls,
        0x0D,
        0x04,
    )

    append(
        code,
        0xD7,
        0x00,
        0x38,
        0x00,
        0xF6,
        0x8E,
        nls,
        0xF3,
        0xF8,
        original_hook_bytes_2_3,
        0xDB,
        0x00,
    )

    if len(code) > CODE_CAVE_SIZE:
        raise LaunchError(
            f"Generated code is too large ({len(code)} bytes; maximum is {CODE_CAVE_SIZE})."
        )
    return bytes(code)


def encode_speed(value: float) -> int:
    raw = round(value / 0.0078125)
    if not 0 <= raw <= 0xFFFF:
        raise ValueError("Speed Threshold is outside the supported range.")
    return raw


def decode_speed(raw: int) -> float:
    return raw * 0.0078125


def encode_rpm(value: int) -> int:
    raw = round(value / 0.25)
    if not 0 <= raw <= 0xFFFF:
        raise ValueError("RPM value is outside the supported range.")
    return raw


def decode_rpm(raw: int) -> int:
    return round(raw * 0.25)


def encode_ignition_ms(value: int) -> int:
    if value < 0 or value % 20 != 0:
        raise ValueError("Ignition Cut Duration must be a multiple of 20 ms.")
    raw = value // 20
    if not 0 <= raw <= 0xFFFF:
        raise ValueError("Ignition Cut Duration is outside the supported range.")
    return raw


def decode_ignition_ms(raw: int) -> int:
    return raw * 20


def encode_pedal(value: float) -> int:
    raw = round(value / 0.392157)
    if not 0 <= raw <= 0xFF:
        raise ValueError("Accelerator Threshold must be between 0 and 100 percent.")
    return raw


def decode_pedal(raw: int) -> float:
    return raw * 0.392157




def encode_temperature(value: int) -> int:
    raw = round((value + 48) / 0.75)
    if not 0 <= raw <= 0xFF:
        raise ValueError("Minimum Temperature is outside the supported range.")
    return raw


def decode_temperature(raw: int) -> int:
    return round(raw * 0.75 - 48)

def read_configuration(data: bytes | bytearray, address: int) -> Configuration:
    speed_raw, launch_raw, ignition_raw, rpm_threshold_raw = struct.unpack_from(
        "<HHHH", data, address
    )
    pedal_raw = data[address + 8]
    temperature_raw = data[address + 9]
    soft_cut = bool(data[address + SOFT_CUT_OFFSET])

    return Configuration(
        speed_kmh=decode_speed(speed_raw),
        launch_rpm=decode_rpm(launch_raw),
        rpm_threshold=decode_rpm(rpm_threshold_raw),
        accelerator_percent=decode_pedal(pedal_raw),
        ignition_cut_ms=decode_ignition_ms(ignition_raw),
        minimum_temperature=decode_temperature(temperature_raw),
        soft_cut=soft_cut,
    )


def write_configuration(
    data: bytearray,
    address: int,
    configuration: Configuration,
    normal_ignition_raw: int,
) -> int:
    ignition_raw = encode_ignition_ms(configuration.ignition_cut_ms)

    struct.pack_into(
        "<HHHHB",
        data,
        address,
        encode_speed(configuration.speed_kmh),
        encode_rpm(configuration.launch_rpm),
        ignition_raw,
        encode_rpm(configuration.rpm_threshold),
        encode_pedal(configuration.accelerator_percent),
    )
    data[address + 9] = encode_temperature(configuration.minimum_temperature)
    data[address + SOFT_CUT_OFFSET] = 1 if configuration.soft_cut else 0
    struct.pack_into("<H", data, address + NORMAL_IGNITION_RAW_OFFSET, normal_ignition_raw)
    return ignition_raw


def write_metadata(
    data: bytearray,
    *,
    config_address: int,
    code_cave_address: int,
    hook_address: int,
    ftomn_address: int,
    original_ftomn: int,
    original_hook_bytes: bytes,
    soft_cut: bool,
    normal_ignition_raw: int,
) -> None:
    data[config_address + MARKER_OFFSET : config_address + MARKER_OFFSET + len(MARKER)] = MARKER
    data[config_address + METADATA_VERSION_OFFSET] = 1
    data[config_address + SOFT_CUT_OFFSET] = 1 if soft_cut else 0
    data[config_address + ORIGINAL_FTOMN_OFFSET] = original_ftomn
    data[config_address + 21] = 0xFF
    struct.pack_into("<I", data, config_address + FTOMN_ADDRESS_OFFSET, ftomn_address)
    struct.pack_into("<I", data, config_address + CODE_CAVE_ADDRESS_OFFSET, code_cave_address)
    struct.pack_into("<I", data, config_address + HOOK_ADDRESS_OFFSET, hook_address)
    data[
        config_address + ORIGINAL_HOOK_BYTES_OFFSET :
        config_address + ORIGINAL_HOOK_BYTES_OFFSET + 4
    ] = original_hook_bytes
    struct.pack_into("<H", data, config_address + NORMAL_IGNITION_RAW_OFFSET, normal_ignition_raw)


def detect_installation(data: bytes) -> Optional[Installation]:
    start = 0
    while True:
        marker_position = data.find(MARKER, start)
        if marker_position < 0:
            return None

        config_address = marker_position - MARKER_OFFSET
        start = marker_position + 1

        if config_address < 0 or config_address + METADATA_SIZE > len(data):
            continue

        try:
            ftomn_address = struct.unpack_from(
                "<I", data, config_address + FTOMN_ADDRESS_OFFSET
            )[0]
            code_cave_address = struct.unpack_from(
                "<I", data, config_address + CODE_CAVE_ADDRESS_OFFSET
            )[0]
            hook_address = struct.unpack_from(
                "<I", data, config_address + HOOK_ADDRESS_OFFSET
            )[0]
            normal_ignition_raw = struct.unpack_from(
                "<H", data, config_address + NORMAL_IGNITION_RAW_OFFSET
            )[0]
        except struct.error:
            continue

        if not (
            0 <= ftomn_address < len(data)
            and 0 <= code_cave_address < len(data)
            and 0 <= hook_address + 4 <= len(data)
        ):
            continue

        if data[hook_address] != 0xDA:
            continue

        return Installation(
            config_address=config_address,
            code_cave_address=code_cave_address,
            hook_address=hook_address,
            ftomn_address=ftomn_address,
            original_ftomn=data[config_address + ORIGINAL_FTOMN_OFFSET],
            original_hook_bytes=data[
                config_address + ORIGINAL_HOOK_BYTES_OFFSET :
                config_address + ORIGINAL_HOOK_BYTES_OFFSET + 4
            ],
            soft_cut=bool(data[config_address + SOFT_CUT_OFFSET]),
            normal_ignition_raw=normal_ignition_raw,
            managed=True,
        )



LEGACY_FUNCTION_LENGTH = 144
LEGACY_CONDITION_OFFSETS = {
    "SpeedThreshold": 14,
    "LaunchRPM": 30,
    "RPMThreshold": 60,
    "AcceleratorThreshold": 76,
    "IgnitionCutDuration": 96,
}


def read_legacy_configuration(
    data: bytes | bytearray,
    address: int,
    ftomn_value: int,
) -> Configuration:
    """Read only the original 9-byte Setzi configuration block."""
    speed_raw, launch_raw, ignition_raw, rpm_threshold_raw = struct.unpack_from(
        "<HHHH", data, address
    )
    pedal_raw = data[address + 8]

    return Configuration(
        speed_kmh=decode_speed(speed_raw),
        launch_rpm=decode_rpm(launch_raw),
        rpm_threshold=decode_rpm(rpm_threshold_raw),
        accelerator_percent=decode_pedal(pedal_raw),
        ignition_cut_ms=decode_ignition_ms(ignition_raw),
        soft_cut=(ftomn_value == FTOMN_SOFT_CUT),
    )


def write_legacy_configuration(
    data: bytearray,
    address: int,
    configuration: Configuration,
) -> int:
    """
    Update only the original Setzi configuration bytes.

    No ME7LC001 marker or restoration metadata is written because the area after
    the 9-byte legacy block may contain unrelated calibration data.
    """
    ignition_raw = encode_ignition_ms(configuration.ignition_cut_ms)
    struct.pack_into(
        "<HHHHB",
        data,
        address,
        encode_speed(configuration.speed_kmh),
        encode_rpm(configuration.launch_rpm),
        ignition_raw,
        encode_rpm(configuration.rpm_threshold),
        encode_pedal(configuration.accelerator_percent),
    )
    return ignition_raw


def decode_calls_target(data: bytes | bytearray, hook_address: int) -> Optional[int]:
    """Decode C167 CALLS bytes DA bank low high to a BIN file offset."""
    if hook_address < 0 or hook_address + 4 > len(data):
        return None
    if data[hook_address] != 0xDA:
        return None

    bank = data[hook_address + 1]
    low = data[hook_address + 2]
    high = data[hook_address + 3]
    return ((bank << 16) | (high << 8) | low) & 0xFFFFF


def normalize_legacy_config_address(
    data: bytes | bytearray,
    reference: int,
) -> Optional[int]:
    candidates: List[int] = []

    if reference < 0x10000:
        candidates.append(reference + 0x10000)
    candidates.append(reference)

    for candidate in candidates:
        if not (0 <= candidate <= len(data) - 9):
            continue

        block = bytes(data[candidate : candidate + 9])
        if block in (b"\x00" * 9, b"\xFF" * 9):
            continue

        try:
            speed_raw, launch_raw, ignition_raw, rpm_raw = struct.unpack_from(
                "<HHHH", data, candidate
            )
            pedal_raw = data[candidate + 8]
        except (struct.error, IndexError):
            continue

        # Broad plausibility checks prevent random machine code from being
        # mistaken for a configuration block.
        speed = decode_speed(speed_raw)
        launch_rpm = decode_rpm(launch_raw)
        ignition_ms = decode_ignition_ms(ignition_raw)
        rpm_threshold = decode_rpm(rpm_raw)
        pedal = decode_pedal(pedal_raw)

        if not (0 <= speed <= 511):
            continue
        if not (500 <= launch_rpm <= 16000):
            continue
        if not (0 <= ignition_ms <= 5000):
            continue
        if not (500 <= rpm_threshold <= 16000):
            continue
        if not (0 <= pedal <= 100.5):
            continue

        return candidate

    return None


def find_compatible_setzi_installation(
    data: bytes,
    memory_layout_kb: int,
) -> Optional[Installation]:
    """
    Detect a Launch Control patch installed by launch.php/launch.exe or another
    compatible Setzi-based tool, even when the ME7LC001 marker is absent.
    """
    scan_start = 0x70000 if len(data) > 0x80000 else 0
    scan_end = max(scan_start, len(data) - LEGACY_FUNCTION_LENGTH)

    ftomn_candidates = find_ftomn(data, memory_layout_kb)
    if not ftomn_candidates:
        return None

    for function_address in range(scan_start, scan_end):
        if data[function_address] not in (0x9A, 0x8A):
            continue

        end = function_address + LEGACY_FUNCTION_LENGTH
        if end > len(data):
            break

        # Structural signatures used by the Setzi LC/NLS implementation.
        if data[function_address + 4 : function_address + 6] != b"\xF2\xF4":
            continue
        if data[function_address + 16 : function_address + 20] != b"\x40\x49\x9D\x0B":
            continue

        refs = {
            name: int.from_bytes(
                data[
                    function_address + offset :
                    function_address + offset + 2
                ],
                "little",
            )
            for name, offset in LEGACY_CONDITION_OFFSETS.items()
        }

        base_ref = refs["SpeedThreshold"]
        if not (
            refs["LaunchRPM"] == ((base_ref + 2) & 0xFFFF)
            and refs["IgnitionCutDuration"] == ((base_ref + 4) & 0xFFFF)
            and refs["RPMThreshold"] == ((base_ref + 6) & 0xFFFF)
            and refs["AcceleratorThreshold"] == ((base_ref + 8) & 0xFFFF)
        ):
            continue

        config_address = normalize_legacy_config_address(data, base_ref)
        if config_address is None:
            continue

        hook_address: Optional[int] = None
        search_from = 0
        signature = b"\xD7\x40\x06\x02\x03\xF8"
        while True:
            signature_position = data.find(signature, search_from)
            if signature_position < 0:
                break
            candidate_hook = signature_position - 4
            if (
                candidate_hook >= 0
                and decode_calls_target(data, candidate_hook) == function_address
            ):
                hook_address = candidate_hook
                break
            search_from = signature_position + 1

        if hook_address is None:
            continue

        ftomn_address = ftomn_candidates[0]
        ftomn_value = data[ftomn_address]
        ignition_raw = int.from_bytes(
            data[config_address + 4 : config_address + 6],
            "little",
        )

        return Installation(
            config_address=config_address,
            code_cave_address=function_address,
            hook_address=hook_address,
            ftomn_address=ftomn_address,
            original_ftomn=FTOMN_SOFT_CUT,
            original_hook_bytes=b"",
            soft_cut=(ftomn_value == FTOMN_SOFT_CUT),
            normal_ignition_raw=ignition_raw,
            managed=False,
        )

    return None

def default_configuration() -> Configuration:
    return Configuration(
        speed_kmh=DEFAULT_SPEED_KMH,
        launch_rpm=DEFAULT_LAUNCH_RPM,
        rpm_threshold=DEFAULT_RPM_THRESHOLD,
        accelerator_percent=DEFAULT_ACCELERATOR_PERCENT,
        ignition_cut_ms=DEFAULT_IGNITION_CUT_MS,
        minimum_temperature=75,
        soft_cut=False,
    )



def safe_input(prompt: str) -> str:
    """Read console input and provide a clear error when stdin is unavailable."""
    try:
        return input(prompt)
    except EOFError as exc:
        raise LaunchError(
            "Console input is unavailable. Run under Wine with: "
            "wineconsole --backend=curses launch.exe test.bin test.ecu"
        ) from exc


def prompt_float(label: str, current: float, minimum: float, maximum: float) -> float:
    while True:
        raw = safe_input(f"{label} [{current:g}]: ").strip()
        if not raw:
            return current
        try:
            value = float(raw.replace(",", "."))
        except ValueError:
            print("  Invalid number. Try again.")
            continue
        if not minimum <= value <= maximum:
            print(f"  Enter a value from {minimum:g} to {maximum:g}.")
            continue
        return value


def prompt_int(
    label: str,
    current: int,
    minimum: int,
    maximum: int,
    multiple: Optional[int] = None,
) -> int:
    while True:
        raw = safe_input(f"{label} [{current}]: ").strip()
        if not raw:
            return current
        try:
            value = int(raw)
        except ValueError:
            print("  Invalid integer. Try again.")
            continue
        if not minimum <= value <= maximum:
            print(f"  Enter a value from {minimum} to {maximum}.")
            continue
        if multiple and value % multiple:
            print(f"  The value must be a multiple of {multiple}.")
            continue
        return value


def prompt_yes_no(label: str, current: bool) -> bool:
    default = "Y" if current else "N"
    while True:
        raw = safe_input(f"{label} [Y/N] [{default}]: ").strip().lower()
        if not raw:
            return current
        if raw in {"y", "yes"}:
            return True
        if raw in {"n", "no"}:
            return False
        print("  Enter Y or N.")


def prompt_trigger(current: str) -> str:
    """Prompt for the Launch Control activation trigger."""
    while True:
        print()
        print("ACTIVATION TRIGGER")
        print("  1. Clutch Manual Transmission")
        print("  2. Brake >>DSG Only<<")
        default = "1" if current == TRIGGER_CLUTCH else "2"
        raw = safe_input(
            f"Select Activation Trigger [1/2] [{default}]: "
        ).strip().lower()

        if not raw:
            return current
        if raw in {"1", "clutch", "c"}:
            return TRIGGER_CLUTCH
        if raw in {"2", "brake", "b"}:
            return TRIGGER_BRAKE
        print("  Enter 1 for Clutch or 2 for Brake.")


def configure(current: Configuration, normal_ignition_ms: int) -> Configuration:
    print()
    print("=" * 64)
    print("LAUNCH CONTROL CONFIGURATION".center(64))
    print("=" * 64)
    print()
    print("Current configuration:")
    print(f"  Speed Threshold           : {current.speed_kmh:.2f} km/h")
    print(f"  Launch RPM                : {current.launch_rpm} rpm")
    print(f"  RPM Threshold             : {current.rpm_threshold} rpm")
    print(f"  Accelerator Threshold     : {current.accelerator_percent:.1f} %")
    print(f"  Ignition Cut Duration     : {current.ignition_cut_ms} ms")
    print(f"  Minimum Temperature       : {current.minimum_temperature} °C")
    print(f"  Activation Trigger        : {current.trigger}")
    print(
        f"  Cut Mode >>Experimental<< : "
        f"{'Soft Cut (FTOMN 0x05)' if current.soft_cut else 'Hard Cut (FTOMN 0x00)'}"
    )
    print()
    print("Press ENTER to keep the current value.")
    print()

    speed = prompt_float("Speed Threshold (km/h)", current.speed_kmh, 0.0, 511.0)
    launch_rpm = prompt_int("Launch RPM", current.launch_rpm, 500, 16000)
    rpm_threshold = prompt_int(
        "RPM Threshold", current.rpm_threshold, 500, 16000
    )
    accelerator = prompt_float(
        "Accelerator Threshold (%)", current.accelerator_percent, 0.0, 100.0
    )
    ignition_ms = prompt_int(
        "Ignition Cut Duration (ms)",
        current.ignition_cut_ms if current.ignition_cut_ms > 0 else normal_ignition_ms,
        20,
        5000,
        multiple=20,
    )

    minimum_temperature = prompt_int(
        "Minimum Temperature (°C)",
        current.minimum_temperature,
        0,
        120,
    )

    trigger = prompt_trigger(current.trigger)

    print()
    print("CUT MODE")
    print("  Soft Cut ON  = FTOMN 0x05 (stock value)")
    print("  Soft Cut OFF = FTOMN 0x00 (hard cut)")
    soft_cut = prompt_yes_no("Enable Soft Cut?", current.soft_cut)

    result = Configuration(
        speed_kmh=speed,
        launch_rpm=launch_rpm,
        rpm_threshold=rpm_threshold,
        accelerator_percent=accelerator,
        ignition_cut_ms=ignition_ms,
        minimum_temperature=minimum_temperature,
        soft_cut=soft_cut,
        trigger=trigger,
    )

    selected_ftomn = FTOMN_SOFT_CUT if result.soft_cut else FTOMN_HARD_CUT

    print()
    print("=" * 64)
    print("CONFIGURATION SUMMARY".center(64))
    print("=" * 64)
    print(f"  Speed Threshold        : {result.speed_kmh:.2f} km/h")
    print(f"  Launch RPM             : {result.launch_rpm} rpm")
    print(f"  RPM Threshold          : {result.rpm_threshold} rpm")
    print(f"  Accelerator Threshold  : {result.accelerator_percent:.1f} %")
    print(f"  Ignition Cut Duration  : {result.ignition_cut_ms} ms")
    print(f"  Minimum Temperature    : {result.minimum_temperature} °C")
    print(f"  Activation Trigger     : {result.trigger}")
    print(
        f"  Cut Mode               : "
        f"{'Soft Cut' if result.soft_cut else 'Hard Cut'}"
    )
    print(f"  FTOMN Value            : 0x{selected_ftomn:02X}")
    print()

    if not prompt_yes_no("Save this configuration?", True):
        raise LaunchError("Operation cancelled by the user.")

    return result



def find_compatible_code_cave(
    data: bytes | bytearray,
    memory_layout_kb: int,
    *,
    exclude_start: Optional[int] = None,
    exclude_size: int = 0,
) -> Optional[int]:
    """
    Find a code cave in the range scanned by common Setzi/ME7 desktop tools.

    For a 1 MB BIN, compatible tools normally scan the upper calibration/code
    area from 0x70000. A cave below that range can work in the ECU but remain
    invisible to those tools.
    """
    start = SETZI_COMPATIBLE_CODE_START_1MB if memory_layout_kb == 1024 else 0
    end = len(data)

    position = start
    while position + CODE_CAVE_SIZE <= end:
        candidate = find_hole(
            data,
            CODE_CAVE_SIZE,
            start=position,
            end=end,
        )
        if candidate is None:
            return None

        if (
            exclude_start is not None
            and candidate < exclude_start + exclude_size
            and exclude_start < candidate + CODE_CAVE_SIZE
        ):
            position = candidate + CODE_CAVE_SIZE
            continue

        return candidate

    return None


def migrate_native_installation_to_setzi_visible_area(
    data: bytearray,
    installation: Installation,
    memory_layout_kb: int,
) -> Installation:
    """
    Move a native ME7LC001 installation to a code cave visible to compatible
    Setzi scanners when an older CMD version placed it too low in the BIN.

    The Launch function contains no self-relative reference to its own location,
    so copying the function and changing CALLS is sufficient.
    """
    if not installation.managed:
        return installation

    if memory_layout_kb != 1024:
        return installation

    if installation.code_cave_address >= SETZI_COMPATIBLE_CODE_START_1MB:
        return installation

    info(
        f"Native code cave 0x{installation.code_cave_address:X} is below "
        "the range scanned by compatible Setzi tools."
    )
    info("Migrating Launch Control to a compatible upper code cave.")

    new_code_cave = find_compatible_code_cave(
        data,
        memory_layout_kb,
        exclude_start=installation.config_address,
        exclude_size=CONFIG_AREA_SIZE,
    )
    if new_code_cave is None:
        raise LaunchError(
            "Cannot find a compatible 256-byte code cave above 0x70000."
        )

    old_code_cave = installation.code_cave_address
    function = bytes(
        data[old_code_cave : old_code_cave + SETZI_FUNCTION_SIZE]
    )
    if len(function) != SETZI_FUNCTION_SIZE:
        raise LaunchError("Cannot read the complete existing Launch function.")

    data[new_code_cave : new_code_cave + SETZI_FUNCTION_SIZE] = function
    data[
        new_code_cave + SETZI_FUNCTION_SIZE :
        new_code_cave + CODE_CAVE_SIZE
    ] = b"\xFF" * (CODE_CAVE_SIZE - SETZI_FUNCTION_SIZE)

    data[
        installation.hook_address :
        installation.hook_address + 4
    ] = b"\xDA" + encode_calls_address(new_code_cave)

    # Clear only the old code cave. Configuration and metadata live elsewhere.
    data[
        old_code_cave :
        old_code_cave + CODE_CAVE_SIZE
    ] = b"\xFF" * CODE_CAVE_SIZE

    installation.code_cave_address = new_code_cave

    write_metadata(
        data,
        config_address=installation.config_address,
        code_cave_address=new_code_cave,
        hook_address=installation.hook_address,
        ftomn_address=installation.ftomn_address,
        original_ftomn=installation.original_ftomn,
        original_hook_bytes=installation.original_hook_bytes,
        soft_cut=installation.soft_cut,
        normal_ignition_raw=installation.normal_ignition_raw,
    )

    verified_native = detect_installation(bytes(data))
    if verified_native is None:
        raise LaunchError("Native installation migration verification failed.")

    verified_legacy = find_compatible_setzi_installation(
        bytes(data),
        memory_layout_kb,
    )
    if verified_legacy is None:
        raise LaunchError(
            "The migrated installation is still not visible to a compatible "
            "Setzi detector."
        )

    ok(
        f"Launch function migrated: 0x{old_code_cave:X} "
        f"-> 0x{new_code_cave:X}"
    )
    ok("Compatible Setzi detection verified")
    return verified_native

def install(
    data: bytearray,
    entries: Dict[str, EcuEntry],
    memory_layout_kb: int,
    nls_counter_address: int,
) -> Installation:
    info("Launch Control not detected.")
    info("Starting installation.")

    tsrldyn = require_address(entries, "tsrldyn")
    vfil_w = require_address(entries, "vfil_w")
    nmot_w = require_address(entries, "nmot_w")
    wped = resolve_wped(entries)
    clutch_address, clutch_bit = require_bit(entries, "b_kuppl")
    brake_address, brake_bit = require_bit(entries, "b_br")

    used_by = is_ecu_address_used(entries, nls_counter_address)
    if used_by:
        raise LaunchError(
            f"NLS counter address 0x{nls_counter_address:06X} is used by '{used_by}'."
        )

    free_bool = find_free_bool(bytes(data))
    ok(f"Unused status flag found at 0x00FD{free_bool * 2:02X}")

    ftomn_candidates = find_ftomn(bytes(data), memory_layout_kb)
    if not ftomn_candidates:
        raise LaunchError("FTOMN could not be found safely.")
    if len(ftomn_candidates) > 1:
        warn(
            "Multiple FTOMN candidates found: "
            + ", ".join(f"0x{x:X}" for x in ftomn_candidates)
        )
        warn(f"Using the first candidate: 0x{ftomn_candidates[0]:X}")

    ftomn_address = ftomn_candidates[0]
    original_ftomn = data[ftomn_address]
    ok(f"FTOMN found at 0x{ftomn_address:X}")
    info(f"Original FTOMN value: 0x{original_ftomn:02X}")

    code_cave = find_compatible_code_cave(data, memory_layout_kb)
    if code_cave is None:
        if memory_layout_kb == 1024:
            raise LaunchError(
                "Cannot find a safe 256-byte code cave in the compatible "
                "0x70000-0xFFFFF range."
            )
        raise LaunchError("Cannot find a safe 256-byte code cave.")
    ok(f"Code cave found at 0x{code_cave:X}")

    config_address = find_hole(
        data,
        CONFIG_AREA_SIZE,
        start=CONFIG_SEARCH_START,
        end=min(CONFIG_SEARCH_END, len(data)),
    )
    if config_address is None:
        raise LaunchError(
            "Cannot find a safe configuration area between 0x17000 and 0x18000."
        )
    ok(f"Configuration area found at 0x{config_address:X}")

    hook_address = find_hook(bytes(data), memory_layout_kb)
    ok(f"Hook location found at 0x{hook_address:X}")

    original_hook = bytes(data[hook_address : hook_address + 4])
    if original_hook[0] == 0xDA:
        raise LaunchError(
            "A CALLS instruction already exists at the hook, but this utility's "
            "installation marker was not found. Use an original BIN."
        )

    code = build_launch_code(
        config_address=config_address,
        tsrldyn=tsrldyn,
        vfil_w=vfil_w,
        nmot_w=nmot_w,
        wped=wped,
        clutch_address=clutch_address,
        clutch_bit=clutch_bit,
        brake_address=brake_address,
        brake_bit=brake_bit,
        nls_counter_address=nls_counter_address,
        original_hook_bytes_2_3=original_hook[2:4],
    )

    data[code_cave : code_cave + len(code)] = code
    data[hook_address : hook_address + 4] = b"\xDA" + encode_calls_address(code_cave)
    data[ftomn_address] = 0x00

    defaults = default_configuration()
    normal_ignition_raw = encode_ignition_ms(DEFAULT_IGNITION_CUT_MS)
    write_configuration(data, config_address, defaults, normal_ignition_raw)
    write_metadata(
        data,
        config_address=config_address,
        code_cave_address=code_cave,
        hook_address=hook_address,
        ftomn_address=ftomn_address,
        original_ftomn=original_ftomn,
        original_hook_bytes=original_hook,
        soft_cut=defaults.soft_cut,
        normal_ignition_raw=normal_ignition_raw,
    )

    installation = detect_installation(bytes(data))
    if installation is None:
        raise LaunchError("Installation verification failed.")

    compatible = find_compatible_setzi_installation(
        bytes(data),
        memory_layout_kb,
    )
    if compatible is None:
        raise LaunchError(
            "Installation is valid for this utility but was not detected by "
            "the compatible Setzi scanner."
        )

    ok(f"Launch Control code installed ({len(code)} bytes)")
    ok("Compatible Setzi detection verified")
    ok("Default cut mode set to Hard Cut (FTOMN 0x00)")
    ok("Original FTOMN and hook bytes stored for restoration")
    ok("Installation verified")
    return installation


def remove_installation(data: bytearray, installation: Installation) -> None:
    info("Removing Launch Control.")

    data[
        installation.hook_address : installation.hook_address + 4
    ] = installation.original_hook_bytes
    data[installation.ftomn_address] = installation.original_ftomn

    code_end = min(installation.code_cave_address + CODE_CAVE_SIZE, len(data))
    data[installation.code_cave_address : code_end] = b"\xFF" * (
        code_end - installation.code_cave_address
    )

    config_end = min(installation.config_address + CONFIG_AREA_SIZE, len(data))
    data[installation.config_address : config_end] = b"\xFF" * (
        config_end - installation.config_address
    )

    ok(f"Original hook restored at 0x{installation.hook_address:X}")
    ok(
        f"Original FTOMN restored to 0x{installation.original_ftomn:02X} "
        f"at 0x{installation.ftomn_address:X}"
    )
    ok("Code cave and configuration area cleared")


def verify_before_save(original: bytes, modified: bytes) -> None:
    if len(original) != len(modified):
        raise LaunchError(
            f"File size changed from {len(original)} to {len(modified)} bytes."
        )
    ok("File size unchanged")


def save_output(input_path: Path, original: bytes, modified: bytearray, suffix: str) -> Path:
    verify_before_save(original, bytes(modified))
    target = output_name(input_path, suffix)
    try:
        target.write_bytes(modified)
    except OSError as exc:
        raise LaunchError(f"Cannot write output file: {exc}") from exc

    reread = target.read_bytes()
    if reread != bytes(modified):
        raise LaunchError("Output verification failed after writing the file.")

    ok("Output file verified")
    ok(f"Saved: {target}")
    return target


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        prog="launch.exe",
        description="Install and configure ME7 Launch Control.",
        epilog=(
            "Examples:\n"
            "  launch.exe test.bin test.ecu\n"
            "  launch.exe --remove test_mod.bin test.ecu"
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--remove", action="store_true", help="Remove this utility's LC patch.")
    parser.add_argument(
        "--nls-counter",
        default=f"0x{DEFAULT_NLS_COUNTER:X}",
        help=f"NLS counter RAM address (default: 0x{DEFAULT_NLS_COUNTER:X}).",
    )
    parser.add_argument("bin_file", type=Path, help="Input ECU BIN file.")
    parser.add_argument("ecu_file", type=Path, help="ME7Info ECU definition file.")
    return parser.parse_args()


def main() -> int:
    args = parse_arguments()
    banner()

    if not args.bin_file.is_file():
        raise LaunchError(f"BIN file not found: {args.bin_file}")
    if not args.ecu_file.is_file():
        raise LaunchError(f"ECU file not found: {args.ecu_file}")

    original = args.bin_file.read_bytes()
    if len(original) not in SUPPORTED_BIN_SIZES:
        raise LaunchError(
            "Unsupported BIN size. Expected 512 KB or 1024 KB, "
            f"received {len(original) / 1024:.0f} KB."
        )

    memory_layout_kb = len(original) // 1024
    data = bytearray(original)

    print(f"Input BIN : {args.bin_file}")
    print(f"Input ECU : {args.ecu_file}")
    print()
    ok(f"BIN loaded ({memory_layout_kb} KB)")
    ok("Memory layout: " + ("29F800" if memory_layout_kb == 1024 else "29F400"))

    installation = detect_installation(original)

    if installation is None:
        info("Native ME7LC001 installation marker not found.")
        info("Searching for a compatible Setzi Launch Control patch.")
        installation = find_compatible_setzi_installation(
            original,
            memory_layout_kb,
        )
        if installation is not None:
            ok("Compatible Launch Control detected")
            ok(f"Launch function found at 0x{installation.code_cave_address:X}")
            ok(f"Configuration area found at 0x{installation.config_address:X}")
            ok(f"Hook found at 0x{installation.hook_address:X}")
            ok(f"FTOMN found at 0x{installation.ftomn_address:X}")
            info("Legacy-compatible installation mode is active.")

    if (
        installation is not None
        and installation.managed
        and not args.remove
    ):
        installation = migrate_native_installation_to_setzi_visible_area(
            data,
            installation,
            memory_layout_kb,
        )

    if args.remove:
        if installation is None:
            raise LaunchError("Launch Control was not detected.")
        if not installation.managed:
            raise LaunchError(
                "Removal is unavailable for a compatible legacy installation "
                "because the original hook bytes and original FTOMN were not "
                "stored by this utility."
            )

        ok("Native Launch Control installation detected")
        remove_installation(data, installation)
        target = save_output(args.bin_file, original, data, "_lc_removed")
        print()
        warn("Checksums are not calculated. Correct and verify checksums before flashing.")
        print(f"Output file: {target}")
        return 0

    entries = parse_ecu_file(args.ecu_file)
    ok("ECU file parsed")

    if installation is None:
        installation = install(
            data,
            entries,
            memory_layout_kb,
            parse_int(args.nls_counter),
        )
    else:
        if installation.managed:
            ok("Native Launch Control installation detected")
        else:
            ok("Compatible external Launch Control installation detected")
        info("Installation skipped.")

        current_ftomn = data[installation.ftomn_address]
        mode = (
            "Soft Cut"
            if current_ftomn == FTOMN_SOFT_CUT
            else "Hard Cut"
            if current_ftomn == FTOMN_HARD_CUT
            else "Custom"
        )
        info(f"Current FTOMN value: 0x{current_ftomn:02X} ({mode})")

    info("Entering configuration mode.")

    clutch_address = ecu_address(entries, "b_kuppl")
    clutch_bit = ecu_mask(entries, "b_kuppl")
    brake_address = ecu_address(entries, "b_br")
    brake_bit = ecu_mask(entries, "b_br")

    if clutch_address is None or clutch_bit is None:
        raise LaunchError(
            "Required ECU bit variable not found or invalid: b_kuppl"
        )
    if brake_address is None or brake_bit is None:
        raise LaunchError(
            "Required ECU bit variable not found or invalid: b_br"
        )

    current_trigger = detect_launch_trigger(
        data,
        installation.code_cave_address,
        clutch_address,
        clutch_bit,
        brake_address,
        brake_bit,
    )
    ok(f"Current Activation Trigger: {current_trigger}")

    current_ftomn = data[installation.ftomn_address]

    if installation.managed:
        current = read_configuration(data, installation.config_address)

        # For native installations, the stored flag remains the primary source.
        expected_ftomn = (
            FTOMN_SOFT_CUT if current.soft_cut else FTOMN_HARD_CUT
        )
        if current_ftomn != expected_ftomn:
            warn(
                f"Stored Cut Mode and FTOMN do not match: "
                f"stored mode is {'Soft Cut' if current.soft_cut else 'Hard Cut'}, "
                f"but FTOMN is 0x{current_ftomn:02X}."
            )
            warn(
                f"FTOMN will be synchronized to 0x{expected_ftomn:02X} "
                "when the configuration is saved."
            )

        normal_ignition_raw = installation.normal_ignition_raw
        if normal_ignition_raw in (0, 0xFFFF):
            normal_ignition_raw = encode_ignition_ms(DEFAULT_IGNITION_CUT_MS)
    else:
        current = read_legacy_configuration(
            data,
            installation.config_address,
            current_ftomn,
        )
        normal_ignition_raw = encode_ignition_ms(current.ignition_cut_ms)

    current.trigger = current_trigger

    normal_ignition_ms = decode_ignition_ms(normal_ignition_raw)
    configured = configure(current, normal_ignition_ms)

    write_launch_trigger(
        data,
        installation.code_cave_address,
        configured.trigger,
        clutch_address,
        clutch_bit,
        brake_address,
        brake_bit,
    )
    ok(f"Activation Trigger set to {configured.trigger}")

    selected_ftomn = (
        FTOMN_SOFT_CUT if configured.soft_cut else FTOMN_HARD_CUT
    )
    data[installation.ftomn_address] = selected_ftomn
    ok(
        f"FTOMN set to 0x{selected_ftomn:02X} "
        f"({'Soft Cut' if configured.soft_cut else 'Hard Cut'})"
    )

    if installation.managed:
        normal_ignition_raw = encode_ignition_ms(configured.ignition_cut_ms)
        write_configuration(
            data,
            installation.config_address,
            configured,
            normal_ignition_raw,
        )
        write_metadata(
            data,
            config_address=installation.config_address,
            code_cave_address=installation.code_cave_address,
            hook_address=installation.hook_address,
            ftomn_address=installation.ftomn_address,
            original_ftomn=installation.original_ftomn,
            original_hook_bytes=installation.original_hook_bytes,
            soft_cut=configured.soft_cut,
            normal_ignition_raw=normal_ignition_raw,
        )

        verified_installation = detect_installation(bytes(data))
        if verified_installation is None:
            raise LaunchError("Final native Launch Control verification failed.")

        verified_configuration = read_configuration(
            data,
            verified_installation.config_address,
        )
    else:
        write_legacy_configuration(
            data,
            installation.config_address,
            configured,
        )

        verified_installation = find_compatible_setzi_installation(
            bytes(data),
            memory_layout_kb,
        )
        if verified_installation is None:
            raise LaunchError(
                "Final compatible Launch Control verification failed."
            )
        if verified_installation.config_address != installation.config_address:
            raise LaunchError(
                "Compatible Launch configuration address changed unexpectedly."
            )

        verified_configuration = read_legacy_configuration(
            data,
            verified_installation.config_address,
            data[verified_installation.ftomn_address],
        )

    verified_trigger = detect_launch_trigger(
        data,
        verified_installation.code_cave_address,
        clutch_address,
        clutch_bit,
        brake_address,
        brake_bit,
    )
    if verified_trigger != configured.trigger:
        raise LaunchError(
            f"Activation Trigger verification failed: expected "
            f"{configured.trigger}, detected {verified_trigger}."
        )

    verified_ftomn = data[verified_installation.ftomn_address]
    expected_ftomn = (
        FTOMN_SOFT_CUT
        if verified_configuration.soft_cut
        else FTOMN_HARD_CUT
    )

    if verified_ftomn != expected_ftomn:
        raise LaunchError(
            f"Cut Mode verification failed: expected FTOMN "
            f"0x{expected_ftomn:02X}, but found 0x{verified_ftomn:02X}."
        )

    ok("Configuration written")
    ok(f"Activation Trigger verified: {verified_trigger}")
    ok(
        f"Cut Mode verified: "
        f"{'Soft Cut' if verified_configuration.soft_cut else 'Hard Cut'} "
        f"(FTOMN 0x{verified_ftomn:02X})"
    )
    ok(
        "Launch Control verification passed "
        f"({'native' if installation.managed else 'compatible legacy'} mode)"
    )

    target = save_output(args.bin_file, original, data, "_mod")
    print()
    warn("Checksums are not calculated. Correct and verify checksums before flashing.")
    if not installation.managed:
        warn(
            "Legacy-compatible patch configuration was updated without writing "
            "ME7LC001 metadata."
        )
    print(f"Output file: {target}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print()
        fail("Operation cancelled by the user.")
        raise SystemExit(130)
    except (LaunchError, OSError, ValueError) as exc:
        fail(str(exc))
        raise SystemExit(1)

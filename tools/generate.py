"""
Rebuilds everything derived from the configuration, so none of it can drift from the firmware:

    can/bms_can1.dbc, can/ivt_can2.dbc, can/charger_can3.dbc     CAN databases
    can/*_signals.csv                                            one row per signal, in the team's CAN sheet columns
    doc/bringup_test_procedure.md                        from doc/bringup_test_procedure.md.in
    README.md                                            the "BEGIN GENERATED" sections
    GUI/bms_model.json                                   pack layout, limits, SoC model and settings for the GUI

Sources: Core/Inc/bms_config.h, plus the charger IDs in bms_can.h and the fault flags in main.h.
It also checks that the CubeMX settings (g474-bms.ioc and the code generated from it) give the bus
speeds and timer tick in bms_config.h; those files belong to CubeMX, so they are checked, not written.

Runs automatically on every STM32CubeIDE build (pre-build step), every git commit (enable once per
clone: git config core.hooksPath tools/githooks) and every test/run_tests.sh. Only needs Python 3.

    python tools/generate.py               write what changed
    python tools/generate.py --check       write nothing; exit 1 if anything is out of date
    python tools/generate.py --pre-commit  write, and stage what changed (the git hook)
    python tools/generate.py --watch       regenerate whenever a source changes (Ctrl+C to stop)
"""

import argparse
import json
import os
import re
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import gen_dbc                                                      # noqa: E402
from gen_dbc import ROOT, CONFIG, read_config, read_pins, read_charger_ids, bitrate_text   # noqa: E402
from gen_dbc import settings_table, SETTINGS_COMMANDS, SETTINGS_RESULTS                    # noqa: E402

DOC_TEMPLATE = "doc/bringup_test_procedure.md.in"
DOC = "doc/bringup_test_procedure.md"
README = "README.md"
GUI_MODEL = "can/bms_model.json"
IOC = "g474-bms.ioc"
FDCAN_C = "Core/Src/fdcan.c"
TIM_C = "Core/Src/tim.c"

# Watched by --watch (README.md is an output too, so it isn't watched)
SOURCES = ["Core/Inc/bms_config.h", "Core/Inc/bms_can.h", "Core/Inc/main.h", DOC_TEMPLATE, IOC, FDCAN_C, TIM_C,
           "tools/gen_dbc.py", "tools/generate.py"]

FRAME_BITS_NOMINAL = 131        # Extended 8-byte frame without stuff bits, including the interframe space

# NUCLEO-G474RE (MB1367 schematic, sheet 5) Morpho header pins used on the bench. Net labels sit just above
# their wire: CN10-10 is not connected, CN10-12 PA12, CN10-14 PA11, CN10-16 PB12 (checked on a bench rig).
NUCLEO_MORPHO = {
    "PC9": "CN10-1", "PC8": "CN10-2", "PC6": "CN10-4", "PA12": "CN10-12", "PA11": "CN10-14", "PB12": "CN10-16",
    "PB13": "CN10-30", "PA8": "CN10-23", "PA15": "CN7-17", "PC2": "CN7-35", "PC1": "CN7-36", "PC3": "CN7-37", "PC0": "CN7-38", "3V3": "CN7-16",
    "GND": "CN7-20",
}

# EVAL-ADBMS6830BMSW (rev D schematic): each GPIO goes through 1 k to header J5, with a 10 k pull-up to
# VREF2 (3.0 V) on the header side; GPIO4/GPIO5 are the EEPROM's I2C lines with 10 k to VREG (4.5-5.5 V)
# on the chip side instead
EVAL_PULLUP_OHM = 10e3
EVAL_SERIES_OHM = 1e3
EVAL_VREF2 = 3.0
EVAL_VREG = (4.5, 5.0, 5.5)
EVAL_I2C_GPIOS = (4, 5)
EVAL_BALANCE_OHM = 30.0
E24 = [1.0, 1.1, 1.2, 1.3, 1.5, 1.6, 1.8, 2.0, 2.2, 2.4, 2.7, 3.0, 3.3, 3.6, 3.9, 4.3, 4.7, 5.1, 5.6, 6.2, 6.8,
       7.5, 8.2, 9.1]
E24_OHMS = [round(m * d) for d in (100, 1000, 10000, 100000) for m in E24]

PIN_NOTES = {
    "FAULT":       ("AMS fault output to the latch", "CFG_FAULT_ACTIVE_HIGH", "fault"),
    "TSAC_READY":  ("TSAC ready relay", "CFG_TSAC_READY_ACTIVE_HIGH", "on"),
    "PRECHARGE":   ("Precharge relay", "CFG_PRECHARGE_ACTIVE_HIGH", "closed"),
    "BYPASS":      ("Bypass (AIR)", "CFG_BYPASS_ACTIVE_HIGH", "closed"),
    "SDC_IN":      ("Shutdown circuit sense", "CFG_SDC_IN_ACTIVE_HIGH", "SDC closed"),
    "CHARGER_BTN": ("Charger button", "CFG_CHARGER_BTN_ACTIVE_LOW", "pressed"),
    "ISOSPI_CS":   ("isoSPI chip select (ADBMS6822 port A)", None, None),
    "ISOSPI_CS2":  ("isoSPI chip select port B, unused, held high", None, None),
    "ISOSPI_MSTR": ("ADBMS6822 MSTR, held high", None, None),
    "ISOSPI_MSTR2": ("ADBMS6822 MSTR2", None, None),
}


class GenerateError(Exception):
    pass


def path(rel):
    return os.path.join(ROOT, *rel.split("/"))


def read_text(rel):
    with open(path(rel), encoding="utf-8") as f:
        return f.read().replace("\r\n", "\n")


def read_array(name, text):
    expr=gen_dbc.expand(name).strip().strip('{}')
    return [float(re.sub(r'(?<=\d)f\b','',v.strip())) for v in expr.split(',') if v.strip()]


# ------------------------------------------------------------------------------------------------
# Values derived from the configuration
# ------------------------------------------------------------------------------------------------

def ivt_commands(cfg):
    """The configuration the BMS sends the IVT-S at start-up (bms_ivt.c configureIvt / resultConfig).
    test/check_dbc.py compares this with what the simulated firmware actually sends."""
    def cmd(*b):
        return list(b) + [0] * (8 - len(b))
    results = [
        (0, 2, cfg["CFG_IVT_CURRENT_PERIOD_MS"], "I"),
        (1, 2, cfg["CFG_IVT_U1_PERIOD_MS"], "U1"),
        (2, 2, cfg["CFG_IVT_U2_PERIOD_MS"], "U2"),
        (3, 0, 60, "U3 off"),                           # Time 0 would make the IVT ignore the command
        (4, 0, 100, "T off"),
        (5, 0, 30, "W off"),
        (6, 0, 30, "As off"),
        (7, 2, cfg["CFG_IVT_ENERGY_PERIOD_MS"], "Wh"),
    ]
    out = [(cmd(0x34, 0x00, 0x01), "stop")]
    for mux, mode, period, name in results:
        note = f"{name} every {period} ms" if mode else name
        out.append((cmd(0x20 | mux, mode, period >> 8, period & 0xFF), note))
    out.append((cmd(0x34, 0x01, 0x01), "run"))
    return out


def ranges(numbers):
    """[1, 2, 3, 6, 7, 8] -> 1-3, 6-8"""
    out, start = [], None
    for i, n in enumerate(numbers):
        if start is None:
            start = n
        if i + 1 == len(numbers) or numbers[i + 1] != n + 1:
            out.append(f"{start}" if start == n else f"{start}-{n}")
            start = None
    return ", ".join(out)


def ohms(r):
    return f"{r / 1000:g} kΩ" if r >= 1000 else f"{r:g} Ω"


def derived(cfg):
    text = read_text("Core/Inc/bms_config.h")
    table_c = read_array("CFG_TEMP_TABLE_C", text)
    table_v = read_array("CFG_TEMP_TABLE_V", text)
    temp_gpios = [int(g) for g in read_array("CFG_SLAVE_TEMP_GPIOS", text)]

    def v_to_temp(v):
        """Temperature the firmware reads for a sensor voltage, None outside the table (sensor fault)"""
        if v > table_v[0] or v < table_v[-1]:
            return None
        for i in range(len(table_v) - 1):
            if table_v[i + 1] <= v <= table_v[i]:
                f = (table_v[i] - v) / (table_v[i] - table_v[i + 1])
                return table_c[i] + f * (table_c[i + 1] - table_c[i])
        return None

    def eval_v(gpio, r, vreg=EVAL_VREG[1]):
        """GPIO voltage on the eval board with r from the J5 pin to V-"""
        if gpio in EVAL_I2C_GPIOS:
            return vreg * (r + EVAL_SERIES_OHM) / (r + EVAL_SERIES_OHM + EVAL_PULLUP_OHM)
        return EVAL_VREF2 * r / (r + EVAL_PULLUP_OHM)

    def pick(gpio, want):
        """E24 resistor whose temperature best satisfies want(t) -> score (lower is better, None = unusable)"""
        best = None
        for r in E24_OHMS:
            t = v_to_temp(eval_v(gpio, r))
            score = want(t) if t is not None else None
            if score is not None and (best is None or score < best[0]):
                best = (score, r, t)
        return best

    tmax, tmin = cfg["CFG_CELL_TEMP_MAX_C"], cfg["CFG_CELL_TEMP_MIN_C"]
    temp_rows = ["| J5 pins | Bias on the eval board | 25 °C | Over-temperature test | Under-temperature test |",
                 "|---|---|---|---|---|"]
    plain = [g for g in temp_gpios if g not in EVAL_I2C_GPIOS]
    i2c = [g for g in temp_gpios if g in EVAL_I2C_GPIOS]
    if plain:
        g = plain[0]
        nominal = pick(g, lambda t: abs(t - 25))
        # Fault tests: at least 3 C past the limit, so resistor and VREF2 tolerances can't hide the fault
        hot = pick(g, lambda t: t - tmax if t >= tmax + 3 else None)
        cold = pick(g, lambda t: tmin - t if t <= tmin - 3 else None)
        temp_rows.append(f"| GPIO{ranges(plain)} | 10 kΩ pull-up to VREF2 ({EVAL_VREF2:g} V) | "
                         f"{ohms(nominal[1])} ({nominal[2]:.0f} °C) | {ohms(hot[1])} ({hot[2]:.0f} °C) | "
                         f"{ohms(cold[1])} ({cold[2]:.0f} °C) |")
    if i2c:
        g = i2c[0]
        nominal = pick(g, lambda t: abs(t - 25))
        temps = [v_to_temp(eval_v(g, nominal[1], vreg)) for vreg in (EVAL_VREG[0], EVAL_VREG[2])]
        spread = (f"{min(temps):.0f} to {max(temps):.0f} °C" if None not in temps
                  else "may read as a sensor fault")
        temp_rows.append(f"| GPIO{ranges(i2c)} | EEPROM I²C: 10 kΩ pull-up to VREG ({EVAL_VREG[0]:g}-{EVAL_VREG[2]:g} V) | "
                         f"{ohms(nominal[1])} ({spread} over the VREG range) | use another GPIO | use another GPIO |")
    bench_temp_table = "\n".join(temp_rows)

    pins = read_pins()
    ioc = dict(line.split("=", 1) for line in read_text(IOC).splitlines() if "=" in line and not line.startswith("#"))
    ioc_pins = {v: k.split(".")[0] for k, v in ioc.items() if k.endswith(".Signal")}

    def where(pin):
        return f"{pin} ({NUCLEO_MORPHO[pin]})" if pin in NUCLEO_MORPHO else pin

    can_rows = ["| Bus | MCU TX → transceiver TXD | MCU RX ← transceiver RXD | Bit rate | Other end |", "|---|---|---|---|---|"]
    for n, other in ((1, "USB-CAN adapter"), (2, "IVT-S (120 Ω at the IVT)"), (3, "Charger, or the adapter")):
        can_rows.append(f"| CAN{n} | {where(ioc_pins.get(f'FDCAN{n}_TX', '?'))} | {where(ioc_pins.get(f'FDCAN{n}_RX', '?'))} | "
                        f"{bitrate_text(cfg[f'CFG_CAN{n}_BITRATE'])} | {other} |")
    bench_can_table = "\n".join(can_rows)

    def output_hint(active_high):
        return "LED + 1 kΩ to GND (lit = on)" if active_high else "LED + 1 kΩ to 3V3 (lit = on)"

    io_rows = ["| Signal | Pin | Bench connection |", "|---|---|---|"]
    io_rows.append(f"| Fault output (open drain) | {where(pins['FAULT'])} | 10 kΩ pull-up to 3V3, scope here: "
                   f"{'high' if cfg['CFG_FAULT_ACTIVE_HIGH'] else 'low'} = fault |")
    for name, label, pol in (("TSAC_READY", "TSAC ready relay", "CFG_TSAC_READY_ACTIVE_HIGH"),
                             ("PRECHARGE", "Precharge relay", "CFG_PRECHARGE_ACTIVE_HIGH"),
                             ("BYPASS", "Bypass (AIR)", "CFG_BYPASS_ACTIVE_HIGH")):
        io_rows.append(f"| {label} | {where(pins[name])} | {output_hint(cfg[pol])} |")
    io_rows.append(f"| SDC sense | {where(pins['SDC_IN'])} | Jumper to {'3V3' if cfg['CFG_SDC_IN_ACTIVE_HIGH'] else 'GND'} "
                   "= SDC closed. The input has no pull resistor: don't leave it open |")
    io_rows.append(f"| Charger button | {where(pins['CHARGER_BTN'])} | Open, or a button to "
                   f"{'GND' if cfg['CFG_CHARGER_BTN_ACTIVE_LOW'] else '3V3'} |")
    io_rows.append(f"| Transceiver supply | 3V3 {NUCLEO_MORPHO['3V3']}, GND {NUCLEO_MORPHO['GND']} | Both transceiver boards |")
    bench_io_table = "\n".join(io_rows)

    def temp_v(celsius):
        """Sensor voltage at a temperature (the firmware's table, interpolated)"""
        if celsius <= table_c[0]:
            return table_v[0]
        for i in range(len(table_c) - 1):
            if celsius <= table_c[i + 1]:
                f = (celsius - table_c[i]) / (table_c[i + 1] - table_c[i])
                return table_v[i] + f * (table_v[i + 1] - table_v[i])
        return table_v[-1]

    burst = cfg["CFG_CAN_BURST_FRAMES"]
    fast = cfg["CFG_CAN_FAST_FRAMES"]
    fps = (burst * 1000 / cfg["CFG_CAN_TX_PERIOD_MS"] + fast * 1000 / cfg["CFG_CAN_FAST_PERIOD_MS"] +
           (cfg["CFG_INVERTER_LIMITS_ENABLED"] * 2 + 1) * 1000 / cfg["CFG_INVERTER_LIMIT_PERIOD_MS"])
    bitrate1 = cfg["CFG_CAN1_BITRATE"]
    ivt_periods = [cfg[k] for k in ("CFG_IVT_CURRENT_PERIOD_MS", "CFG_IVT_U1_PERIOD_MS", "CFG_IVT_U2_PERIOD_MS",
                                    "CFG_IVT_ENERGY_PERIOD_MS")]
    pins = read_pins()

    temp_points = [(table_c[-1], ""), (cfg["CFG_CELL_TEMP_MAX_C"], " (max)"),
                   (cfg["CFG_CELL_CHARGE_TEMP_MAX_C"], " (max charging)"), (25, ""),
                   (cfg["CFG_CELL_CHARGE_TEMP_MIN_C"], " (min charging)"), (cfg["CFG_CELL_TEMP_MIN_C"], " (min)"),
                   (table_c[0], "")]
    temp_table = ("| Temperature | " + " | ".join(f"{t:g} °C{note}" for t, note in temp_points) + " |\n"
                  "|---|" + "---|" * len(temp_points) + "\n"
                  "| Sensor voltage | " + " | ".join(f"{temp_v(t):.2f} V" for t, _ in temp_points) + " |")

    ts = cfg["CFG_CAN_ID_TS_REQUEST"]
    ts_frame = f"ID 0x{ts:X} ({'extended' if cfg['CFG_CAN_TS_REQUEST_EXTENDED'] else 'standard'}), data `01`"

    return {
        "temp_v": temp_v,
        "v_to_temp": v_to_temp,
        "bench_temp_table": bench_temp_table,
        "bench_can_table": bench_can_table,
        "bench_io_table": bench_io_table,
        "bench_pack_v_min": cfg["CFG_CELLS_PER_SLAVE"] * cfg["CFG_CELL_UV_V"],
        "bench_pack_v_max": cfg["CFG_CELLS_PER_SLAVE"] * cfg["CFG_CELL_OV_V"],
        "eval_balance_ohm": EVAL_BALANCE_OHM,
        "ts_request_frame": ts_frame,
        "temp_table": temp_table,
        "temp_table_v_min": min(table_v),
        "temp_table_v_max": max(table_v),
        "pin": pins,
        "bitrate": bitrate_text,
        "can1_bitrate": bitrate_text(bitrate1),
        "can2_bitrate": bitrate_text(cfg["CFG_CAN2_BITRATE"]),
        "can3_bitrate": bitrate_text(cfg["CFG_CAN3_BITRATE"]),
        "can_pins": {n: (ioc_pins.get(f"FDCAN{n}_TX", "?"), ioc_pins.get(f"FDCAN{n}_RX", "?")) for n in (1, 2, 3)},
        "burst_frames": burst,
        "burst_hz": 1000 / cfg["CFG_CAN_TX_PERIOD_MS"],
        "fast_hz": 1000 / cfg["CFG_CAN_FAST_PERIOD_MS"],
        "can1_frames_per_s": fps,
        "can1_load_min": fps * FRAME_BITS_NOMINAL / bitrate1 * 100,
        "can1_load_max": fps * cfg["CFG_CAN_FRAME_BITS_MAX"] / bitrate1 * 100,
        "burst_ms_min": burst * FRAME_BITS_NOMINAL / bitrate1 * 1000,
        "burst_ms_max": burst * cfg["CFG_CAN_FRAME_BITS_MAX"] / bitrate1 * 1000,
        "ivt_msgs_per_s": sum(1000 / p for p in ivt_periods),
        "ivt_commands": " ".join(f"`{' '.join(f'{b:02X}' for b in c[:4])}` ({note})," for c, note in ivt_commands(cfg)).rstrip(","),
        "ivt_command_count": len(ivt_commands(cfg)),
        "power_max": 32767 * cfg["CFG_CAN_POWER_W_PER_BIT"],
        "worst_reaction_ms": cfg["CFG_FAULT_DEADLINE_MS"] + cfg["CFG_SAFETY_TICK_MS"],
        "charger": read_charger_ids(),
    }


# ------------------------------------------------------------------------------------------------
# README sections
# ------------------------------------------------------------------------------------------------

def readme_can(cfg, d):
    slaves = cfg["CFG_NUM_SLAVES"]
    vf, tf = cfg["CFG_CAN_V_FRAMES_PER_SLAVE"], cfg["CFG_CAN_T_FRAMES_PER_SLAVE"]
    vb, tb, sb = cfg["CFG_CAN_ID_CELL_V_BASE"], cfg["CFG_CAN_ID_CELL_T_BASE"], cfg["CFG_CAN_ID_SLAVE_BASE"]
    burst = f"{d['burst_hz']:g} Hz"
    fast = f"{d['fast_hz']:g} Hz"
    lsb = cfg["CFG_CAN_POWER_W_PER_BIT"]
    power = f"int16 power {lsb:g} W (+ = discharge, saturates at ±{d['power_max']:g} W, -32768 = not available)"
    ts_type = "extended" if cfg["CFG_CAN_TS_REQUEST_EXTENDED"] else "standard"
    rows = [
        (f"`0x{vb:X}`-`0x{vb + slaves * vf - 1:X}` (`0x{vb:X} + slave*{vf} + n`)", burst,
         "4 x int16 cell voltage mV, cells `4n+1`..`4n+4` of the slave"),
        (f"`0x{tb:X}`-`0x{tb + slaves * tf - 1:X}` (`0x{tb:X} + slave*{tf} + n`)", burst,
         f"4 x int16 cell temperature 0.1 C, sensors `4n+1`..`4n+4` ({cfg['CFG_TEMP_INVALID_C']:g} C = sensor fault)"),
        (f"`0x{sb:X}`-`0x{sb + slaves - 1:X}` (`0x{sb:X} + slave`)", burst,
         "int16 slave voltage 0.01 V, int16 die temp 0.1 C, byte 4 balancing bitmask, byte 5 cell fault bitmask, "
         "byte 6 open wire bitmask (bit0 = cell 1), byte 7: bit0 comms error, bit1 slave fault"),
        (f"`0x{cfg['CFG_CAN_ID_PACK']:X}`", burst,
         "int16 IVT U1 0.1 V, int16 current 0.1 A (+ = discharge), int16 sum of cells 0.1 V, "
         "byte 6: bit0 current timeout, bit1 U1 timeout, bit2 IVT error"),
        (f"`0x{cfg['CFG_CAN_ID_STATUS']:X}`", burst,
         "byte 0 active fault flags (`BMS_StatusTypeDef`), byte 1: bit0 fault output, bit1 TSAC ready, "
         "bit2 balancing enabled, bit3 charging, bit4 SDC closed, bit5 bench build (`bms_config_local.h`), "
         "bit6 inverter limits derated for temperature, bit7 inverter discharge at the power limit, "
         "int16 measurement age ms, int16 loop time ms, "
         "byte 6 precharge state (0 idle, 1 precharging, 2 closing, 3 TS active, 4 failed), byte 7 U2/U1 %"),
        (f"`0x{cfg['CFG_CAN_ID_SUMMARY']:X}`", burst,
         "int16 min cell mV, int16 max cell mV, int16 min temp 0.1 C, int16 max temp 0.1 C"),
        (f"`0x{cfg['CFG_CAN_ID_ENERGY']:X}`", burst,
         f"{power}, uint16 remaining energy Wh (0xFFFF = not estimated yet), uint16 SoC 0.1 % "
         "(0xFFFF = not estimated yet), int16 energy used since start-up Wh"),
        (f"`0x{cfg['CFG_CAN_ID_FAST_1']:X}`", fast,
         "int32 IVT current mA (+ = discharge), int32 IVT U1 (pack) mV. INT32_MIN = not available"),
        (f"`0x{cfg['CFG_CAN_ID_FAST_2']:X}`", fast,
         f"int32 IVT U2 (vehicle side) mV (INT32_MIN = not available), {power}, "
         "uint16 remaining energy Wh (0xFFFF = unknown)"),
        (f"`0x{cfg['CFG_CAN_ID_LIMITS']:X}`", f"{1000 / cfg['CFG_INVERTER_LIMIT_PERIOD_MS']:g} Hz",
         f"int16 discharge power limit {lsb:g} W (current limit x pack voltage), int16 discharge current limit 0.1 A, "
         "int16 regen current limit 0.1 A, byte 6: bit0 discharge derated for temperature, bit1 held to the power "
         "limit, bit2 regen derated for temperature, bit3 AMS fault (0 A), bit4 window limiter boosting, "
         "bit5 runtime settings differ from the built-in limits, byte 7 500 ms average discharge power 0.5 kW"),
        (f"`0x{cfg['CFG_CAN_ID_CONFIG_RESPONSE']:X}`", "on request (at most one per fast period)",
         "Settings response: byte 0 command, byte 1 setting, byte 2 result (" +
         ", ".join(f"{k} {v.replace('_', ' ').lower().replace('ts ', 'TS ')}" for k, v in SETTINGS_RESULTS.items()) +
         "), byte 3: bit0 unsaved, bit1 differs from the built-in values, bit2 saved settings in flash, "
         "int32 value in force"),
        (f"`0x{cfg['CFG_CAN_ID_INVERTER_DC_LIMIT']:04X}`", f"{1000 / cfg['CFG_INVERTER_LIMIT_PERIOD_MS']:g} Hz",
         "To the inverter: DTI \"Set max. DC current\" (0x0A), int16 big-endian A x 10, bytes 2-7 0xFF"),
        (f"`0x{cfg['CFG_CAN_ID_INVERTER_DC_BRAKE_LIMIT']:04X}`", f"{1000 / cfg['CFG_INVERTER_LIMIT_PERIOD_MS']:g} Hz",
         "To the inverter: DTI \"Set max. DC brake current\" (0x0B), negative, same format"),
        (f"`0x{cfg['CFG_CAN_ID_TS_REQUEST']:X}` (TBD, {ts_type}, received)", "from the VCU",
         "TS request: byte 0 = 1 activate, 0 deactivate"),
        (f"`0x{cfg['CFG_CAN_ID_CONFIG_REQUEST']:X}` (extended, received)", "from the GUI",
         "Settings request: byte 0 command (" +
         ", ".join(f"{k} {v.replace('_', ' ').lower()}" for k, v in SETTINGS_COMMANDS.items()) +
         "), byte 1 setting (0 for save, defaults, read all), bytes 4-7 int32 value (write)"),
    ]
    p = cfg["CFG_IVT_CURRENT_PERIOD_MS"]
    lines = [
        "| Bus | Speed | Use |",
        "|---|---|---|",
        f"| CAN1 | {d['can1_bitrate']} | Car / datalogger bus, shared with the inverter (DTI HV500): BMS data out, TS request and "
        "settings (GUI) in |",
        f"| CAN2 | {d['can2_bitrate']} | Inside the TSAC: the IVT-S |",
        f"| CAN3 | {d['can3_bitrate']} | Charger |",
        "",
        f"CAN1, **{d['can1_bitrate']}** (every node on the car bus must match, the HV500 included), 29-bit IDs, "
        "8 bytes, little-endian. IDs 0x0000-"
        f"0x{cfg['CFG_CAN1_INVERTER_ID_MAX']:04X} are the inverter's (DTI protocol: packet ID << 8 | node ID); the "
        f"BMS uses 0x{cfg['CFG_CAN_ID_CELL_V_BASE']:X} and up, so inverter frames win arbitration.",
        "",
        f"HV500 setup in the DTI CAN Tool (DTI Inverter CAN Manual v2.4): car bus on the inverter's CAN2 port, "
        f"CAN2 speed {d['can1_bitrate']}, extended IDs (in standard-ID mode every BMS frame would win arbitration "
        "over the inverter), CAN2 map version V24, Send CAN2 Status enabled, Drive enable via CAN2 on (or it "
        f"ignores the limits). Node ID: `CFG_INVERTER_NODE_ID` = 0x{cfg['CFG_INVERTER_NODE_ID']:02X}"
        f"{' (DTI broadcast)' if cfg['CFG_INVERTER_NODE_ID'] == 0xFF else ''}.",
        "",
        f"Inverter current limits, every {cfg['CFG_INVERTER_LIMIT_PERIOD_MS']} ms: discharge = min("
        f"{cfg['CFG_MAX_DISCHARGE_CURRENT_A']:g} A, {cfg['CFG_INVERTER_POWER_LIMIT_W'] / 1000:g} kW / IVT U1), so it "
        f"falls above {cfg['CFG_INVERTER_POWER_LIMIT_W'] / cfg['CFG_MAX_DISCHARGE_CURRENT_A']:g} V, and falls linearly "
        f"to {cfg['CFG_INVERTER_DERATE_CURRENT_A']:g} A as the hottest cell goes from "
        f"{cfg['CFG_INVERTER_DERATE_START_C']:g} to {cfg['CFG_INVERTER_DERATE_END_C']:g} °C. Regen = "
        f"{cfg['CFG_INVERTER_REGEN_CURRENT_A']:g} A (no power limit, EV 2.2.3), falling linearly to "
        f"{cfg['CFG_INVERTER_REGEN_DERATE_CURRENT_A']:g} A from {cfg['CFG_INVERTER_REGEN_DERATE_START_C']:g} to "
        f"{cfg['CFG_INVERTER_REGEN_DERATE_END_C']:g} °C (the AMS charge trip stays at "
        f"{cfg['CFG_MAX_CHARGE_CURRENT_A']:g} A). Both 0 A while the AMS is faulted. `BMS_Limits` reports them live, "
        "with the discharge power limit (current limit x pack voltage) and the 500 ms average discharge power.",
        "",
        f"Moving-window power limiter: **{'on' if cfg['CFG_POWER_WINDOW_LIMITER_ENABLED'] else 'off'}** "
        "(`CFG_POWER_WINDOW_LIMITER_ENABLED`). FS 2027 D 10.4.1 judges power after a "
        f"{cfg['CFG_POWER_WINDOW_MS']} ms moving average, so with it on, power may go above "
        f"{cfg['CFG_POWER_WINDOW_LIMIT_W'] / 1000:g} kW (up to the current limit) while that average stays at or below "
        f"it, and after a boost never drops below {cfg['CFG_POWER_WINDOW_FLOOR_W'] / 1000:g} kW. Any "
        f"{cfg['CFG_POWER_WINDOW_MS']} ms still holds at most {cfg['CFG_POWER_WINDOW_LIMIT_W'] * cfg['CFG_POWER_WINDOW_MS'] / 1e6:g} kJ, "
        "so it only moves power earlier: it helps short bursts after lower power (corner exits), not sustained "
        "full throttle, and only when the current limit allows more than the power limit (high pack voltage). "
        "Power limits become current limits allowing for the pack resistance "
        f"({cfg['CFG_PACK_RESISTANCE_MIN_OHM']:g}-{cfg['CFG_PACK_RESISTANCE_MAX_OHM']:g} R, `CFG_PACK_RESISTANCE_*`): "
        "the power can't overshoot, but when a boost ends the first lower limit can undershoot the floor by a few kW "
        "for one update. Narrow the range once logged runs show the real resistance.",
        "",
        f"- Measurement burst every {cfg['CFG_CAN_TX_PERIOD_MS']} ms ({burst}): {d['burst_frames']} frames, pack, "
        f"status and summary first, {d['burst_ms_min']:.0f}-{d['burst_ms_max']:.0f} ms on the bus.",
        f"- Fast frames every {cfg['CFG_CAN_FAST_PERIOD_MS']} ms ({fast}): `BMS_Fast_1`, `BMS_Fast_2`. The burst "
        "never holds more than one TX buffer, so they wait for at most one frame.",
        f"- BMS share of CAN1: {d['can1_frames_per_s']:g} frames/s, {d['can1_load_min']:.0f}-{d['can1_load_max']:.0f} % "
        f"(typical to worst-case bit stuffing; compile-time limit {cfg['CFG_CAN1_MAX_LOAD_PERCENT']} %), plus up to "
        f"{d['fast_hz']:g} settings responses/s while the GUI reads or changes settings.",
        "",
        "| ID | Rate | Content |",
        "|---|---|---|",
    ]
    lines += [f"| {a} | {b} | {c} |" for a, b, c in rows]
    lines += [
        "",
        f"CAN2, {d['can2_bitrate']}: IVT-S (factory IDs): `0x{cfg['CFG_IVT_ID_RESULT_I']:X}` current, "
        f"`0x{cfg['CFG_IVT_ID_RESULT_U1']:X}` U1, `0x{cfg['CFG_IVT_ID_RESULT_U2']:X}` U2 every "
        f"{p if p == cfg['CFG_IVT_U1_PERIOD_MS'] == cfg['CFG_IVT_U2_PERIOD_MS'] else '/'.join(str(cfg[k]) for k in ('CFG_IVT_CURRENT_PERIOD_MS', 'CFG_IVT_U1_PERIOD_MS', 'CFG_IVT_U2_PERIOD_MS'))} ms, "
        f"`0x{cfg['CFG_IVT_ID_RESULT_WH']:X}` energy every {cfg['CFG_IVT_ENERGY_PERIOD_MS']} ms; U3, temperature, "
        f"power and As off: {d['ivt_msgs_per_s']:g} result messages/s (IVT-S maximum 1000; U1 and U2 share one "
        "ADC, 2 ms minimum each). The BMS configures this at start-up.",
        "",
        f"CAN3, {d['can3_bitrate']}: charger (Elcon/TC protocol until the new charger section): "
        f"`0x{d['charger']['CHARGER_CONFIG_CAN_ID']:X}` command from the BMS every {cfg['CFG_CAN_TX_PERIOD_MS']} ms "
        f"(big-endian), `0x{d['charger']['CHARGER_STATUS_CAN_ID']:X}` status from the charger.",
        "",
        "Every signal: `can/bms_can1.dbc`, `can/ivt_can2.dbc`, `can/charger_can3.dbc`, or the matching "
        "`can/*_signals.csv` in the CAN sheet's columns.",
        "",
        "Runtime settings (GUI, `BMS_Config_Request` / `BMS_Config_Response`; values are int32 in the raw unit). "
        "A write applies at once, save stores them in flash (refused while the TS is active or charging), and saved "
        "settings load at start-up. The inverter limits can only go below the built-in values:",
        "",
        "| ID | Setting | Raw unit | Range | Built-in | |",
        "|---|---|---|---|---|---|",
    ]
    for s in settings_table(cfg):
        unit = s["unit"] or "-"
        raw_unit = unit if s["scale"] == 1 else f"{s['scale']:g} {unit}"
        shown, factor = ("kW", 0.001) if s["unit"] == "W" else (s["unit"].replace("degC", "°C"), 1)   # As the GUI shows them
        lo, hi, de = (s[k] * s["scale"] * factor for k in ("min", "max", "default"))
        lines.append(f"| {s['id']} | {s['label']} | {raw_unit} | {lo:g}-{hi:g} {shown} | {de:g} {shown} | "
                     f"{s['description']} |")
    return "\n".join(lines)


def readme_pins(cfg, d):
    lines = ["| Signal | Pin | Config | Active level |", "|---|---|---|---|"]
    for name, pin in d["pin"].items():
        desc, polarity, meaning = PIN_NOTES.get(name, (name, None, None))
        level = ""
        if polarity:
            high = bool(cfg.get(polarity, 1))
            if polarity.endswith("ACTIVE_LOW"):
                high = not high
            level = f"{'high' if high else 'low'} = {meaning}"
        lines.append(f"| {desc} | {pin} | `CFG_PIN_{name}_*` | {level} |")
    for n, (tx, rx) in d["can_pins"].items():
        lines.append(f"| CAN{n} TX / RX (to the transceiver) | {tx} / {rx} | `g474-bms.ioc` | |")
    return "\n".join(lines)


def replace_blocks(text, blocks, source):
    for name, content in blocks.items():
        pattern = re.compile(rf"(<!-- BEGIN GENERATED: {name}\b[^>]*-->\n).*?(<!-- END GENERATED: {name} -->)", re.S)
        if not pattern.search(text):
            raise GenerateError(f"{source}: markers for the generated '{name}' section are missing")
        text = pattern.sub(lambda m: m.group(1) + content + "\n" + m.group(2), text)
    return text


# ------------------------------------------------------------------------------------------------
# Templates: {{expression}} or {{expression:format}}, evaluated over the config and derived values
# ------------------------------------------------------------------------------------------------

def render(template, namespace, source):
    def substitute(m):
        body = m.group(1).strip()
        expr, fmt = body, ""
        if ":" in body:
            head, tail = body.rsplit(":", 1)
            if re.fullmatch(r"[<>^=+\- #0-9,._%a-zA-Z]*", tail):
                expr, fmt = head.strip(), tail.strip()
        try:
            value = eval(expr, {"__builtins__": {"round": round, "abs": abs, "int": int, "min": min, "max": max}}, namespace)
            return format(value, fmt)
        except Exception as e:
            line = template.count("\n", 0, m.start()) + 1
            raise GenerateError(f"{source}:{line}: can't fill in {{{{{body}}}}}: {e}")
    return re.sub(r"\{\{(.*?)\}\}", substitute, template)


# ------------------------------------------------------------------------------------------------
# CubeMX settings: checked against the config, never written
# ------------------------------------------------------------------------------------------------

def int_expr(text):
    if text is None or not re.fullmatch(r"[\d\s+\-*/()]+", text):
        raise ValueError(f"not a plain number: {text!r}")
    return int(eval(text.replace("/", "//")))


def clock_div(text):
    m = re.search(r"DIV(\d+)", text or "")
    return int(m.group(1)) if m else 1


def check_cubemx(cfg):
    problems = []
    ioc = dict(line.split("=", 1) for line in read_text(IOC).splitlines() if "=" in line and not line.startswith("#"))
    fdcan_c = read_text(FDCAN_C)
    tim_c = read_text(TIM_C)

    def c_field(text, field):
        m = re.search(re.escape(field) + r"\s*=\s*([^;]+);", text)
        return m.group(1).strip() if m else None

    def check(where, what, got, want, fix):
        if got != want:
            problems.append(f"{where}: {what} is {got}, bms_config.h wants {want} ({fix})")

    try:
        fdcan_clk = int(ioc["RCC.FDCANFreq_Value"])
        ioc_ips = {v for k, v in ioc.items() if re.fullmatch(r"Mcu\.IP\d+", k)}
        for n in (1, 2, 3):
            want = cfg[f"CFG_CAN{n}_BITRATE"]
            if f"FDCAN{n}" not in ioc_ips:
                problems.append(f"{IOC}: FDCAN{n} is not enabled")
                continue
            # .ioc (the clock divider is shared and taken from FDCAN1)
            tq = (1 + int(ioc[f"FDCAN{n}.NominalTimeSeg1"]) + int(ioc[f"FDCAN{n}.NominalTimeSeg2"]))
            rate = fdcan_clk // clock_div(ioc.get("FDCAN1.ClockDivider")) // int(ioc[f"FDCAN{n}.NominalPrescaler"]) // tq
            check(IOC, f"FDCAN{n} bit rate", rate, want, "change the FDCAN bit timing in CubeMX")
            # Generated code
            tq = 1 + int_expr(c_field(fdcan_c, f"hfdcan{n}.Init.NominalTimeSeg1")) + \
                int_expr(c_field(fdcan_c, f"hfdcan{n}.Init.NominalTimeSeg2"))
            rate = fdcan_clk // clock_div(c_field(fdcan_c, "hfdcan1.Init.ClockDivider")) // \
                int_expr(c_field(fdcan_c, f"hfdcan{n}.Init.NominalPrescaler")) // tq
            check(FDCAN_C, f"FDCAN{n} bit rate", rate, want, "generate the code from the .ioc again")

        timer_clk = int(ioc["RCC.APB2TimFreq_Value"])
        psc = int_expr(ioc["TIM16.Prescaler"])
        arr = int_expr(ioc.get("TIM16.PeriodNoDither", ioc.get("TIM16.Period")))
        check(IOC, "TIM16 period (us)", (psc + 1) * (arr + 1) * 1000000 // timer_clk, cfg["CFG_TIMER_TICK_MS"] * 1000,
              "set TIM16 prescaler/period in CubeMX")
        psc = int_expr(c_field(tim_c, "htim16.Init.Prescaler"))
        arr = int_expr(c_field(tim_c, "htim16.Init.Period"))
        check(TIM_C, "TIM16 period (us)", (psc + 1) * (arr + 1) * 1000000 // timer_clk, cfg["CFG_TIMER_TICK_MS"] * 1000,
              "generate the code from the .ioc again")
    except (KeyError, ValueError) as e:
        problems.append(f"CubeMX settings could not be read: {e}")
    return problems


# ------------------------------------------------------------------------------------------------
# Driver
# ------------------------------------------------------------------------------------------------

def gui_model(cfg):
    """What the GUI needs from the configuration, so it can't drift from the firmware"""
    text = read_text("Core/Inc/bms_config.h")
    model = {
        "generated": "from Core/Inc/bms_config.h by tools/generate.py: do not edit",
        "dbc": "protocol/bms_can1.dbc",
        "protocol_version": cfg["CFG_PROTOCOL_VERSION"],
        "faults": {str(1<<bit):name for bit,name in gen_dbc.read_status_flags()},
        "config_version": cfg["CFG_CONFIG_VERSION"],
        "can1_bitrate": cfg["CFG_CAN1_BITRATE"],
        "slaves": cfg["CFG_NUM_SLAVES"],
        "cells_per_slave": cfg["CFG_CELLS_PER_SLAVE"],
        "temps_per_slave": cfg["CFG_TEMPS_PER_SLAVE"],
        "cell_uv_v": cfg["CFG_CELL_UV_V"],
        "cell_ov_v": cfg["CFG_CELL_OV_V"],
        "cell_temp_min_c": cfg["CFG_CELL_TEMP_MIN_C"],
        "cell_temp_max_c": cfg["CFG_CELL_TEMP_MAX_C"],
        "cell_charge_temp_min_c": cfg["CFG_CELL_CHARGE_TEMP_MIN_C"],
        "cell_charge_temp_max_c": cfg["CFG_CELL_CHARGE_TEMP_MAX_C"],
        "temp_invalid_c": cfg["CFG_TEMP_INVALID_C"],
        "max_discharge_current_a": cfg["CFG_MAX_DISCHARGE_CURRENT_A"],
        "power_limit_w": cfg["CFG_INVERTER_POWER_LIMIT_W"],
        "soc": {
            "vmin": cfg["CFG_SOC_VMIN_V"],
            "vmax": cfg["CFG_SOC_VMAX_V"],
            "coeffs": read_array("CFG_SOC_COEFFS", text),
            "cell_min_v": cfg["CFG_SOC_CELL_MIN_V"],
            "cell_max_v": cfg["CFG_SOC_CELL_MAX_V"],
            "no_load_current_a": cfg["CFG_SOC_NO_LOAD_CURRENT_A"],
            "pack_energy_wh": round(cfg["CFG_CELL_ENERGY_WH"] * cfg["CFG_TOTAL_CELLS"], 3),
        },
        "config_request_id": cfg["CFG_CAN_ID_CONFIG_REQUEST"],
        "config_response_id": cfg["CFG_CAN_ID_CONFIG_RESPONSE"],
        "commands": {v: k for k, v in SETTINGS_COMMANDS.items()},
        "results": {str(k): v for k, v in SETTINGS_RESULTS.items()},
        "settings": settings_table(cfg),
    }
    return json.dumps(model, indent=2) + "\n"


def build_outputs():
    cfg = read_config(CONFIG)
    d = derived(cfg)
    can1 = gen_dbc.bms_can1(cfg)
    can2 = gen_dbc.ivt_can2(cfg)
    can3 = gen_dbc.charger_can3(cfg)
    namespace = dict(cfg)
    namespace.update(d)

    header = (f"<!-- Generated from {DOC_TEMPLATE} by tools/generate.py with the values in Core/Inc/bms_config.h. "
              "Edit the template, not this file. -->\n\n")
    outputs = {
        "can/bms_can1.dbc": can1.text(),
        "can/ivt_can2.dbc": can2.text(),
        "can/bms_can1_signals.csv": can1.csv(),
        "can/ivt_can2_signals.csv": can2.csv(),
        "can/charger_can3.dbc": can3.text(),
        "can/charger_can3_signals.csv": can3.csv(),
        DOC: header + render(read_text(DOC_TEMPLATE), namespace, DOC_TEMPLATE),
        README: replace_blocks(read_text(README), {"can": readme_can(cfg, d), "pins": readme_pins(cfg, d)}, README),
        GUI_MODEL: gui_model(cfg),
    }
    registry=[]
    for bus,db in [('TSCAN',can1),('TSACCAN',can2),('CHRGCAN',can3)]:
        seen=set()
        for msg in db.messages:
            key=(msg['id'],msg['extended'])
            if key in seen:raise GenerateError(f'Duplicate same-bus CAN owner {bus} {key}')
            seen.add(key)
            registry.append(dict(bus=bus,direction='TX' if msg['sender']=='BMS' else 'RX',can_id=msg['id'],extended=msg['extended'],dlc=msg.get('dlc',8),period_ms=msg['cycle_ms'],owner=msg['sender'],message=msg['name'],signals=msg['signals']))
    outputs['can/registry.json']=json.dumps({'protocol_version':cfg['CFG_PROTOCOL_VERSION'],'config_version':cfg['CFG_CONFIG_VERSION'],'faults':{str(1<<bit):name for bit,name in gen_dbc.read_status_flags()},'settings':settings_table(cfg),'messages':registry},indent=2)+'\n'
    return outputs, check_cubemx(cfg)


def stale(outputs):
    changed = []
    for rel, content in outputs.items():
        try:
            current = read_text(rel)
        except FileNotFoundError:
            current = None
        if current != content:
            changed.append(rel)
    return changed


def write(outputs, rels):
    for rel in rels:
        os.makedirs(os.path.dirname(path(rel)), exist_ok=True)
        with open(path(rel), "w", encoding="utf-8", newline="\n") as f:
            f.write(outputs[rel])


def git(*args):
    return subprocess.run(["git", *args], cwd=ROOT, capture_output=True, text=True)


def hooks_hint():
    try:
        if os.path.isdir(path(".git")) and git("config", "--get", "core.hooksPath").stdout.strip() != "tools/githooks":
            print("generate: tip: 'git config core.hooksPath tools/githooks' regenerates on every commit")
    except OSError:
        pass


def run_once(mode):
    try:
        outputs, problems = build_outputs()
    except GenerateError as e:
        print(f"generate: ERROR {e}")
        return 1
    changed = stale(outputs)

    if mode == "check":
        for rel in changed:
            print(f"generate: {rel} is out of date (run python tools/generate.py)")
        if not changed:
            print("generate: everything generated from bms_config.h is up to date")
    elif mode == "pre-commit":
        unstaged = set(git("diff", "--name-only").stdout.split())
        write(outputs, changed)
        blocked = [rel for rel in changed if rel in unstaged]
        staged = [rel for rel in changed if rel not in unstaged]
        if staged:
            git("add", "--", *staged)
            print("generate: updated and added to this commit: " + ", ".join(staged))
        for rel in blocked:
            print(f"generate: {rel} had unstaged edits and has been regenerated: review it, stage it, commit again")
        if blocked:
            problems.append("generated files need staging")
    else:
        write(outputs, changed)
        for rel in changed:
            print(f"generate: updated {rel}")
        if not changed:
            print("generate: everything generated from bms_config.h is up to date")

    for p in problems:
        print(f"generate: ERROR {p}")
    return 1 if problems or (mode == "check" and changed) else 0


def watch():
    print("generate: watching " + ", ".join(SOURCES) + " (Ctrl+C to stop)")
    last = None
    while True:
        stamp = [os.path.getmtime(path(rel)) if os.path.exists(path(rel)) else 0 for rel in SOURCES]
        if stamp != last:
            if last is not None:
                print(time.strftime("generate: %H:%M:%S source changed"))
            run_once("write")
            last = stamp
        time.sleep(1)


def main(argv=None):
    parser = argparse.ArgumentParser(description="Regenerate the files derived from bms_config.h")
    group = parser.add_mutually_exclusive_group()
    group.add_argument("--check", action="store_true", help="write nothing; exit 1 if anything is out of date")
    group.add_argument("--pre-commit", action="store_true", help="write and stage what changed (git hook)")
    group.add_argument("--watch", action="store_true", help="regenerate whenever a source changes")
    args = parser.parse_args(argv)

    if args.watch:
        try:
            watch()
        except KeyboardInterrupt:
            return 0
    mode = "check" if args.check else "pre-commit" if args.pre_commit else "write"
    result = run_once(mode)
    if mode == "write":
        hooks_hint()
    return result


if __name__ == "__main__":
    sys.exit(main())

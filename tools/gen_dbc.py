"""
CAN databases and signal lists, built from Core/Inc/bms_config.h (plus the charger IDs in bms_can.h
and the fault flags in main.h), so they always match the firmware. Used by tools/generate.py.

CAN1 (car / datalogger bus, shared with the inverter): everything the BMS sends, plus the TS request and
the GUI's settings requests.
CAN2 (inside the TSAC): the IVT-S messages the BMS uses (IVT-S datasheet v1.01, chapter 8).
CAN3: the charger.

The frame layouts here mirror the packing code in bms_libWrapper.c; test/check_dbc.py decodes the
simulated firmware's traffic with these files, so a mismatch fails the test suite.
"""

import csv
import io
import os
import re
import sys
from pathlib import Path

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CONFIG = os.path.join(ROOT, "Core", "Inc", "bms_config.h")
CAN_HEADER = os.path.join(ROOT, "Core", "Inc", "bms_can.h")
MAIN_HEADER = os.path.join(ROOT, "Core", "Inc", "main.h")
OUT_DIR = os.path.join(ROOT, "can")

# Column order of the team's CAN signal sheet
CSV_COLUMNS = ["CAN ID", "ID Type", "Rate (Hz)", "Bit Position", "Bit Length", "Byte Order", "Signed", "Frame Name",
               "Channel", "Units", "Type", "Conversion", "Min", "Max", "Source", "Node", "Destination", "Notes"]

# Signal names for the BMS_StatusTypeDef bits, where plain title case would read badly
FLAG_NAMES = {"OPEN_WIRE":"OpenWire","IVT":"IVT","CAN":"CAN","PEC":"PEC","CELL_UV":"CellUV","CELL_OV":"CellOV","CHARGE_TEMP":"ChargeTemperature"}


def configuration_text():
    """Vehicle protocol schema uses the 14-device digital reference profile.
    Hardware profiles remain separate; BENCH advertises its topology at runtime.
    """
    files=[Path(ROOT)/'Core/Inc/profiles/board_digital.h',Path(ROOT)/'Core/Inc/profiles/bms_mcu_routes.h',Path(ROOT)/'Core/Inc/profiles/bms_board_profile.h']
    files+=sorted((Path(ROOT)/'Core/Inc/config').glob('*.h'))
    files+=[Path(CONFIG)]
    return '\n'.join(p.read_text(encoding='utf-8') for p in files)

def macros():
    text=re.sub(r'/\*.*?\*/','',configuration_text(),flags=re.S).replace('\\\n',' ')
    return {m.group(1):m.group(2).split('//')[0].strip() for m in re.finditer(r'^\s*#define\s+(\w+)\s+([^\n]+)',text,re.M)}

def expand(name):
    defs=macros();expr=defs[name]
    for _ in range(30):
        new=re.sub(r'\b(?:CFG_|BMS_)\w+\b',lambda m:defs.get(m[0],m[0]),expr)
        if new==expr:return expr
        expr=new
    raise ValueError('Recursive configuration '+name)

def read_config(path=CONFIG):
    values={}
    for name in macros():
        if not name.startswith('CFG_'):continue
        expr=expand(name)
        expr=re.sub(r'(?<=\d)[uUlLfF]+\b','',expr)
        expr=expr.replace('&&',' and ').replace('||',' or ').replace('/','//')
        try:
            value=eval(expr,{'__builtins__':{}},{})
            values[name]=int(value) if isinstance(value,float) and value.is_integer() else value
        except (SyntaxError,NameError,TypeError):pass
    return values

def read_pins(path=CONFIG):
    text=configuration_text();pins={}
    for m in re.finditer(r'#define\s+CFG_PIN_(\w+)_PORT\s+GPIO([A-G])\b',text):
        pin=re.search(rf'#define\s+CFG_PIN_{m.group(1)}_PIN\s+GPIO_PIN_(\d+)\b',text)
        if pin:pins[m.group(1)]=f'P{m.group(2)}{pin.group(1)}'
    return pins


def read_charger_ids(path=CAN_HEADER):
    text = open(path, encoding="utf-8").read()
    ids = {}
    for name in ("CHARGER_CONFIG_CAN_ID", "CHARGER_STATUS_CAN_ID"):
        ids[name] = int(re.search(rf"#define\s+{name}\s+(0x[0-9A-Fa-f]+)", text).group(1), 16)
    return ids


def read_status_flags(path=MAIN_HEADER):
    """BMS_ERR_* bits from main.h -> [(bit, signal name), ...]"""
    flags = []
    for m in re.finditer(r"\bBMS_ERR_(\w+)\s*=\s*(0x[0-9A-Fa-f]+|\d+)", open(path, encoding="utf-8").read()):
        value = int(m.group(2), 0)
        name = FLAG_NAMES.get(m.group(1), m.group(1).title().replace("_", ""))
        flags.append((value.bit_length() - 1, name))
    return flags


def settings_table(cfg):
    """Runtime settings (bms_settings.c): id, DBC/GUI name, label, unit, scale (raw -> unit) and the raw
    default/min/max the firmware works out from bms_config.h (rounded like its WHOLE()/TENTHS() macros)."""
    def raw(value, scale):
        return int(value / scale + (0.5 if value >= 0 else -0.5))

    rows = [
        (1, "Power_Limit", "Power limit", "W", 1, cfg["CFG_INVERTER_POWER_LIMIT_W"], cfg["CFG_SETTING_POWER_MIN_W"],
         cfg["CFG_INVERTER_POWER_LIMIT_W"], "Discharge power limit sent to the inverter (the window limit with the "
         "moving-window limiter). Can only be lowered below the built-in value"),
        (2, "Discharge_Current_Limit", "Discharge current limit", "A", 0.1, cfg["CFG_MAX_DISCHARGE_CURRENT_A"],
         cfg["CFG_SETTING_CURRENT_MIN_A"], cfg["CFG_MAX_DISCHARGE_CURRENT_A"],
         "Inverter DC current limit before derating. The AMS trip stays at the built-in value"),
        (3, "Temp_Limit", "Temperature limit", "degC", 0.1, cfg["CFG_INVERTER_DERATE_END_C"],
         cfg["CFG_SETTING_TEMP_LIMIT_MIN_C"], cfg["CFG_INVERTER_DERATE_END_C"],
         f"Hottest cell at which both inverter limits are fully derated (discharge to "
         f"{cfg['CFG_INVERTER_DERATE_CURRENT_A']:g} A, regen to {cfg['CFG_INVERTER_REGEN_DERATE_CURRENT_A']:g} A). "
         "Lowering it moves both derating ramps down by the same amount. The AMS over-temperature trip is not changed"),
        (4, "Regen_Enabled", "Regen", "", 1, 1, 0, 1, "0 = off (0 A brake current to the inverter), 1 = on"),
        (5, "Regen_Current_Limit", "Regen current limit", "A", 0.1, cfg["CFG_INVERTER_REGEN_CURRENT_A"], 0,
         cfg["CFG_INVERTER_REGEN_CURRENT_A"], "Inverter DC brake current limit before derating"),
        (6, "Charge_Current", "Charge current", "A", 0.1, cfg["CFG_CHARGER_CURRENT_A"], 0, cfg["CFG_CHARGER_CURRENT_MAX_A"],
         "Charger current setpoint"),
        (7, "Charge_Voltage", "Charge voltage", "V", 0.1, cfg["CFG_CHARGER_VOLTAGE_V"], cfg["CFG_PACK_V_MIN"],
         cfg["CFG_CHARGER_VOLTAGE_MAX_V"], "Charger voltage setpoint (pack voltage at the end of charge)"),
    ]
    return [dict(id=i, name=n, label=l, unit=u, scale=s, default=raw(d, s), min=raw(lo, s), max=raw(hi, s), description=c)
            for i, n, l, u, s, d, lo, hi, c in rows]


SETTINGS_COMMANDS = {1: "Read", 2: "Write", 3: "Save", 4: "Defaults", 5: "Read_All"}
SETTINGS_RESULTS = {0: "OK", 1: "Unknown_Setting", 2: "Out_Of_Range", 3: "TS_Active", 4: "Flash_Error",
                    5: "Unknown_Command", 6: "Busy", 7: "Incompatible_Version", 8: "Malformed"}


def bitrate_text(bps):
    return f"{bps / 1e6:g} Mbit/s" if bps >= 1000000 else f"{bps / 1e3:g} kbit/s"


class Dbc:
    def __init__(self, bus_comment, nodes):
        self.lines = []
        self.comments = []
        self.cycle_times = []
        self.value_tables = []
        self.nodes = nodes
        self.bus_comment = bus_comment
        self.rows = []
        self.messages=[]
        self._msg = None

    def values(self, signal, table):
        entries = " ".join(f'{k} "{v}"' for k, v in table.items())
        self.value_tables.append(f"VAL_ {self._msg['raw_id']} {signal} {entries} ;")

    def message(self, can_id, name, sender, extended, cycle_ms=None, comment=None, dlc=8):
        raw_id = can_id | 0x80000000 if extended else can_id
        self.lines.append(f"\nBO_ {raw_id} {name}: {dlc} {sender}")
        self._msg = dict(raw_id=raw_id, id=can_id, name=name, sender=sender, extended=extended, cycle_ms=cycle_ms,dlc=dlc,signals=[])
        self.messages.append(self._msg)
        if cycle_ms:
            self.cycle_times.append(f'BA_ "GenMsgCycleTime" BO_ {raw_id} {cycle_ms};')
        if comment:
            self.comments.append(f'CM_ BO_ {raw_id} "{comment}";')

    def signal(self, name, start, length, little_endian=True, signed=False, scale=1, offset=0,
               minimum=0, maximum=0, unit="", receivers="Vector__XXX", comment=None, mux=None):
        """mux: "M" for the multiplexer, or the multiplexer value this signal belongs to"""
        self._msg["signals"].append(dict(name=name,start_bit=start,bits=length,little_endian=little_endian,signed=signed,scale=scale,offset=offset,minimum=minimum,maximum=maximum,unit=unit,multiplexer=mux))
        order = "1" if little_endian else "0"
        sign = "-" if signed else "+"
        if length == 1 and maximum == 0:
            maximum = 1
        mux_text = "" if mux is None else (" M" if mux == "M" else f" m{mux}")
        self.lines.append(f' SG_ {name}{mux_text} : {start}|{length}@{order}{sign} ({scale:g},{offset:g}) '
                          f'[{minimum:g}|{maximum:g}] "{unit}" {receivers}')
        if mux is not None and mux != "M":
            comment = f"Only when the multiplexer is {mux}" + (f". {comment}" if comment else "")
        if comment:
            self.comments.append(f'CM_ SG_ {self._msg["raw_id"]} {name} "{comment}";')

        m = self._msg
        conversion = "y = x" if scale == 1 else f"y = {scale:g}x"
        if offset:
            conversion += f" {'+' if offset > 0 else '-'} {abs(offset):g}"
        self.rows.append({
            "CAN ID": f"0x{m['id']:X}",
            "ID Type": "Extended" if m["extended"] else "Standard",
            "Rate (Hz)": f"{1000 / m['cycle_ms']:g}" if m["cycle_ms"] else "event",
            "Bit Position": start,
            "Bit Length": length,
            "Byte Order": "Little" if little_endian else "Big",
            "Signed": "yes" if signed else "no",
            "Frame Name": m["name"],
            "Channel": name,
            "Units": unit,
            "Type": "bool" if length == 1 else f"{'int' if signed else 'uint'}{length}",
            "Conversion": conversion,
            "Min": f"{minimum:g}",
            "Max": f"{maximum:g}",
            "Source": m["sender"],
            "Node": "n/a",
            "Destination": "" if receivers == "Vector__XXX" else receivers,
            "Notes": comment or "",
        })

    def text(self):
        out = ['VERSION ""', "", "NS_ :", "", "BS_:", "", f"BU_: {' '.join(self.nodes)}"]
        out += self.lines
        out += ["", f'CM_ "{self.bus_comment}";']
        out += self.comments
        out += ['', 'BA_DEF_ BO_ "GenMsgCycleTime" INT 0 65535;', 'BA_DEF_DEF_ "GenMsgCycleTime" 0;']
        out += self.cycle_times
        out += [""] + self.value_tables
        return "\n".join(out) + "\n"

    def csv(self):
        """One row per signal, in the team's CAN sheet columns. Bit positions are DBC start bits
        (big-endian signals: the MSB, counting down)."""
        buf = io.StringIO()
        writer = csv.DictWriter(buf, fieldnames=CSV_COLUMNS, lineterminator="\n")
        writer.writeheader()
        writer.writerows(self.rows)
        return buf.getvalue()


def bms_can1(cfg):
    period = cfg["CFG_CAN_TX_PERIOD_MS"]
    slaves = cfg["CFG_NUM_SLAVES"]
    cells = cfg["CFG_CELLS_PER_SLAVE"]
    temps = cfg["CFG_TEMPS_PER_SLAVE"]
    v_frames = cfg["CFG_CAN_V_FRAMES_PER_SLAVE"]
    t_frames = cfg["CFG_CAN_T_FRAMES_PER_SLAVE"]
    power_lsb = cfg["CFG_CAN_POWER_W_PER_BIT"]
    power_max = 32767 * power_lsb
    power_comment = (f"IVT current x U1, positive = discharge. Saturates at +-32767 (+-{power_max:g} W). "
                     "Raw -32768 = not available")
    remaining_comment = "Raw 65535 = unknown (start-up SoC estimate not taken yet)"

    d = Dbc(f"BMS CAN1 (car / datalogger bus), {bitrate_text(cfg['CFG_CAN1_BITRATE'])}, generated from bms_config.h "
            "by tools/generate.py", ["BMS", "VCU", "Inverter", "GUI"])

    d.message(cfg["CFG_CAN_ID_PACK"], "BMS_Pack", "BMS", True, period, "IVT-S pack measurements and sum of cells")
    d.signal("Pack_Voltage_IVT", 0, 16, signed=True, scale=0.1, minimum=-3276.8, maximum=3276.7, unit="V")
    d.signal("Pack_Current", 16, 16, signed=True, scale=0.1, minimum=-3276.8, maximum=3276.7, unit="A",
             comment="Positive = discharge")
    d.signal("Pack_Voltage_SumOfCells", 32, 16, signed=True, scale=0.1, minimum=-3276.8, maximum=3276.7, unit="V")
    d.signal("IVT_Current_Timeout", 48, 1)
    d.signal("IVT_Voltage_Timeout", 49, 1)
    d.signal("IVT_Sensor_Error", 50, 1)

    d.message(cfg["CFG_CAN_ID_STATUS"], "BMS_Status", "BMS", True, period, "AMS state")
    for bit, name in read_status_flags():
        if bit<8:d.signal(f"Fault_{name}", bit, 1)
    d.signal("Fault_Output_Active", 8, 1, comment="AMS fault output to the SDC latch")
    d.signal("TSAC_Ready", 9, 1, comment="TSAC ready relay on")
    d.signal("Balancing_Enabled", 10, 1)
    d.signal("Charging", 11, 1)
    d.signal("SDC_Closed", 12, 1)
    d.signal("Local_Config_Overrides", 13, 1, comment="Built with bms_config_local.h (bench overrides): not for the car")
    d.signal("Inverter_Temp_Derating", 14, 1, comment="Inverter current limits derated for cell temperature")
    d.signal("Inverter_Power_Limited", 15, 1, comment="Inverter discharge limit held to the power limit")
    d.signal("Measurement_Age", 16, 16, signed=True, maximum=32767, unit="ms")
    d.signal("Measurement_Cycle_Time", 32, 16, signed=True, maximum=32767, unit="ms")
    d.signal("Precharge_State", 48, 8, maximum=4)
    d.values("Precharge_State", {0: "Idle", 1: "Precharging", 2: "Closing", 3: "TS_Active", 4: "Failed"})
    d.signal("Precharge_Percent", 56, 8, maximum=255, unit="%", comment="Vehicle side (IVT U2) / battery (IVT U1)")

    d.message(cfg["CFG_CAN_ID_ENERGY"], "BMS_Energy", "BMS", True, period, "Power and energy from the IVT-S")
    d.signal("Power", 0, 16, signed=True, scale=power_lsb, minimum=-power_max, maximum=power_max, unit="W",
             comment=power_comment)
    d.signal("Energy_Remaining", 16, 16, maximum=65535, unit="Wh", comment=remaining_comment)
    d.signal("SoC", 32, 16, scale=0.1, maximum=6553.5, unit="%", comment="6553.5 = not estimated yet")
    d.signal("Energy_Used", 48, 16, signed=True, minimum=-32768, maximum=32767, unit="Wh",
             comment="Since start-up, positive = discharged")

    fast = cfg["CFG_CAN_FAST_PERIOD_MS"]
    d.message(cfg["CFG_CAN_ID_FAST_1"], "BMS_Fast_1", "BMS", True, fast,
              "IVT-S current and pack voltage at the IVT's own resolution and output rate")
    d.signal("Fast_Pack_Current", 0, 32, signed=True, scale=0.001, minimum=-2147483.647, maximum=2147483.647, unit="A",
             comment="IVT current, 1 mA steps, positive = discharge. Raw -2147483648 = not available")
    d.signal("Fast_Pack_Voltage", 32, 32, signed=True, scale=0.001, minimum=-2147483.647, maximum=2147483.647, unit="V",
             comment="IVT U1, battery side of the AIRs, 1 mV steps. Raw -2147483648 = not available")

    d.message(cfg["CFG_CAN_ID_FAST_2"], "BMS_Fast_2", "BMS", True, fast, "Vehicle side voltage, power and remaining energy")
    d.signal("Fast_Vehicle_Voltage", 0, 32, signed=True, scale=0.001, minimum=-2147483.647, maximum=2147483.647, unit="V",
             comment="IVT U2, vehicle side of the AIRs, 1 mV steps. Raw -2147483648 = not available")
    d.signal("Fast_Power", 32, 16, signed=True, scale=power_lsb, minimum=-power_max, maximum=power_max, unit="W",
             comment=power_comment)
    d.signal("Fast_Energy_Remaining", 48, 16, maximum=65535, unit="Wh", comment=remaining_comment)

    limit_period = cfg["CFG_INVERTER_LIMIT_PERIOD_MS"]
    d.message(cfg["CFG_CAN_ID_LIMITS"], "BMS_Limits", "BMS", True, limit_period,
              "Live discharge power limit and the current limits sent to the inverter")
    d.signal("Discharge_Power_Limit", 0, 16, signed=True, scale=power_lsb, minimum=-power_max, maximum=power_max, unit="W",
             comment="Discharge current limit x pack voltage: the power limit when that applies, less when the current "
                     "limit or derating does")
    d.signal("Discharge_Current_Limit", 16, 16, signed=True, scale=0.1, minimum=-3276.7, maximum=3276.7, unit="A")
    d.signal("Regen_Current_Limit", 32, 16, signed=True, scale=0.1, minimum=-3276.7, maximum=3276.7, unit="A",
             comment="Magnitude; sent to the inverter as a negative brake current")
    d.signal("Limit_Temp_Derating", 48, 1, comment="Discharge limit derated for cell temperature")
    d.signal("Limit_Power", 49, 1, comment="Discharge held to the power limit")
    d.signal("Limit_Regen_Temp_Derating", 50, 1, comment="Regen limit derated for cell temperature")
    d.signal("Limit_AMS_Fault", 51, 1, comment="AMS faulted: both limits 0 A")
    d.signal("Limit_Boost", 52, 1, comment="Moving-window limiter allowing more than the window limit")
    d.signal("Limit_Settings_Modified", 53, 1, comment="Runtime settings (GUI) differ from the built-in limits")
    d.signal("Window_Avg_Power", 56, 8, scale=500, maximum=127500, unit="W",
             comment="500 ms moving average of the IVT's discharge power, as FS 2027 D 10.4.1 judges it")

    if cfg["CFG_INVERTER_LIMITS_ENABLED"]:
        d.message(cfg["CFG_CAN_ID_INVERTER_DC_LIMIT"], "BMS_Inverter_DC_Limit", "BMS", True, limit_period,
                  "DTI Set max. DC current (packet 0x0A), big-endian, unused bytes 0xFF")
        d.signal("Inverter_Max_DC_Current", 7, 16, little_endian=False, signed=True, scale=0.1, minimum=-3276.8,
                 maximum=3276.7, unit="A", receivers="Inverter", comment="Battery discharge limit set by the BMS")
        d.message(cfg["CFG_CAN_ID_INVERTER_DC_BRAKE_LIMIT"], "BMS_Inverter_DC_Brake_Limit", "BMS", True, limit_period,
                  "DTI Set max. DC brake current (packet 0x0B), big-endian, unused bytes 0xFF")
        d.signal("Inverter_Max_DC_Brake_Current", 7, 16, little_endian=False, signed=True, scale=0.1, minimum=-3276.8,
                 maximum=3276.7, unit="A", receivers="Inverter", comment="Battery charge (regen) limit set by the BMS, negative")

    d.message(cfg["CFG_CAN_ID_SUMMARY"], "BMS_Summary", "BMS", True, period, "Pack extremes")
    d.signal("Cell_Voltage_Min", 0, 16, signed=True, scale=0.001, minimum=0, maximum=5, unit="V")
    d.signal("Cell_Voltage_Max", 16, 16, signed=True, scale=0.001, minimum=0, maximum=5, unit="V")
    d.signal("Cell_Temp_Min", 32, 16, signed=True, scale=0.1, minimum=-40, maximum=888, unit="degC")
    d.signal("Cell_Temp_Max", 48, 16, signed=True, scale=0.1, minimum=-40, maximum=888, unit="degC")

    for s in range(slaves):
        seg = f"Seg{s + 1:02d}"
        d.message(cfg["CFG_CAN_ID_SLAVE_BASE"] + s, f"BMS_{seg}", "BMS", True, period, f"Slave {s + 1} status")
        d.signal(f"{seg}_Voltage", 0, 16, signed=True, scale=0.01, minimum=0, maximum=100, unit="V")
        d.signal(f"{seg}_Die_Temp", 16, 16, signed=True, scale=0.1, minimum=-40, maximum=150, unit="degC")
        for c in range(cells):
            d.signal(f"{seg}_Cell{c + 1:02d}_Balancing", 32 + c, 1)
        for c in range(cells):
            d.signal(f"{seg}_Cell{c + 1:02d}_Fault", 40 + c, 1)
        for c in range(cells):
            d.signal(f"{seg}_Cell{c + 1:02d}_OpenWire", 48 + c, 1)
        d.signal(f"{seg}_Comms_Error", 56, 1)
        d.signal(f"{seg}_Fault", 57, 1)

        for k in range(v_frames):
            d.message(cfg["CFG_CAN_ID_CELL_V_BASE"] + s * v_frames + k, f"BMS_{seg}_Voltages_{k + 1}", "BMS", True, period)
            for j in range(4):
                c = k * 4 + j
                if c < cells:
                    d.signal(f"{seg}_Cell{c + 1:02d}_Voltage", j * 16, 16, signed=True, scale=0.001,
                             minimum=0, maximum=5, unit="V")

        for k in range(t_frames):
            d.message(cfg["CFG_CAN_ID_CELL_T_BASE"] + s * t_frames + k, f"BMS_{seg}_Temps_{k + 1}", "BMS", True, period)
            for j in range(4):
                t = k * 4 + j
                if t < temps:
                    d.signal(f"{seg}_Temp{t + 1:02d}", j * 16, 16, signed=True, scale=0.1, minimum=-40, maximum=888,
                             unit="degC", comment="888 degC = sensor open or out of range")

    # Runtime settings (bms_settings.h): request from the GUI, one response per setting answered
    settings = settings_table(cfg)
    setting_names = {0: "None"}
    setting_names.update({s["id"]: s["name"] for s in settings})

    def setting_values(receiver):
        d.signal("Config_Value_None", 32, 32, signed=True, minimum=-2147483648, maximum=2147483647, receivers=receiver,
                 mux=0, comment="Save, defaults: no setting")
        for s in settings:
            d.signal(f"Setting_{s['name']}", 32, 32, signed=True, scale=s["scale"], minimum=s["min"] * s["scale"],
                     maximum=s["max"] * s["scale"], unit=s["unit"], receivers=receiver, mux=s["id"],
                     comment=f"{s['description']}. Default {s['default'] * s['scale']:g}")

    d.message(cfg["CFG_CAN_ID_CONFIG_REQUEST"], "BMS_Config_Request", "GUI", True, None,
              "Runtime settings request (GUI). Write applies at once; Save stores all settings in flash (refused "
              "while the TS is active or charging)")
    d.signal("Config_Command", 0, 8, maximum=255, receivers="BMS")
    d.values("Config_Command", SETTINGS_COMMANDS)
    d.signal("Config_Setting", 8, 8, maximum=255, receivers="BMS", mux="M", comment="0 for Save, Defaults, Read_All")
    d.values("Config_Setting", setting_names)
    d.signal("Config_Version",16,8,maximum=255,receivers="BMS")
    d.signal("Config_Token",24,8,maximum=31,receivers="BMS")
    setting_values("BMS")

    d.message(cfg["CFG_CAN_ID_CONFIG_RESPONSE"], "BMS_Config_Response", "BMS", True, None,
              "Answer to BMS_Config_Request: the setting's value now in force (Read_All: one per setting)")
    d.signal("Config_Command", 0, 8, maximum=255, receivers="GUI")
    d.values("Config_Command", SETTINGS_COMMANDS)
    d.signal("Config_Setting", 8, 8, maximum=255, receivers="GUI", mux="M")
    d.values("Config_Setting", setting_names)
    d.signal("Config_Result", 16, 8, maximum=255, receivers="GUI")
    d.values("Config_Result", SETTINGS_RESULTS)
    d.signal("Config_Unsaved", 24, 1, receivers="GUI", comment="Settings differ from what flash holds")
    d.signal("Config_Modified", 25, 1, receivers="GUI", comment="Settings differ from the built-in values")
    d.signal("Config_Stored", 26, 1, receivers="GUI", comment="Flash holds saved settings")
    d.signal("Config_Token",27,5,maximum=31,receivers="GUI")
    setting_values("GUI")

    d.message(cfg["CFG_CAN_ID_TS_REQUEST"], "VCU_TS_Request", "VCU", bool(cfg["CFG_CAN_TS_REQUEST_EXTENDED"]),
              cfg["CFG_TS_REQUEST_TIMEOUT_MS"] or None, "TS activate request to the BMS (ID TBD)")
    d.signal("TS_Request", 0, 8, maximum=1, receivers="BMS",
             comment="1 = activate (starts precharge on a change from 0), 0 = deactivate")
    d.values("TS_Request", {0: "Deactivate", 1: "Activate"})
    for ic in range(cfg['CFG_NUM_SLAVES']):
        prefix=f'Seg{ic+1:02d}'
        for frame in range(cfg['CFG_CAN_T_FRAMES_PER_SLAVE']):
            d.message(cfg['CFG_CAN_ID_RAW_AUX_BASE']+ic*cfg['CFG_CAN_T_FRAMES_PER_SLAVE']+frame,prefix+f'_RawAux{frame+1}','BMS',True,cfg['CFG_CAN_TX_PERIOD_MS'])
            for j in range(4):d.signal(prefix+f'_RawAux{frame*4+j+1:02d}',j*16,16,signed=True,scale=0.001,minimum=-32.768,maximum=32.767,unit='V')
        d.message(cfg['CFG_CAN_ID_DIAGNOSTIC_BASE']+ic,prefix+'_Diagnostics','BMS',True,cfg['CFG_CAN_TX_PERIOD_MS'])
        for name,start,bits in [('PEC_Count',0,16),('Counter_Count',16,16),('RawAux_ValidMask',32,8),('Celsius_ValidMask',40,8),('Transport_Error',48,8),('BalanceStop_Failed',56,8)]:d.signal(prefix+'_'+name,start,bits,maximum=(1<<bits)-1)
    d.message(cfg['CFG_CAN_ID_IDENTITY'],'BMS_Identity','BMS',True,100)
    for i,name in enumerate(['ProtocolVersion','ConfigVersion','BoardProfile','Slaves','CellsPerSlave','TempsPerSlave','PhysicalChannels','Bench']):d.signal('BMS_'+name,i*8,8,maximum=255)
    d.message(cfg['CFG_CAN_ID_QUALITY'],'BMS_Quality','BMS',True,100)
    d.signal('BMS_FaultMask',0,32,maximum=4294967295)
    for i,name in enumerate(['CellsValid','AuxValid','CelsiusValid','IVTConfigured','CurrentValid','U1Valid','U2Valid','EnergyValid']):d.signal('BMS_'+name,32+i,1)
    d.signal('BMS_SampleAge',40,16,maximum=65535,unit='ms');d.signal('BMS_Generation',56,8,maximum=255)
    d.message(cfg['CFG_CAN_ID_IVT_DIAGNOSTIC'],'BMS_IVT_Diagnostics','BMS',True,100)
    for i,name in enumerate(['CurrentStatusCounter','U1StatusCounter','U2StatusCounter','EnergyStatusCounter','Validity','RestartEpoch']):d.signal('IVT_'+name,i*8,8,maximum=255)
    d.signal('IVT_CounterGaps',48,16,maximum=65535)
    d.message(cfg['CFG_CAN_ID_CHARGER'],'BMS_Charger','BMS',True,100)
    d.signal('Charger_State',0,8,maximum=5);d.signal('Charger_FaultMask',8,16,maximum=65535);d.signal('Charger_Healthy',24,1);d.signal('Charger_Enabled',25,1)
    d.signal('Charger_Voltage',32,16,signed=True,scale=0.1,minimum=-3276.8,maximum=3276.7,unit='V');d.signal('Charger_Current',48,16,signed=True,scale=0.1,minimum=-3276.8,maximum=3276.7,unit='A')

    return d


def charger_can3(cfg):
    ids=read_charger_ids()
    d=Dbc('Current supplied charger gateway; BCD setpoints, 500 kbit/s. Requires hardware protocol confirmation.', ['BMS','Charger'])
    d.message(ids['CHARGER_CONFIG_CAN_ID'],'BMS_Charger_Command','BMS',True,100,'BCD bytes 0..2 voltage /1000 V; bytes 3..5 current /100000 A')
    for i in range(6):d.signal(f'Setpoint_BCD_Byte{i}',i*8,8,maximum=153,receivers='Charger')
    d.message(0x001FFFFF,'BMS_Charger_Inhibit','BMS',True,100,'AA 00 00 means inhibit; 00 00 00 healthy heartbeat only with deliberate charging enable')
    d.signal('Charger_Inhibit_Code',0,24,maximum=0xFFFFFF,receivers='Charger')
    d.message(ids['CHARGER_STATUS_CAN_ID'],'Charger_Status','Charger',True,None)
    d.signal('Charger_State',0,8,maximum=5,receivers='BMS');d.values('Charger_State',{0:'Idle',1:'BatteryConnected',2:'Precharge',3:'Active',4:'Discharge',5:'Error'})
    for i,name in enumerate(['UARTTimeout','LVFault','InputUV','InputOV','OutputOV','OutputOC','OutputPower','OverTemp','BatteryReverse','BatteryOV']):d.signal('Charger_'+name,8+i,1,receivers='BMS')
    d.message(0x01000000,'Charger_Measurements','Charger',True,None)
    for i,name in enumerate(['PFCVoltage','OutputVoltage','OutputCurrent','BatteryVoltage']):d.signal('Charger_'+name,i*16,16,signed=True,scale=0.1,minimum=-3276.8,maximum=3276.7,receivers='BMS')
    d.message(0x02000000,'Charger_Thermal','Charger',True,None)
    for i,name in enumerate(['Power','PFCTemp','PSFBTemp','AuxTemp']):d.signal('Charger_'+name,i*16,16,signed=True,scale=1 if i==0 else 0.1,minimum=-32768,maximum=32767,receivers='BMS')
    return d


def ivt_can2(cfg):
    d = Dbc(f"IVT-S on CAN2, {bitrate_text(cfg['CFG_CAN2_BITRATE'])}, factory default IDs (IVT-S datasheet v1.01)",
            ["BMS", "IVT"])
    results = [
        (cfg["CFG_IVT_ID_RESULT_I"], "IVT_Msg_Result_I", "I", 0.001, "A", cfg["CFG_IVT_CURRENT_PERIOD_MS"]),
        (cfg["CFG_IVT_ID_RESULT_U1"], "IVT_Msg_Result_U1", "U1", 0.001, "V", cfg["CFG_IVT_U1_PERIOD_MS"]),
        (cfg["CFG_IVT_ID_RESULT_U2"], "IVT_Msg_Result_U2", "U2", 0.001, "V", cfg["CFG_IVT_U2_PERIOD_MS"]),
        (cfg["CFG_IVT_ID_RESULT_WH"], "IVT_Msg_Result_Wh", "Wh", 1, "Wh", cfg["CFG_IVT_ENERGY_PERIOD_MS"]),
    ]
    for can_id, name, short, scale, unit, period in results:
        d.message(can_id, name, "IVT", False, period, dlc=6)        # Result frames are 6 bytes
        d.signal(f"IVT_{short}_MuxID", 0, 8, maximum=255, receivers="BMS")
        d.signal(f"IVT_{short}_Counter", 8, 4, maximum=15, receivers="BMS")
        d.signal(f"IVT_{short}_Status", 12, 4, maximum=15, receivers="BMS",
                 comment="bit0 OCS, bit1 result error/out of range, bit2 any result error, bit3 system error")
        d.signal(f"IVT_{short}", 23, 32, little_endian=False, signed=True, scale=scale,
                 minimum=-2147483648 * scale, maximum=2147483647 * scale, unit=unit, receivers="BMS")
    d.message(cfg["CFG_IVT_ID_COMMAND"], "IVT_Msg_Command", "BMS", False, None, "Commands to the IVT, sent at start-up")
    d.signal("IVT_Command_ID", 0, 8, maximum=255, receivers="IVT")
    d.signal("IVT_Command_Data", 15, 56, little_endian=False, maximum=72057594037927935, receivers="IVT")
    d.message(cfg["CFG_IVT_ID_RESPONSE"], "IVT_Msg_Response", "IVT", False, None, "Command responses (filtered out by the BMS)")
    d.signal("IVT_Response_ID", 0, 8, maximum=255, receivers="BMS")
    d.signal("IVT_Response_Data", 15, 56, little_endian=False, maximum=72057594037927935, receivers="BMS")
    return d


if __name__ == "__main__":
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import generate                     # noqa: E402  (this script used to write the DBCs on its own)
    sys.exit(generate.main([]))

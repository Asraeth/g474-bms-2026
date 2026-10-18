"""
Decodes the CAN traffic recorded by the can_layout test with the generated DBC files and checks the
values against what the simulation fed in. Catches any mismatch between can/*.dbc and the firmware.

    python test/check_dbc.py [test/build/can_layout.csv]
"""

import csv
import os
import re
import sys

import cantools

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "tools"))
from gen_dbc import read_config, CONFIG, settings_table  # noqa: E402
from generate import ivt_commands  # noqa: E402

failures = []


def cfg_pack_energy(cfg):
    # CFG_PACK_ENERGY_WH is a float expression; recompute it from the parsed values
    return cfg["CFG_CELL_ENERGY_WH"] * cfg["CFG_NUM_SLAVES"] * cfg["CFG_CELLS_PER_SLAVE"]


def check(condition, message):
    if not condition:
        failures.append(message)


def close(a, b, tol):
    return abs(a - b) <= tol


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else os.path.join(ROOT, "test", "build", "can_layout.csv")
    cfg = read_config(CONFIG)
    settings = settings_table(cfg)
    can1 = cantools.database.load_file(os.path.join(ROOT, "can", "bms_can1.dbc"), strict=True)
    can2 = cantools.database.load_file(os.path.join(ROOT, "can", "ivt_can2.dbc"), strict=True)
    can3 = cantools.database.load_file(os.path.join(ROOT, "can", "charger_can3.dbc"), strict=True)

    frames = []
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            frames.append({
                "t": float(row["time_s"]), "bus": int(row["bus"]), "id": int(row["id"], 16),
                "data": bytes.fromhex(row["data"]), "from_mcu": row["from_mcu"] == "1",
            })

    # Last complete CAN1 burst, and the fast frames sent during it
    fast_ids = {cfg["CFG_CAN_ID_FAST_1"], cfg["CFG_CAN_ID_FAST_2"]}
    limit_ids = {cfg["CFG_CAN_ID_INVERTER_DC_LIMIT"], cfg["CFG_CAN_ID_INVERTER_DC_BRAKE_LIMIT"], cfg["CFG_CAN_ID_LIMITS"]}
    event_ids = {cfg["CFG_CAN_ID_CONFIG_RESPONSE"]}       # Only sent on request
    starts = [i for i, f in enumerate(frames) if f["bus"] == 1 and f["from_mcu"] and f["id"] == cfg["CFG_CAN_ID_PACK"]]
    window = [f for f in frames[starts[-2]:starts[-1]] if f["bus"] == 1 and f["from_mcu"]]
    burst = [f for f in window if f["id"] not in fast_ids | limit_ids | event_ids]
    limits = [f for f in window if f["id"] in limit_ids]
    fast = [f for f in window if f["id"] in fast_ids]

    # Values the can_layout test injected
    slaves, cells, temps = cfg["CFG_NUM_SLAVES"], cfg["CFG_CELLS_PER_SLAVE"], cfg["CFG_TEMPS_PER_SLAVE"]
    cell_v = lambda s, c: 3.70 + s * 0.004 + c * 0.001
    temp_c = lambda s, t: 20.0 + s * 1.0 + t * 0.5
    pack_sum = sum(cell_v(s, c) for s in range(slaves) for c in range(cells))

    decoded = {}
    for f in burst:
        msg = can1.get_message_by_frame_id(f["id"] | 0x80000000)
        decoded[msg.name] = msg.decode(f["data"])
    check(len(decoded) == len(burst), "duplicate or unknown frames in the burst")
    expected_names = {m.name for m in can1.messages
                      if m.senders == ["BMS"] and m.frame_id & 0x1FFFFFFF not in fast_ids | limit_ids | event_ids}
    check(set(decoded) == expected_names, f"burst vs DBC: missing {sorted(expected_names - set(decoded))[:5]}, "
                                          f"extra {sorted(set(decoded) - expected_names)[:5]}")

    for s in range(slaves):
        seg = f"Seg{s + 1:02d}"
        for c in range(cells):
            name = f"BMS_{seg}_Voltages_{c // 4 + 1}"
            v = decoded.get(name, {}).get(f"{seg}_Cell{c + 1:02d}_Voltage")
            check(v is not None and close(v, cell_v(s, c), 0.002), f"{seg} cell {c + 1}: {v} vs {cell_v(s, c):.3f}")
        for t in range(temps):
            name = f"BMS_{seg}_Temps_{t // 4 + 1}"
            v = decoded.get(name, {}).get(f"{seg}_Temp{t + 1:02d}")
            check(v is not None and close(v, temp_c(s, t), 0.3), f"{seg} temp {t + 1}: {v} vs {temp_c(s, t)}")
        slave = decoded.get(f"BMS_{seg}", {})
        vseg = sum(cell_v(s, c) for c in range(cells))
        check(close(slave.get(f"{seg}_Voltage", -1), vseg, 0.05), f"{seg} voltage {slave.get(f'{seg}_Voltage')} vs {vseg:.2f}")
        check(close(slave.get(f"{seg}_Die_Temp", -1), 35.0 + s, 0.5), f"{seg} die temp {slave.get(f'{seg}_Die_Temp')}")
        check(slave.get(f"{seg}_Comms_Error") == 0 and slave.get(f"{seg}_Fault") == 0, f"{seg} flags set")

    pack = decoded.get("BMS_Pack", {})
    check(close(pack.get("Pack_Current", 0), 123.4, 0.15), f"pack current {pack.get('Pack_Current')}")
    check(close(pack.get("Pack_Voltage_IVT", 0), pack_sum, 0.2), f"pack U1 {pack.get('Pack_Voltage_IVT')} vs {pack_sum:.1f}")
    check(close(pack.get("Pack_Voltage_SumOfCells", 0), pack_sum, 0.2), f"sum of cells {pack.get('Pack_Voltage_SumOfCells')}")

    status = decoded.get("BMS_Status", {})
    check(status.get("Fault_Output_Active") == 0 and status.get("TSAC_Ready") == 1, f"status {status}")
    check(all(status.get(f"Fault_{n}") == 0 for n in ("Comms", "Voltage", "Temp", "Current", "OpenWire", "IVT", "Startup")),
          "fault flags set in BMS_Status")

    summary = decoded.get("BMS_Summary", {})
    check(close(summary.get("Cell_Voltage_Min", 0), cell_v(0, 0), 0.002), f"summary min {summary.get('Cell_Voltage_Min')}")
    check(close(summary.get("Cell_Voltage_Max", 0), cell_v(slaves - 1, cells - 1), 0.002), f"summary max {summary.get('Cell_Voltage_Max')}")

    check(status.get("Precharge_State") in (0, "Idle"), f"precharge state {status.get('Precharge_State')}")

    energy = decoded.get("BMS_Energy", {})
    expect_power = 123.4 * pack_sum
    check(close(energy.get("Power", 0), expect_power, 0.002 * expect_power + 20), f"power {energy.get('Power')} vs {expect_power:.0f}")
    check(0 < energy.get("SoC", 0) <= 100, f"SoC {energy.get('SoC')}")
    check(close(energy.get("Energy_Remaining", 0) / cfg_pack_energy(cfg) * 100, energy.get("SoC", 0), 0.2),
          f"remaining {energy.get('Energy_Remaining')} Wh vs SoC {energy.get('SoC')} %")
    check(energy.get("Energy_Used", 0) > 0, f"energy used {energy.get('Energy_Used')}")

    # Fast frames: one pair per CFG_CAN_FAST_PERIOD_MS, values as the IVT reports them
    per_burst = cfg["CFG_CAN_TX_PERIOD_MS"] // cfg["CFG_CAN_FAST_PERIOD_MS"]
    for fid in fast_ids:
        count = sum(1 for f in fast if f["id"] == fid)
        check(abs(count - per_burst) <= 1, f"0x{fid:X}: {count} frames per burst period, expected {per_burst}")
    fast_decoded = {}
    for f in fast:
        msg = can1.get_message_by_frame_id(f["id"] | 0x80000000)
        check(msg.cycle_time == cfg["CFG_CAN_FAST_PERIOD_MS"], f"{msg.name} cycle time {msg.cycle_time}")
        fast_decoded.update(msg.decode(f["data"]))
    check(close(fast_decoded.get("Fast_Pack_Current", 0), 123.4, 0.0005), f"fast current {fast_decoded.get('Fast_Pack_Current')}")
    check(close(fast_decoded.get("Fast_Pack_Voltage", 0), pack_sum, 0.002), f"fast U1 {fast_decoded.get('Fast_Pack_Voltage')} vs {pack_sum:.3f}")
    check(abs(fast_decoded.get("Fast_Vehicle_Voltage", 99)) < 1.0, f"fast U2 {fast_decoded.get('Fast_Vehicle_Voltage')} with no precharge")
    check(close(fast_decoded.get("Fast_Power", 0), 123.4 * pack_sum, 2.5), f"fast power {fast_decoded.get('Fast_Power')} vs {123.4 * pack_sum:.0f}")
    check(close(fast_decoded.get("Fast_Energy_Remaining", 0), energy.get("Energy_Remaining", -100), 3),
          f"fast remaining {fast_decoded.get('Fast_Energy_Remaining')} vs energy frame {energy.get('Energy_Remaining')}")

    # Inverter limits: discharge = min(max discharge, power / U1), regen = max charge (cool pack, no fault)
    per_burst_limits = cfg["CFG_CAN_TX_PERIOD_MS"] // cfg["CFG_INVERTER_LIMIT_PERIOD_MS"]
    for lid in limit_ids:
        count = sum(1 for f in limits if f["id"] == lid)
        check(abs(count - per_burst_limits) <= 1, f"0x{lid:X}: {count} limit frames per burst period, expected {per_burst_limits}")
    limit_decoded = {}
    for f in limits:
        if f["id"] != cfg["CFG_CAN_ID_LIMITS"]:
            check(f["data"][2:] == b"\xff" * 6, f"0x{f['id']:X}: unused bytes {f['data'][2:].hex()}, expected FF")
        limit_decoded.update(can1.get_message_by_frame_id(f["id"] | 0x80000000).decode(f["data"]))
    # Current for the power limit, as bms_inverter.c works it out: the pack voltage sags above the present
    # current (at least CFG_PACK_RESISTANCE_MIN_OHM) and recovers below it (at most _MAX_OHM)
    i_now, u, p_lim, i_max = 123.4, pack_sum, cfg["CFG_INVERTER_POWER_LIMIT_W"], cfg["CFG_MAX_DISCHARGE_CURRENT_A"]
    i0 = p_lim / u
    r = cfg["CFG_PACK_RESISTANCE_MAX_OHM"] if i0 < i_now else cfg["CFG_PACK_RESISTANCE_MIN_OHM"]
    b = u + r * i_now
    i_power = (b - (b * b - 4 * r * p_lim) ** 0.5) / (2 * r) if r > 0 else i0
    power_limit = min(i_max, i_power)
    power_limited = i_power < i_max
    expect_power_w = p_lim if power_limited else i_max * (u - cfg["CFG_PACK_RESISTANCE_MIN_OHM"] * (i_max - i_now))
    check(close(limit_decoded.get("Inverter_Max_DC_Current", 0), power_limit, 0.15),
          f"inverter DC limit {limit_decoded.get('Inverter_Max_DC_Current')} A, expected {power_limit:.1f} A")
    check(close(limit_decoded.get("Inverter_Max_DC_Brake_Current", 0), -cfg["CFG_INVERTER_REGEN_CURRENT_A"], 0.05),
          f"inverter brake limit {limit_decoded.get('Inverter_Max_DC_Brake_Current')} A")
    check(status.get("Inverter_Power_Limited") == int(power_limited) and status.get("Inverter_Temp_Derating") == 0,
          f"inverter status bits {status}")
    check(close(limit_decoded.get("Discharge_Power_Limit", 0), expect_power_w, 0.002 * expect_power_w + 5),
          f"power limit {limit_decoded.get('Discharge_Power_Limit')} W, expected {expect_power_w:.0f} W")
    check(close(limit_decoded.get("Discharge_Current_Limit", 0), power_limit, 0.15) and
          close(limit_decoded.get("Regen_Current_Limit", 0), cfg["CFG_INVERTER_REGEN_CURRENT_A"], 0.05) and
          limit_decoded.get("Limit_Power") == int(power_limited) and limit_decoded.get("Limit_Temp_Derating") == 0 and
          limit_decoded.get("Limit_Regen_Temp_Derating") == 0 and limit_decoded.get("Limit_AMS_Fault") == 0,
          f"BMS_Limits {limit_decoded}")

    # Charger command on CAN3
    charger_frames = [f for f in frames if f["bus"] == 3 and f["from_mcu"]]
    check(len(charger_frames) > 0, "no charger frames on CAN3")
    # Current charger gateway: separate six-digit BCD setpoint and fault heartbeat.
    targets=[f for f in charger_frames if f['id']==0x00FFFFFF]
    faults=[f for f in charger_frames if f['id']==0x001FFFFF]
    check(bool(targets) and bool(faults),'missing charger setpoint or fault heartbeat')
    def bcd(data):
        digits=[n for byte in data for n in (byte>>4,byte&15)]
        check(all(n<=9 for n in digits),'non-decimal charger BCD digit')
        return sum(n*10**(len(digits)-i-1) for i,n in enumerate(digits))
    if targets:
        data=targets[-1]['data']
        target_v=next(s for s in settings if s['name']=='Charge_Voltage')['default']/10
        check(close(bcd(data[:3])/1000,target_v,0.001),'charger BCD voltage mismatch')
        check(close(bcd(data[3:6])/100000,0,0.00001),'charger BCD current mismatch')
        check(data[6:]==bytes(2),'charger setpoint reserved bytes')
    if faults:check(faults[-1]['data']==bytes.fromhex('AA00000000000000'),'charger must be inhibited without deliberate enable')

    # Settings: the can_layout test reads all settings, then writes a power limit above the built-in one
    request_msg = can1.get_message_by_name("BMS_Config_Request")
    response_msg = can1.get_message_by_name("BMS_Config_Response")
    requests = [request_msg.decode(f["data"]) for f in frames
                if f["bus"] == 1 and not f["from_mcu"] and f["id"] == cfg["CFG_CAN_ID_CONFIG_REQUEST"]]
    responses = [response_msg.decode(f["data"]) for f in frames
                 if f["bus"] == 1 and f["from_mcu"] and f["id"] == cfg["CFG_CAN_ID_CONFIG_RESPONSE"]]
    check([r["Config_Command"] for r in requests] == ["Read_All", "Write"], f"settings requests {requests}")
    check(len(responses) == len(settings) + 1, f"{len(responses)} settings responses, expected {len(settings) + 1}")
    for s, r in zip(settings, responses):
        value = r.get(f"Setting_{s['name']}")
        check(r["Config_Command"] == "Read_All" and r["Config_Setting"] == s["name"] and r["Config_Result"] == "OK" and
              value is not None and close(value, s["default"] * s["scale"], 1e-6) and
              (r["Config_Unsaved"], r["Config_Modified"], r["Config_Stored"]) == (0, 0, 0),
              f"setting {s['name']}: {r}, expected the built-in {s['default'] * s['scale']:g}")
    if len(responses) > len(settings):
        r = responses[len(settings)]
        power = settings[0]
        check(r["Config_Command"] == "Write" and r["Config_Setting"] == power["name"] and
              r["Config_Result"] == "Out_Of_Range" and close(r.get("Setting_Power_Limit", 0), power["default"], 1e-6),
              f"power limit above the built-in value: {r}")

    # IVT frames on CAN2 decoded with ivt_can2.dbc
    ivt = {}
    ivt_ids = [cfg[k] for k in ("CFG_IVT_ID_RESULT_I", "CFG_IVT_ID_RESULT_U1", "CFG_IVT_ID_RESULT_U2", "CFG_IVT_ID_RESULT_WH")]
    for f in frames:
        if f["bus"] == 2 and not f["from_mcu"] and f["id"] in ivt_ids:
            msg = can2.get_message_by_frame_id(f["id"])
            ivt.update(msg.decode(f["data"]))
    check(close(ivt.get("IVT_I", 0), 123.4, 0.01), f"IVT current {ivt.get('IVT_I')}")
    check(close(ivt.get("IVT_U1", 0), pack_sum, 0.01), f"IVT U1 {ivt.get('IVT_U1')}")
    check("IVT_U2" in ivt, "no IVT U2 frames")
    check("IVT_Wh" in ivt, "no IVT energy frames")
    check(not any(f["bus"] == 2 and not f["from_mcu"] and f["id"] not in ivt_ids + [cfg["CFG_IVT_ID_RESPONSE"]]
                  and f["t"] > 2.0 for f in frames), "IVT sends results the BMS switched off")
    commands = [f for f in frames if f["bus"] == 2 and f["from_mcu"] and f["id"] == cfg["CFG_IVT_ID_COMMAND"]]
    expected = [bytes(c) for c, _ in ivt_commands(cfg)]
    check([f["data"] for f in commands] == expected,
          f"IVT commands sent {[f['data'].hex(' ') for f in commands]} differ from the documented {[c.hex(' ') for c in expected]}")

    print(f"decoded {len(burst)} BMS frames, {len(fast)} fast frames, {len(responses)} settings responses and "
          f"{sum(1 for f in frames if f['bus'] == 2)} CAN2 frames with the DBC files")
    for f in failures:
        print("  FAIL", f)
    print("DBC check:", "PASS" if not failures else f"{len(failures)} failures")
    return 0 if not failures else 1


if __name__ == "__main__":
    sys.exit(main())

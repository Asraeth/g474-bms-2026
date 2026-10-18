"""IVT bus (CAN2) stress tests against the BMS on the bench (hardware in the loop).

    python test/bench/ivt_bus_stress.py --channel COM8 --uart COM7 [--interface seeedstudio] [--list] [test ...]

Needs the BMS running its normal firmware, its UART (the ST-LINK's virtual COM port, 1 Mbaud) and a python-can
adapter on the IVT bus next to the IVT-S (500 kbit/s). Each test injects something on CAN2 (floods, junk on the
IVT's IDs, spurious alive messages, stray commands, an IVT restart) and checks the BMS's reaction from its UART
log ('Fault <tick> ms: ...' lines, timed by the BMS itself) and from the bus. The spoof_ tests and the soak stand
in for the IVT: they stop it and send its results themselves, with values a bench can't produce (overcurrent,
pack over/undervoltage, sensor errors, current noise around the limit).

With no slaves connected the BMS sits in STARTUP/VOLTAGE faults; the tests only look at the IVT, current and
pack voltage flags. When the IVT reports a system error (status 8: a restart command sets it on the team's
unit), the script clears it with Reset Error- and Logdata, which also wipes the IVT's error counters.
Requirements: GUI/requirements.txt (python-can, pyserial).
"""
import argparse
import re
import sys
import threading
import time

import can
import serial

ERR = {"COMMS": 0x01, "VOLTAGE": 0x02, "TEMP": 0x04, "CURRENT": 0x08, "OPEN_WIRE": 0x10, "IVT": 0x20, "STARTUP": 0x40}
IVT_CMD, IVT_RESP = 0x411, 0x511
IVT_I, IVT_U1, IVT_U2, IVT_WH = 0x521, 0x522, 0x523, 0x528
STOP = [0x34, 0x00, 0x01, 0, 0, 0, 0, 0]
RUN = [0x34, 0x01, 0x01, 0, 0, 0, 0, 0]
FAULT_RE = re.compile(r"Fault (\d+) ms: (?:0x([0-9A-F]+) -> 0x([0-9A-F]+))?")
RUNTIME_RE = re.compile(r"Runtime: (\d+) ms")
now = time.perf_counter                     # time.monotonic() only ticks every ~16 ms on Windows


class Bench:
    def __init__(self, interface, channel, bitrate, uart):
        self.kwargs = dict(interface=interface, channel=channel, bitrate=bitrate)
        if interface == "seeedstudio":
            self.kwargs.update(frame_type="STD", operation_mode="normal")
        self.bus = can.ThreadSafeBus(**self.kwargs)
        self.paused = False
        self.last_rx = now()
        self.reopens = 0
        self.uart = serial.Serial(uart, 1000000, timeout=0.05)
        self.lines = []             # (host time, line)
        self.frames = []            # (host time, can.Message)
        self.offset = None          # host s - BMS tick s, smallest seen (least latency)
        self.flags = None
        self.rx_errors, self.last_rx_error = 0, None
        self.stop = threading.Event()
        self.lock = threading.Lock()
        threading.Thread(target=self._read_uart, daemon=True).start()
        threading.Thread(target=self._read_can, daemon=True).start()
        time.sleep(0.5)             # The adapter drops frames sent right after it opens

    def _read_uart(self):
        buf = b""
        while not self.stop.is_set():
            buf += self.uart.read(4096)
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                t = now()
                line = raw.decode(errors="replace").strip()
                m = RUNTIME_RE.search(line) or FAULT_RE.match(line)
                if m:
                    off = t - int(m.group(1)) / 1000
                    self.offset = off if self.offset is None else min(self.offset, off)
                f = FAULT_RE.match(line)
                if f and f.group(3):
                    self.flags = int(f.group(3), 16)
                with self.lock:
                    self.lines.append((t, line))

    def _read_can(self):
        while not self.stop.is_set():
            if self.paused:
                time.sleep(0.01)
                continue
            try:
                m = self.bus.recv(timeout=0.05)
            except Exception as e:          # A garbled frame from the adapter mustn't end the reader
                self.rx_errors += 1
                self.last_rx_error = repr(e)
                continue
            if m is not None:
                self.last_rx = now()
                with self.lock:
                    self.frames.append((now(), m))

    def reopen(self):
        """The adapter's receive stream can stall after a flood (its USB link was saturated): start it afresh"""
        self.paused = True
        time.sleep(0.1)
        try:
            self.bus.shutdown()
        except Exception:
            pass
        self.bus = can.ThreadSafeBus(**self.kwargs)
        self.reopens += 1
        self.paused = False
        time.sleep(0.5)

    def check_rx(self):
        """Reopens the adapter if it has been silent although the IVT should be talking"""
        if now() - self.last_rx > 0.5:
            self.reopen()

    def close(self):
        self.stop.set()
        time.sleep(0.15)
        self.bus.shutdown()
        self.uart.close()

    def send(self, ident, data, ext=False, remote=False, dlc=None):
        self.bus.send(can.Message(arbitration_id=ident, data=bytes(data), is_extended_id=ext, is_remote_frame=remote,
                                  dlc=len(data) if dlc is None else dlc))

    def mark(self):
        with self.lock:
            return len(self.lines), len(self.frames), now()

    def lines_since(self, mark):
        with self.lock:
            return list(self.lines[mark[0]:])

    def frames_since(self, mark):
        with self.lock:
            return list(self.frames[mark[1]:])

    def faults_since(self, mark):
        """[(host time of the BMS tick, old flags, new flags)] of fault flag changes since mark"""
        out = []
        for t, line in self.lines_since(mark):
            f = FAULT_RE.match(line)
            if f and f.group(3):
                tick = int(f.group(1)) / 1000
                out.append((tick + self.offset if self.offset is not None else t, int(f.group(2), 16), int(f.group(3), 16)))
        return out

    def new_faults(self, mark, until=None):
        """{flag name: host time it first came on} since mark"""
        new = {}
        for t, old, flags in self.faults_since(mark):
            if until is not None and t > until:
                break
            for name, bit in ERR.items():
                if flags & bit and not old & bit and name not in new:
                    new[name] = t
        return new

    def resets_since(self, mark):
        return [line for _, line in self.lines_since(mark) if "PROGRAM START" in line]

    def bms_commands(self, mark, first=None):
        return [(t, m) for t, m in self.frames_since(mark)
                if m.arbitration_id == IVT_CMD and (first is None or bytes(m.data[:len(first)]) == bytes(first))]

    def rates(self, mark, seconds):
        counts = {}
        for t, m in self.frames_since(mark):
            if t >= mark[2] + seconds:
                break
            counts[m.arbitration_id] = counts.get(m.arbitration_id, 0) + 1
        return {k: v / seconds for k, v in counts.items()}

    def ivt_status(self):
        with self.lock:
            for t, m in reversed(self.frames[-200:]):
                if m.arbitration_id == IVT_I and len(m.data) >= 2:
                    return m.data[1] >> 4
        return None

    def wait_frame(self, pred, timeout, mark=None):
        mark = mark or self.mark()
        end = now() + timeout
        while now() < end:
            for t, m in self.frames_since(mark):
                if pred(m):
                    return t, m
            time.sleep(0.002)
        return None

    def wait_healthy(self, timeout=6.0):
        """IVT data at the BMS's rates and no IVT or current fault. Returns seconds taken or None."""
        start = now()
        while now() - start < timeout:
            mark = self.mark()
            time.sleep(0.5)
            r = self.rates(mark, 0.5)
            flags = self.flags if self.flags is not None else 0
            if r.get(IVT_I, 0) > 150 and r.get(IVT_U1, 0) > 150 and not flags & (ERR["IVT"] | ERR["CURRENT"]):
                return now() - start
        return None


def ivt_frame(mux, value, counter=0, status=0):
    v = value & 0xFFFFFFFF
    return [mux, (status << 4) | (counter & 0x0F), (v >> 24) & 0xFF, (v >> 16) & 0xFF, (v >> 8) & 0xFF, v & 0xFF]


NOMINAL = dict(current_mA=10_000, u1_mV=400_000, u2_mV=399_000, status=0)


class Impersonator:
    """Stops the real IVT and sends I, U1, U2 (every 4 ms) and Wh (100 ms) in its place from profile(t) -> dict
    (current_mA, u1_mV, u2_mV, status; a value of None isn't sent). hook(t) runs every pass, False ends early.
    keep_stopped: stop the real IVT again whenever the BMS starts it."""

    def __init__(self, bench, profile, hook=None, keep_stopped=False):
        self.b, self.profile, self.hook, self.keep_stopped = bench, profile, hook, keep_stopped
        self.sent, self.late, self.worst, self.restops = 0, 0, 0.0, 0

    def run(self, seconds):
        b = self.b
        counters = [0] * 8
        b.send(IVT_CMD, STOP)
        mark = b.mark()
        start = now()
        next_fast, next_wh = start, start
        p = self.profile(0.0)
        while True:
            t_now = now()
            t = t_now - start
            if t >= seconds or (self.hook and self.hook(t) is False):
                break
            if t_now >= next_fast:
                self.worst = max(self.worst, t_now - next_fast)
                p = self.profile(t)
                for mux, ident, key in ((0, IVT_I, "current_mA"), (1, IVT_U1, "u1_mV"), (2, IVT_U2, "u2_mV")):
                    if p.get(key) is not None:
                        b.send(ident, ivt_frame(mux, int(p[key]), counters[mux], p.get("status", 0)))
                        counters[mux] += 1
                        self.sent += 1
                next_fast += 0.004
                if t_now - next_fast > 0.008:       # 2 periods: the BMS starts to see a slow IVT at 9 ms
                    self.late += 1
                    next_fast = t_now + 0.004
            if t_now >= next_wh:
                b.send(IVT_WH, ivt_frame(7, 0, counters[7], p.get("status", 0)))
                counters[7] += 1
                next_wh += 0.1
            if self.keep_stopped and b.bms_commands(mark, RUN[:2]):
                b.send(IVT_CMD, STOP)
                self.restops += 1
                mark = b.mark()
            time.sleep(0)
        return start


def clear_ivt_errors(b):
    """Reset Error- and Logdata (system errors, all counters): stop mode only, answered after ~0.8 s. The test
    stands in for the IVT meanwhile so the BMS doesn't reconfigure it half way."""
    state = {"serial": None, "sent": False, "done": None}
    mark = b.mark()

    def hook(t):
        if state["serial"] is None and t > 0.02:
            if not state["sent"]:
                b.send(IVT_CMD, [0x7B, 0, 0, 0, 0, 0, 0, 0])
                state["sent"] = True
            for _, m in b.frames_since(mark):
                if m.arbitration_id == IVT_RESP and m.data[0] == 0xBB:
                    state["serial"] = bytes(m.data[1:5])
                    b.send(IVT_CMD, [0x30, 0x01, 0x00] + list(state["serial"]))
            if t > 0.5 and state["serial"] is None:
                return False
        elif state["serial"] is not None:
            for _, m in b.frames_since(mark):
                if m.arbitration_id == IVT_RESP and m.data[0] == 0xB0:
                    state["done"] = t
                    return False
        return True

    Impersonator(b, lambda t: NOMINAL, hook=hook).run(3.0)
    b.send(IVT_CMD, RUN)
    return state["done"] is not None


# ---------------------------------------------------------------------------------------------------------------
# Tests: each returns (passed, notes)

def t_baseline(b):
    """The IVT at the BMS's rates, no new faults"""
    mark = b.mark()
    time.sleep(3.0)
    r = b.rates(mark, 3.0)
    ok = r.get(IVT_I, 0) > 200 and not b.resets_since(mark) and not b.new_faults(mark)
    return ok, f"IVT I {r.get(IVT_I, 0):.0f}/s U1 {r.get(IVT_U1, 0):.0f}/s U2 {r.get(IVT_U2, 0):.0f}/s " \
               f"Wh {r.get(IVT_WH, 0):.0f}/s, IVT status {b.ivt_status()}, BMS flags 0x{b.flags or 0:02X}"


def flood(b, ident, seconds, rate, dlc=8):
    """Paced, so the adapter doesn't buffer what the bus can't carry. The adapter's own reception is unreliable
    while it floods (and can stall), so it's reopened afterwards."""
    mark = b.mark()
    start = now()
    sent = 0
    while now() - start < seconds:
        while sent < (now() - start) * rate:
            b.send(ident, [0x55] * dlc)
            sent += 1
        time.sleep(0)
    time.sleep(0.2)
    b.reopen()
    return mark, sent / seconds


def ivt_gap(b, mark, seconds):
    last, worst = None, 0.0
    for t, m in b.frames_since(mark):
        if t > mark[2] + seconds:
            break
        if m.arbitration_id == IVT_I:
            if last is not None:
                worst = max(worst, t - last)
            last = t
    return worst


def t_flood_low_priority(b):
    """Bus full of the lowest-priority ID: the IVT's frames still win arbitration"""
    mark, rate = flood(b, 0x7FF, 5.0, 4000)
    time.sleep(0.3)
    new = b.new_faults(mark)
    cmds = b.bms_commands(mark)
    ok = "IVT" not in new and not cmds and not b.resets_since(mark)
    return ok, f"{rate:.0f} frames/s of 0x7FF; IVT I {b.rates(mark, 5.0).get(IVT_I, 0):.0f}/s, longest IVT gap " \
               f"{ivt_gap(b, mark, 5.0) * 1000:.0f} ms; new faults {sorted(new)}, BMS commands {len(cmds)}"


def t_flood_high_priority(b):
    """Bus (nearly) full of the highest-priority ID. If the IVT is starved the BMS must fault within the deadline,
    and either way recover by itself. (The adapter can't quite fill the bus, and can't report what it receives
    while flooding, so whether the IVT was starved shows only in the BMS's reaction.)"""
    mark, rate = flood(b, 0x001, 3.0, 4500)
    new = b.new_faults(mark)
    rec = b.wait_healthy(8.0)
    react = (new["IVT"] - mark[2]) * 1000 if "IVT" in new else None
    ok = not b.resets_since(mark) and rec is not None and (react is None or react <= 480) and "CURRENT" not in new
    starved = "%.0f ms after the flood started" % react if react is not None else "none (not starved)"
    return ok, f"{rate:.0f} frames/s of 0x001; IVT fault {starved}, healthy again after "                f"{rec if rec is None else '%.1f s' % rec}"


def t_flood_high_priority_partial(b):
    """High-priority traffic leaving gaps: the IVT's frames get through late, nothing should fault"""
    mark, rate = flood(b, 0x001, 5.0, 2500)
    time.sleep(0.3)
    new = b.new_faults(mark)
    cfgs = b.bms_commands(mark, RUN[:2])
    rec = b.wait_healthy()
    ok = "IVT" not in new and "CURRENT" not in new and rec is not None and not b.resets_since(mark)
    return ok, f"{rate:.0f} frames/s of 0x001; IVT I {b.rates(mark, 5.0).get(IVT_I, 0):.0f}/s, longest IVT gap " \
               f"{ivt_gap(b, mark, 5.0) * 1000:.0f} ms; new faults {sorted(new)}, BMS reconfigured {len(cfgs)} times"


def t_junk_frames(b):
    """Frames on the IVT's IDs that must be ignored: wrong mux, short, remote, extended, non-alive responses"""
    mark = b.mark()
    sent = 0
    start = now()
    while now() - start < 5.0:
        k = sent % 7
        if k == 0:
            b.send(IVT_I, [0x05, 0x00, 0x7F, 0xFF, 0xFF, 0xFF])          # Wrong mux, huge value
        elif k == 1:
            b.send(IVT_I, [0x00, 0x00, 0x7F, 0xFF, 0xFF])                # Right mux, 5 bytes
        elif k == 2:
            b.send(IVT_I, [], remote=True, dlc=6)                          # Remote frame
        elif k == 3:
            b.send(IVT_I, [0x00, 0x00, 0x7F, 0xFF, 0xFF, 0xFF], ext=True)  # Extended ID 0x521
        elif k == 4:
            b.send(IVT_RESP, [0xBE, 0, 0, 0, 0, 0, 0, 0])                  # Not the alive message
        elif k == 5:
            b.send(IVT_RESP, [0xBF, 0, 0])                                 # Alive, but 3 bytes
        else:
            b.send(IVT_U1, [0x02, 0x00, 0x00, 0x00, 0x00, 0x00])          # U2's mux on U1's ID
        sent += 1
        while sent > (now() - start) * 2000:
            time.sleep(0)
    time.sleep(0.3)
    new = b.new_faults(mark)
    cmds = b.bms_commands(mark)
    ok = "IVT" not in new and "CURRENT" not in new and not cmds and not b.resets_since(mark)
    return ok, f"{sent} junk frames; new faults {sorted(new)}, BMS commands to the IVT {len(cmds)}"


def t_alive_spam(b):
    """Spurious alive messages while the IVT runs: the BMS must not stop it to reconfigure"""
    mark = b.mark()
    for _ in range(40):
        b.send(IVT_RESP, [0xBF, 0x04, 0x11, 0x00, 0x99, 0x47, 0x0E, 0x00])
        time.sleep(0.05)
    time.sleep(0.5)
    cfgs = b.bms_commands(mark, RUN[:2])
    ignored = sum("alive message while its data is arriving" in l for _, l in b.lines_since(mark))
    new = b.new_faults(mark)
    ok = not cfgs and "IVT" not in new and not b.resets_since(mark)
    return ok, f"40 alive messages in 2 s: BMS reconfigured the IVT {len(cfgs)} times, logged {ignored} as ignored, " \
               f"longest IVT gap {ivt_gap(b, mark, 2.5) * 1000:.0f} ms"


def t_foreign_results(b):
    """Results the BMS switched off: it reconfigures, at most every CFG_IVT_RECONFIG_INTERVAL_MS"""
    mark = b.mark()
    for i in range(100):
        b.send(0x524 + (i % 4), ivt_frame(3 + (i % 4), 0))
        time.sleep(0.03)
    time.sleep(0.5)
    cfgs = b.bms_commands(mark, RUN[:2])
    rec = b.wait_healthy()
    ok = rec is not None and 1 <= len(cfgs) <= 3 and not b.resets_since(mark)
    return ok, f"100 foreign result frames over 3 s: BMS reconfigured {len(cfgs)} times"


def stray_stop(b, after_config):
    if after_config:
        b.send(IVT_CMD, STOP)
        if b.wait_frame(lambda m: m.arbitration_id == IVT_CMD and bytes(m.data[:2]) == bytes(RUN[:2]), 4.0) is None:
            return None, None, None
        time.sleep(0.003)
    mark = b.mark()
    b.send(IVT_CMD, STOP)
    back = b.wait_frame(lambda m: m.arbitration_id == IVT_CMD and bytes(m.data[:2]) == bytes(RUN[:2]), 4.0, mark)
    b.wait_healthy()
    new = b.new_faults(mark)
    return mark, (back[0] - mark[2]) * 1000 if back else None, new


def t_stray_stop(b):
    """Another node stops the IVT: the BMS waits CFG_IVT_RESTART_WAIT_MS for an alive message, then restarts it, before
    the data counts as lost"""
    mark, back, new = stray_stop(b, False)
    ok = back is not None and back < 520 and not b.resets_since(mark)
    return ok, f"BMS restarted the IVT after {'%.0f ms' % back if back is not None else 'never'}; new faults {sorted(new)}"


def t_stray_stop_after_config(b):
    """... right after the BMS configured it: the retry interval (CFG_IVT_RETRY_INTERVAL_MS) decides"""
    mark, back, new = stray_stop(b, True)
    if mark is None:
        return False, "BMS didn't reconfigure the IVT"
    react = (new["IVT"] - mark[2]) * 1000 if "IVT" in new else None
    ok = back is not None and back < 650 and (react is None or react <= 480) and not b.resets_since(mark)
    return ok, f"BMS restarted the IVT after {'%.0f ms' % back if back is not None else 'never'}; IVT fault "                f"{'%.0f ms' % react if react else 'none'}"


def t_restart_command(b):
    """IVT RESTART: the BMS rides through the gap and reconfigures it on its alive message. The team's IVT comes
    back with a system error (status 8, a 'watchdog reset'), so the BMS then faults until it's cleared."""
    mark = b.mark()
    b.send(IVT_CMD, [0x3F, 0, 0, 0, 0, 0, 0, 0])
    alive = b.wait_frame(lambda m: m.arbitration_id == IVT_RESP and m.data[0] == 0xBF, 1.0, mark)
    run = b.wait_frame(lambda m: m.arbitration_id == IVT_CMD and bytes(m.data[:2]) == bytes(RUN[:2]), 1.0, mark)
    time.sleep(1.0)
    gap = ivt_gap(b, mark, 1.0)
    status = b.ivt_status()
    new = b.new_faults(mark)
    react = (new["IVT"] - mark[2]) * 1000 if "IVT" in new else None
    gap_fault = react is not None and react < gap * 1000
    cleared = clear_ivt_errors(b)
    rec = b.wait_healthy(8.0)
    ok = alive is not None and run is not None and not gap_fault and rec is not None and not b.resets_since(mark)
    return ok, f"alive after {(alive[0] - mark[2]) * 1000 if alive else float('nan'):.0f} ms, BMS configured it after " \
               f"{(run[0] - mark[2]) * 1000 if run else float('nan'):.0f} ms, data gap {gap * 1000:.0f} ms; IVT status " \
               f"{status} after; IVT fault {'%.0f ms' % react if react else 'none'}; errors cleared {cleared}, " \
               f"healthy after {rec if rec is None else '%.1f s' % rec}"


def impersonate_test(b, profile, seconds, expect, what, keep_stopped=False):
    imp = Impersonator(b, profile, keep_stopped=keep_stopped)
    mark = b.mark()
    t0 = imp.run(seconds)
    t1 = now()
    time.sleep(0.2)
    new = {k: (v - t0) * 1000 for k, v in b.new_faults(mark, until=t1 + 0.05).items()}
    rec = b.wait_healthy(8.0)
    ok = expect(new) and rec is not None and not b.resets_since(mark) and imp.late == 0
    desc = ", ".join(f"{k} at {v:.0f} ms" for k, v in sorted(new.items(), key=lambda kv: kv[1])) or "none"
    return ok, f"{what}: new faults {desc}; {imp.sent} frames, worst send lateness {imp.worst * 1000:.1f} ms" \
               f"{', IVT re-stopped %d times' % imp.restops if keep_stopped else ''}"


def t_spoof_nominal(b):
    """Standing in for the IVT with healthy values: no faults (checks the stand-in itself)"""
    return impersonate_test(b, lambda t: NOMINAL, 3.0, lambda new: "CURRENT" not in new and "IVT" not in new,
                            "400 V, 10 A")


def t_spoof_overcurrent_step(b):
    """250 A (limit 200 A): current fault after the debounce"""
    prof = lambda t: dict(NOMINAL, current_mA=250_000 if t >= 1.0 else 10_000)
    return impersonate_test(b, prof, 2.0, lambda new: "CURRENT" in new and new["CURRENT"] < 1000 + 250,
                            "250 A from 1.0 s")


def t_spoof_overcurrent_flapping(b):
    """2/3 of the samples just over the limit, 1/3 just under (noise around the limit): must trip"""
    def prof(t):
        over = t >= 1.0 and int(t / 0.004) % 3 != 0
        return dict(NOMINAL, current_mA=205_000 if over else 195_000 if t >= 1.0 else 10_000)
    return impersonate_test(b, prof, 2.5, lambda new: "CURRENT" in new and new["CURRENT"] < 1000 + 500,
                            "2/3 of samples at 205 A, 1/3 at 195 A from 1.0 s")


def t_spoof_overcurrent_pulses(b):
    """150 ms over the limit, 20 ms under, repeated: over the limit 88 % of the time, must trip"""
    def prof(t):
        over = t >= 1.0 and ((t - 1.0) % 0.17) < 0.15
        return dict(NOMINAL, current_mA=300_000 if over else 100_000)
    return impersonate_test(b, prof, 3.0, lambda new: "CURRENT" in new and new["CURRENT"] < 1000 + 500,
                            "300 A for 150 ms / 100 A for 20 ms from 1.0 s")


def t_spoof_single_spikes(b):
    """One 4 ms sample at 400 A every 100 ms: glitches, must not trip"""
    prof = lambda t: dict(NOMINAL, current_mA=400_000 if int(t / 0.004) % 25 == 0 else 50_000)
    return impersonate_test(b, prof, 3.0, lambda new: "CURRENT" not in new, "one 400 A sample every 100 ms")


def t_spoof_sensor_error(b):
    """IVT status 8 (system error): IVT fault after the debounce"""
    prof = lambda t: dict(NOMINAL, status=0x8 if t >= 1.0 else 0)
    return impersonate_test(b, prof, 2.0, lambda new: "IVT" in new and new["IVT"] < 1000 + 250,
                            "status 8 (system error) from 1.0 s")


def t_spoof_pack_overvoltage(b):
    """U1 over the pack maximum: voltage fault after the debounce"""
    prof = lambda t: dict(NOMINAL, u1_mV=480_000 if t >= 1.0 else 400_000)
    return impersonate_test(b, prof, 2.0, lambda new: "VOLTAGE" in new and new["VOLTAGE"] < 1000 + 250,
                            "U1 480 V from 1.0 s")


def t_spoof_current_stops(b):
    """Current frames stop while U1/U2 continue (and the real IVT is kept stopped): IVT fault by the deadline"""
    prof = lambda t: dict(NOMINAL, current_mA=None if t >= 1.0 else 10_000)
    return impersonate_test(b, prof, 2.0, lambda new: "IVT" in new and 1000 + 440 <= new["IVT"] <= 1000 + 480,
                            "current frames stop at 1.0 s", keep_stopped=True)


def t_rx_load(b):
    """As many frames as the adapter can send on IDs the BMS has to take in (every one an RX interrupt):
    no resets, no faults, the main loop keeps its period"""
    mark = b.mark()
    start = now()
    sent = 0
    while now() - start < 5.0:
        b.send(IVT_RESP, [0xBE, sent & 0xFF, 0, 0, 0, 0, 0, 0])
        sent += 1
        while sent > (now() - start) * 4500:
            time.sleep(0)
    time.sleep(0.3)
    b.reopen()
    new = b.new_faults(mark)
    loops = [int(m.group(1)) for _, l in b.lines_since(mark) for m in [re.search(r"LoopTime: (\d+) ms", l)] if m]
    warnings = [l for _, l in b.lines_since(mark)                  # (PEC errors: no slaves on the bench)
                if ("WARNING" in l or "overrun" in l or "Error" in l) and "PEC ERROR" not in l]
    ok = "IVT" not in new and "CURRENT" not in new and not b.resets_since(mark) and not warnings
    return ok, f"{sent / 5.0:.0f} frames/s on 0x511; new faults {sorted(new)}, loop time {min(loops, default=0)}.." \
               f"{max(loops, default=0)} ms, warnings {len(warnings)}" + (f" {sorted(set(warnings))[:3]}" if warnings else "")


SOAK_S = 180.0


def t_soak(b, seconds=None, seed=1):
    """Minutes of drifting healthy values with harmless glitches and real faults of each kind at random times
    (the bench version of the simulator's soak): every real fault must trip within 500 ms, nothing else may"""
    import random
    seconds = seconds or SOAK_S
    rnd = random.Random(seed)
    real_kinds = {"overcurrent": "CURRENT", "overcurrent_charge": "CURRENT", "flapping": "CURRENT",
                  "pulses": "CURRENT", "pack_ov": "VOLTAGE", "pack_uv": "VOLTAGE", "sensor_error": "IVT",
                  "current_lost": "IVT"}
    glitch_kinds = ["spike_sample", "spike_30ms", "status_sample", "u1_sample"]
    windows, glitches = [], []
    t = 2.0
    while t < seconds - 4.0:
        if rnd.random() < 0.5:
            kind = rnd.choice(list(real_kinds))
            length = rnd.uniform(0.6, 1.5)
            windows.append((t, t + length, kind))
            t += length + rnd.uniform(2.5, 4.0)        # Room for the 1 s clear delay
        else:
            kind = rnd.choice(glitch_kinds)
            glitches.append((t, kind))
            t += rnd.uniform(0.4, 1.5)
    drift = {"i": 50_000.0, "u": 400_000.0}

    def prof(tt):
        drift["i"] = min(150_000.0, max(-50_000.0, drift["i"] + rnd.uniform(-800, 800)))
        drift["u"] = min(420_000.0, max(380_000.0, drift["u"] + rnd.uniform(-100, 100)))
        p = dict(current_mA=drift["i"], u1_mV=drift["u"], u2_mV=drift["u"] - 500, status=0)
        for s0, s1, kind in windows:
            if s0 <= tt < s1:
                d = tt - s0
                if kind == "overcurrent":
                    p["current_mA"] = 260_000
                elif kind == "overcurrent_charge":
                    p["current_mA"] = -260_000
                elif kind == "flapping":
                    p["current_mA"] = 206_000 if int(tt / 0.004) % 3 else 196_000
                elif kind == "pulses":
                    p["current_mA"] = 300_000 if (d % 0.17) < 0.15 else 100_000
                elif kind == "pack_ov":
                    p["u1_mV"] = 480_000
                elif kind == "pack_uv":
                    p["u1_mV"] = 260_000
                elif kind == "sensor_error":
                    p["status"] = 0x8
                elif kind == "current_lost":
                    p["current_mA"] = None
        for g0, kind in glitches:
            if kind == "spike_sample" and g0 <= tt < g0 + 0.004:
                p["current_mA"] = 400_000
            elif kind == "spike_30ms" and g0 <= tt < g0 + 0.03:
                p["current_mA"] = 350_000
            elif kind == "status_sample" and g0 <= tt < g0 + 0.004:
                p["status"] = 0x8
            elif kind == "u1_sample" and g0 <= tt < g0 + 0.004:
                p["u1_mV"] = 480_000
        return p

    imp = Impersonator(b, prof, keep_stopped=True)
    mark = b.mark()
    t0 = imp.run(seconds)
    time.sleep(0.3)
    rises = []                                         # (time from t0, flag name)
    for ht, old, flags in b.faults_since(mark):
        for name in ("CURRENT", "VOLTAGE", "IVT"):
            if flags & ERR[name] and not old & ERR[name]:
                rises.append((ht - t0, name))
    worst, missed, false = 0.0, [], []
    for s0, s1, kind in windows:
        hits = [r for r in rises if r[1] == real_kinds[kind] and s0 <= r[0] <= s1 + 0.05]
        if not hits or hits[0][0] - s0 > 0.5:
            missed.append(f"{kind}@{s0:.1f}s")
        else:
            worst = max(worst, hits[0][0] - s0)
    for rt, name in rises:
        if not any(s0 <= rt <= s1 + 0.6 for s0, s1, _ in windows):
            false.append(f"{name}@{rt:.2f}s")
    rec = b.wait_healthy(8.0)
    ok = not missed and not false and not b.resets_since(mark) and rec is not None
    return ok, f"{seconds:.0f} s: {len(windows)} real faults, {len(glitches)} glitches; worst reaction " \
               f"{worst * 1000:.0f} ms; missed {missed or 'none'}; false trips {false or 'none'}; " \
               f"{imp.sent} frames, worst send lateness {imp.worst * 1000:.1f} ms ({imp.late} late), IVT re-stopped "                f"{imp.restops} times"


def t_bus_errors(b, tries=10):
    """A node at the wrong bitrate (the adapter at 250 kbit/s, 2 s at a time): the IVT's frames are destroyed, the
    BMS faults within the deadline and recovers once the bus is clean. Now and then the IVT latches a system
    error with only CAN errors logged: the BMS must clear it by itself (once is enough: it may do so only
    CFG_IVT_AUTO_CLEAR_MAX times per power-up)."""
    bitrate = b.kwargs["bitrate"]
    tried, cleared, late = 0, False, []
    for tried in range(1, tries + 1):
        mark = b.mark()
        b.kwargs["bitrate"] = bitrate // 2
        b.reopen()
        time.sleep(2.0)
        b.kwargs["bitrate"] = bitrate
        b.reopen()
        rec = b.wait_healthy(8.0)
        lines = [l for _, l in b.lines_since(mark)]
        if rec is None:
            kept = [l for l in lines if "Not cleared automatically" in l or "not again" in l]
            return False, f"try {tried}: not healthy 8 s after the disturbance; {kept[0] if kept else 'IVT status %s' % b.ivt_status()}"
        if any("system errors cleared" in l for l in lines):
            cleared = True
            what = next(l for l in lines if "Clearing them" in l)
            late.append(rec)
            break
        if b.resets_since(mark):
            return False, f"try {tried}: BMS reset"
    return True, f"{tried} disturbances; " + (f"the IVT latched a system error, the BMS cleared it ({what.strip()}), "
                                                f"healthy {late[0]:.1f} s after the bus was clean" if cleared else
                                                "the IVT never latched a system error; the BMS recovered every time")


TESTS = {name[2:]: fn for name, fn in globals().items() if name.startswith("t_")}


def main():
    global SOAK_S
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("tests", nargs="*")
    ap.add_argument("--list", action="store_true")
    ap.add_argument("--interface", default="seeedstudio")
    ap.add_argument("--channel", required=not any(a == "--list" for a in sys.argv),
                    help="adapter channel, e.g. COM8")
    ap.add_argument("--bitrate", type=int, default=500000)
    ap.add_argument("--uart", required=not any(a == "--list" for a in sys.argv),
                    help="BMS UART, e.g. COM7")
    ap.add_argument("--soak-seconds", type=float, default=SOAK_S)
    args = ap.parse_args()
    if args.list:
        for n, fn in TESTS.items():
            print(f"{n:30} {(fn.__doc__ or '').strip().splitlines()[0] if fn.__doc__ else ''}")
        return
    SOAK_S = args.soak_seconds
    names = args.tests or list(TESTS)
    b = Bench(args.interface, args.channel, args.bitrate, args.uart)
    results = []
    try:
        for n in names:
            b.check_rx()
            if b.wait_healthy() is None:
                print("      (BMS/IVT not healthy before the test)")
            if (b.ivt_status() or 0) & 0x8:
                print(f"      (IVT reports a system error: clearing it: {clear_ivt_errors(b)})")
                b.wait_healthy()
            ok, notes = TESTS[n](b)
            if b.rx_errors:
                notes += f" [adapter: {b.rx_errors} receive errors, last {b.last_rx_error}]"
                b.rx_errors = 0
            results.append((n, ok))
            print(f"{'PASS' if ok else 'FAIL'}  {n:28} {notes}", flush=True)
            time.sleep(0.5)
    finally:
        b.close()
    print(f"{sum(ok for _, ok in results)} passed, {sum(not ok for _, ok in results)} failed")


if __name__ == "__main__":
    main()

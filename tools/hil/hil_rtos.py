#!/usr/bin/env python3
"""
Hardware-in-the-loop test for TIBURON-AUV-FreeRTOS on a NUCLEO-F446RE.

The laptop talks to the board's console over the ST-LINK USB cable, presses
the bench keys (Core/Src/bench.c, BENCH_HIL build) and checks what comes
back. The board measures itself; the laptop drives it and keeps score.

Wiring (each block of tests SKIPs cleanly if its wiring is absent):

    PA9  (D8)  ->  PA10 (D2)     USART1 loopback: the Pi-link UART talks to
                                 itself through DMA + IDLE, as on the vehicle
    PA15 (CN7-17) -> PWM pin     moved pin to pin when the script asks
    MPU-6050 (GY-521): VCC->3V3, GND->GND, SCL->PB8 (D15), SDA->PB9 (D14)

    pip install pyserial
    python hil_rtos.py                 # auto-detects the ST-LINK port
    python hil_rtos.py --port COM5
    python hil_rtos.py --no-pwm        # skip the jumper-moving PWM test
    python hil_rtos.py --selftest      # run against a fake board, no hardware

Exit code 0 if every test that ran passed. Writes hil_rtos_report.md next to
this script, for pasting into the README.
"""
from __future__ import annotations

import argparse
import datetime as dt
import os
import platform
import random
import re
import subprocess
import sys
import threading
import time

CYC_PER_US = 180.0                 # DWT counts HCLK
TICK_US = 20000                    # TIM7 period
CMD_TIMEOUT_MS = 500
LSI_MIN, LSI_TYP, LSI_MAX = 17000, 32000, 47000   # DS10693, Hz
IWDG_COUNTS = (500 + 1) * 64       # RLR 500, PR /64 (iwdg.h)

# Static predictions this run is checked against (README section 8, rebuilt
# for the current task set). Bytes of stack used, worst case.
STACK_PRED = {}                    # filled in from the README once measured

PWM_PINS = [                       # channel -> (pin, where it is on a Nucleo-64)
    ("PB4", "D5"), ("PB5", "D4"), ("PB0", "A3"), ("PB1", "CN10 pin 24"),
    ("PC6", "CN10 pin 4"), ("PC7", "D9"), ("PC8", "CN10 pin 2"), ("PC9", "CN10 pin 1"),
]


# ---------------------------------------------------------------------------
# Console link
# ---------------------------------------------------------------------------
class Console:
    """Reader thread that timestamps every console line."""

    def __init__(self, ser):
        self.ser = ser
        self.lines: list[tuple[float, str]] = []
        self.lock = threading.Lock()
        self._buf = bytearray()
        self._stop = threading.Event()
        self._t = threading.Thread(target=self._reader, daemon=True)
        self._t.start()

    def _reader(self):
        while not self._stop.is_set():
            try:
                data = self.ser.read(256)
            except Exception:
                break
            if not data:
                continue
            t = time.monotonic()
            self._buf += data
            while b"\n" in self._buf:
                raw, _, rest = self._buf.partition(b"\n")
                self._buf = bytearray(rest)
                line = raw.decode("ascii", "replace").strip("\r").rstrip()
                if line:
                    with self.lock:
                        self.lines.append((t, line))

    def close(self):
        self._stop.set()
        self._t.join(1)
        self.ser.close()

    def mark(self) -> int:
        with self.lock:
            return len(self.lines)

    def since(self, idx: int) -> list[tuple[float, str]]:
        with self.lock:
            return self.lines[idx:]

    def key(self, k: str):
        self.ser.write(k.encode("ascii"))

    def wait(self, pred, timeout: float, start: int | None = None):
        """First line at or after index `start` matching pred, or None."""
        start = self.mark() if start is None else start
        end = time.monotonic() + timeout
        seen = start
        while time.monotonic() < end:
            new = self.since(seen)
            for i, (t, ln) in enumerate(new):
                if pred(ln):
                    return t, ln, seen + i
            seen += len(new)
            time.sleep(0.02)
        return None

    def command(self, k: str, ack_prefix: str | None, timeout: float = 3.0):
        m = self.mark()
        self.key(k)
        if ack_prefix is None:
            return True
        return self.wait(lambda ln: ln.startswith(ack_prefix), timeout, m) is not None

    def block(self, k: str, first: str, last: str, timeout: float = 5.0) -> list[str] | None:
        """Press k, return the lines from the one starting with `first` to the
        one starting with `last`, inclusive."""
        for _attempt in range(2):            # one retry: a key can be overwritten
            m = self.mark()
            self.key(k)
            hit = self.wait(lambda ln: ln.startswith(first), timeout, m)
            if hit is None:
                continue
            end = self.wait(lambda ln: ln.startswith(last), timeout, hit[2])
            if end is None:
                continue
            return [ln for _, ln in self.since(hit[2])[: end[2] - hit[2] + 1]]
        return None


def kv(line: str) -> dict:
    out = {}
    for tok in line.split():
        if "=" in tok:
            k, v = tok.split("=", 1)
            out[k] = v
    return out


def ints(d: dict) -> dict:
    o = {}
    for k, v in d.items():
        try:
            o[k] = int(v, 0) if v.startswith("0x") else int(v)
        except ValueError:
            o[k] = v
    return o


def bench_report(c: Console) -> dict | None:
    lines = c.block("b", "B:clk", "B:end")
    if lines is None:
        return None
    rep: dict = {"_lines": lines}
    for ln in lines:
        tag = ln[2:].split(" ", 1)[0]
        rep.setdefault(tag, {}).update(ints(kv(ln)))
    return rep


def health(c: Console) -> dict | None:
    lines = c.block("h", "--- HEALTH", "--- END")
    if lines is None:
        return None
    h: dict = {"_lines": lines}
    for ln in lines:
        if ":" not in ln or ln.startswith("---"):
            continue
        k, v = ln.split(":", 1)
        h[k.strip()] = v.strip()
    m = re.search(r"(\d+) B of (\d+)", h.get("heap free", ""))
    if m:
        h["heap_free"], h["heap_total"] = int(m.group(1)), int(m.group(2))
    m = re.search(r"(\d+) B ever", h.get("heap min", ""))
    if m:
        h["heap_min"] = int(m.group(1))
    m = re.search(r"(\d+) lines", h.get("con drops", ""))
    if m:
        h["con_drops"] = int(m.group(1))
    return h


def stack_audit(c: Console) -> dict | None:
    lines = c.block("s", "--- STACK HWM", "--- END")
    if lines is None:
        return None
    out = {}
    for ln in lines:
        m = re.match(r"(\S+)\s+alloc=\s*(\d+) w\s+free=\s*(\d+) w \(\s*(\d+) B\)\s+used<=\s*(\d+) B", ln)
        if m:
            out[m.group(1)] = {"alloc": int(m.group(2)) * 4, "free": int(m.group(4)), "used": int(m.group(5))}
    out["_lines"] = lines
    return out


def us(cycles) -> float:
    return cycles / CYC_PER_US


# ---------------------------------------------------------------------------
# Results
# ---------------------------------------------------------------------------
class Result:
    def __init__(self, name, proves):
        self.name, self.proves = name, proves
        self.status, self.details = "SKIP", []

    def ok(self, cond: bool, msg: str):
        self.details.append(("PASS" if cond else "FAIL") + ": " + msg)
        if not cond:
            self.status = "FAIL"
        elif self.status == "SKIP":
            self.status = "PASS"
        return cond

    def note(self, msg: str):
        self.details.append("info: " + msg)

    def skip(self, why: str):
        self.status = "SKIP"
        self.details.append("skipped: " + why)
        return self


class Ctx:
    """What earlier tests learned that later ones need."""
    iwdg = False
    loopback = False
    mpu = None


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
def test_boot(c: Console, ctx: Ctx, press: bool) -> Result:
    r = Result("Boot and console", "clock, reset cause, sensor-absence handling and complete on-demand reports")
    if not press:
        return r.skip("--no-reset")
    m = c.mark()
    print("\n  >>> Press the black RESET button (B2) on the Nucleo now. Waiting 30 s ...", flush=True)
    hit = c.wait(lambda ln: ln == "BOOT OK", 30, m)
    if not r.ok(hit is not None, "'BOOT OK' received (console readable: 180 MHz tree and USART2 BRR right)"):
        return r
    t0, _, i0 = hit
    done = c.wait(lambda ln: ln.startswith("--- END"), 12, i0)       # auto stack+health ~5 s after boot
    lines = [ln for _, ln in c.since(i0)]
    for ln in lines[:40]:
        r.note("console: " + ln)
    clk = next((ln for ln in lines if ln.startswith("CLK:")), "")
    r.ok("OK (HSE bypass" in clk, f"clock source: {clk or 'no CLK line'}")
    rst = next((ln for ln in lines if ln.startswith("RESET:")), "")
    r.ok("PIN" in rst, f"reset cause reported: '{rst}'")
    r.ok(any(ln.startswith("UARTS CONFIGURED") for ln in lines), "init ran to the scheduler start")
    ctx.iwdg = any("IWDG: ENABLED" in ln for ln in lines)
    r.note("watchdog: " + ("ENABLED" if ctx.iwdg else "DISABLED (bench default; the IWDG test will be skipped)"))
    r.ok(any("BENCH_HIL BUILD" in ln for ln in lines), "BENCH_HIL build banner present")
    b30 = next((ln for ln in lines if ln.startswith("Bar30:")), None)
    if b30:
        r.note(b30)
    r.ok(done is not None, "automatic stack + health report printed after boot (scheduler and stdio owner alive)")
    h = health(c)
    if r.ok(h is not None, "'h' health report returned"):
        r.ok("log stage" in h and "sd blocks" in h,
             f"health report complete: {len(h['_lines'])} lines, last field present (was cut at 8 by the queue before the fix)")
        r.ok(h.get("con_drops", -1) == 0, f"console drops: {h.get('con_drops')} (expect 0)")
        r.note("sensors: " + h.get("sensors", "?"))
    return r


def test_clock(c: Console, ctx: Ctx, seconds: float) -> Result:
    r = Result("Clock tree against the laptop", "SYSCLK 180 MHz, APB1 timers 90 MHz, SysTick 1 kHz, all mutually consistent")
    a = bench_report(c)
    t_a = time.monotonic()
    time.sleep(seconds)
    b = bench_report(c)
    t_b = time.monotonic()
    if not r.ok(a is not None and b is not None, "two bench reports"):
        return r
    dtick = (b["clk"]["tick"] - a["clk"]["tick"]) & 0xFFFFFFFF
    dus = (b["clk"]["us"] - a["clk"]["us"]) & 0xFFFFFFFF
    dcyc = (b["clk"]["cyc"] - a["clk"]["cyc"]) & 0xFFFFFFFF
    host_ms = 1000 * (t_b - t_a)
    ratio = dtick / host_ms
    r.ok(abs(ratio - 1) < 0.005, f"SysTick vs laptop: {dtick} ms vs {host_ms:.0f} ms ({100 * (ratio - 1):+.3f} %, limit 0.5 %)")
    r.ok(abs(dus / (dtick * 1000) - 1) < 0.0005,
         f"TIM2 (APB1 timer clock / 90) vs SysTick (HCLK / 180000): {dus} us over {dtick} ticks")
    cyc_per_us = dcyc / dus if dus else 0
    r.ok(abs(cyc_per_us - 180) < 0.05, f"DWT CYCCNT / TIM2 = {cyc_per_us:.4f} cycles per us (expect 180: HCLK = 2 x APB1 timer clock)")
    r.note("laptop ratio carries USB timestamp jitter of a few ms; a wrong PLL shows up as tens of percent")
    return r


def timing_block(r: Result, rep: dict, label: str):
    t7, wk, ex = rep.get("t7", {}), rep.get("wake", {}), rep.get("exec", {})
    if not r.ok(t7.get("n", 0) > 100, f"[{label}] {t7.get('n', 0)} TIM7 periods measured"):
        return
    pmin, pmax, pavg = us(t7["min"]), us(t7["max"]), us(t7["avg"])
    r.ok(abs(pavg - TICK_US) < 2, f"[{label}] TIM7 period mean {pavg:.2f} us (expect 20000)")
    jit = max(TICK_US - pmin, pmax - TICK_US)
    r.ok(jit < 100, f"[{label}] TIM7 ISR entry jitter: period {pmin:.2f}..{pmax:.2f} us, worst {jit:.2f} us from nominal")
    r.ok(us(wk["max"]) < 100, f"[{label}] TIM7 ISR -> control_task running: min {us(wk['min']):.2f} us, "
                              f"avg {us(wk['avg']):.2f} us, max {us(wk['max']):.2f} us")
    r.ok(us(ex["max"]) < 1000, f"[{label}] control_task body (PID + allocation + PWM + log post): "
                               f"avg {us(ex['avg']):.1f} us, max {us(ex['max']):.1f} us of the 20 ms period")
    cpu = rep.get("cpu", {})
    if cpu:
        parts = ", ".join(f"{k} {v / 10:.1f} %" for k, v in cpu.items() if k not in ("win_us",) and isinstance(v, int))
        r.note(f"[{label}] CPU over {cpu.get('win_us', 0) / 1e6:.1f} s: {parts}")


def test_timing_idle(c: Console, ctx: Ctx, seconds: float) -> Result:
    r = Result("Scheduling, link idle", "50 Hz control deadline met with measured jitter, latency and CPU budget")
    c.command("z", "B:zeroed")
    time.sleep(seconds)
    rep = bench_report(c)
    if not r.ok(rep is not None, "bench report"):
        return r
    timing_block(r, rep, "idle")
    msp = rep.get("msp", {})
    r.ok(0 < msp.get("used", 0) < msp.get("size", 1024),
         f"MSP high-water mark {msp.get('used')} B of {msp.get('size')} B (README predicted <= 360 B)")
    return r


def test_heap(c: Console, ctx: Ctx) -> Result:
    r = Result("Heap: the falsifiable prediction", "every allocation happens before the scheduler: free == minimum-ever-free")
    h = health(c)
    if not r.ok(h is not None and "heap_free" in h, "health report"):
        return r
    r.ok(h["heap_free"] == h["heap_min"],
         f"heap free {h['heap_free']} B == minimum ever {h['heap_min']} B of {h['heap_total']} B")
    return r


def test_loopback_detect(c: Console, ctx: Ctx) -> Result:
    r = Result("USART1 loopback present", "PA9 jumpered to PA10: the vehicle's Pi-link UART receives its own frames")
    c.command("x", "B:inj off")
    c.command("z", "B:zeroed")
    time.sleep(2.0)
    rep = bench_report(c)
    n = rep["inj"]["tlm"] if rep else 0
    ctx.loopback = n > 20
    if not ctx.loopback:
        r.note(f"{n} telemetry frames came back in 2 s: no PA9->PA10 jumper. USART1 tests skipped.")
        return r.skip("no loopback jumper")
    r.ok(True, f"{n} looped-back telemetry frames in 2 s")
    return r


def test_telemetry(c: Console, ctx: Ctx, seconds: float) -> Result:
    r = Result("Telemetry over USART1, DMA + IDLE", "TX framing and CRC, and the DMA/IDLE receive path, on the real UART")
    if not ctx.loopback:
        return r.skip("no loopback")
    c.command("z", "B:zeroed")
    time.sleep(seconds)
    rep = bench_report(c)
    inj, rx = rep["inj"], rep["rx"]
    rate = inj["tlm"] / seconds
    r.ok(48.5 <= rate <= 51.5, f"{inj['tlm']} frames in {seconds:.0f} s = {rate:.2f} Hz")
    r.ok(inj["tbad"] == 0, f"{inj['tbad']} looped frames failed CRC")
    r.ok(rx["ringdrop"] == 0, f"ring buffer drops: {rx['ringdrop']}")
    r.note(f"DMA bytes {rx['dma']}, drains from HT/TC {rx['dmaev']}, from IDLE {rx['idleev']}")
    return r


def wait_ctl(c: Console, pred, timeout: float):
    end = time.monotonic() + timeout
    rep = None
    while time.monotonic() < end:
        rep = bench_report(c)
        if rep and pred(rep["ctl"]):
            return rep
        time.sleep(0.2)
    return None


def test_injection(c: Console, ctx: Ctx, seconds: float) -> Result:
    r = Result("Commands in, round trip out", "CMD frames parsed, queued and applied on the next tick; values survive exactly")
    if not ctx.loopback:
        return r.skip("no loopback")
    c.command("x", "B:inj off")
    time.sleep(0.8)
    c.command("z", "B:zeroed")
    c.command("a", "B:inj armed")
    up = wait_ctl(c, lambda k: k["fs"] == 0 and k["armed"] == 1, 3)
    if not r.ok(up is not None, "link up and armed after 3 valid packets"):
        return r
    c.command("z", "B:zeroed")
    time.sleep(seconds)
    rep = bench_report(c)
    inj, cl, ctl = rep["inj"], rep["cmdlat"], rep["ctl"]
    r.ok(inj["rtok"] > 0.9 * inj["tlm"] and inj["rtbad"] == 0,
         f"{inj['rtok']}/{inj['tlm']} telemetry depth values are exact copies of an injected command; {inj['rtbad']} unknown")
    r.ok(ctl["fs"] == 0 and ctl["armed"] == 1, "stayed linked and armed at 50 Hz")
    r.ok(ctl["qdrop"] == 0, f"commandQueue drops: {ctl['qdrop']}")
    r.ok(cl["n"] > 0 and us(cl["max"]) < 20000,
         f"parse -> applied by control_task: avg {us(cl['avg']) / 1000:.2f} ms, max {us(cl['max']) / 1000:.2f} ms (bound: one 20 ms tick)")
    timing_block(r, rep, "loaded: 2 x 62 B on USART1 per tick")
    return r


def test_junk(c: Console, ctx: Ctx, seconds: float) -> Result:
    r = Result("Resync through junk and split frames",
               "parser recovers from garbage, decoy headers and frames split across two IDLE events")
    if not ctx.loopback:
        return r.skip("no loopback")
    c.command("j", "B:inj junk")
    time.sleep(0.5)
    c.command("z", "B:zeroed")
    time.sleep(seconds)
    rep = bench_report(c)
    inj, ctl = rep["inj"], rep["ctl"]
    r.ok(ctl["fs"] == 0, "link never timed out")
    r.ok(rep["fs_ms"]["n"] == 0, f"failsafe entries during the run: {rep['fs_ms']['n']}")
    r.ok(inj["rtbad"] == 0 and inj["poison"] == 0, f"{inj['rtbad']} corrupted values reached control_task")
    r.ok(inj["rtok"] > 0, f"{inj['rtok']} round trips matched")
    c.command("x", "B:inj off")
    return r


def test_badcrc(c: Console, ctx: Ctx) -> Result:
    r = Result("Corrupted frames rejected", "a bad CRC is never acted on and the link times out as if silent")
    if not ctx.loopback:
        return r.skip("no loopback")
    c.command("a", "B:inj armed")
    wait_ctl(c, lambda k: k["fs"] == 0, 3)
    c.command("z", "B:zeroed")
    c.command("e", "B:inj badcrc")
    time.sleep(1.5)
    rep = bench_report(c)
    r.ok(rep["inj"]["poison"] == 0, "the -999 value carried by every bad frame never reached the control loop")
    r.ok(rep["ctl"]["fs"] == 1 and rep["ctl"]["armed"] == 0, "failsafe entered, disarmed")
    fs = rep["fs_ms"]
    r.ok(fs["n"] == 1 and CMD_TIMEOUT_MS < fs["max"] <= CMD_TIMEOUT_MS + 20,
         f"tripped {fs['max']} ms after the last good frame (limit 500 ms + one tick)")
    c.command("x", "B:inj off")
    return r


def test_failsafe(c: Console, ctx: Ctx, runs: int) -> Result:
    r = Result("Command-timeout failsafe", "silence from the Pi disarms the vehicle within 500 ms + one tick, every time")
    if not ctx.loopback:
        return r.skip("no loopback")
    c.command("z", "B:zeroed")
    for i in range(runs):
        c.command("a", "B:inj armed")
        wait_ctl(c, lambda k: k["fs"] == 0 and k["armed"] == 1, 3)
        time.sleep(random.uniform(0.2, 0.6))      # vary the phase against TIM7
        c.command("x", "B:inj off")
        time.sleep(1.0)
    rep = bench_report(c)
    fs = rep["fs_ms"]
    r.ok(fs["n"] == runs, f"{fs['n']}/{runs} failsafe entries")
    r.ok(CMD_TIMEOUT_MS < fs["min"] and fs["max"] <= CMD_TIMEOUT_MS + 20,
         f"latency {fs['min']}..{fs['max']} ms after the last command (detected on the 20 ms tick)")
    r.ok(rep["ctl"]["armed"] == 0, "disarmed")
    return r


def test_failsafe_comms_dead(c: Console, ctx: Ctx) -> Result:
    r = Result("Failsafe with comms_task suspended",
               "the timeout lives in control_task, so a hung comms task still stops the thrusters")
    if not ctx.loopback:
        return r.skip("no loopback")
    c.command("a", "B:inj armed")
    wait_ctl(c, lambda k: k["fs"] == 0 and k["armed"] == 1, 3)
    c.command("z", "B:zeroed")
    c.command("K", "B:comms suspended")
    time.sleep(1.2)
    rep = bench_report(c)
    c.command("k", "B:comms resumed")
    c.command("x", "B:inj off")
    fs = rep["fs_ms"]
    r.ok(rep["ctl"]["fs"] == 1 and rep["ctl"]["armed"] == 0, "failsafe entered and disarmed with comms_task suspended")
    r.ok(fs["n"] == 1 and CMD_TIMEOUT_MS < fs["max"] <= CMD_TIMEOUT_MS + 20, f"after {fs['max']} ms")
    time.sleep(0.5)
    return r


def test_recovery(c: Console, ctx: Ctx) -> Result:
    r = Result("Failsafe recovery needs 3 consecutive packets",
               "one stray packet cannot re-arm; a gap longer than the timeout restarts the count")
    if not ctx.loopback:
        return r.skip("no loopback")
    c.command("x", "B:inj off")
    time.sleep(1.0)
    rep = wait_ctl(c, lambda k: k["fs"] == 1, 3)
    if not r.ok(rep is not None, "starting in failsafe"):
        return r

    def one():
        c.command("1", "B:inj one")
        time.sleep(0.15)
        return bench_report(c)["ctl"]

    k1, k2, k3 = one(), one(), one()
    r.ok(k1["fs"] == 1 and k1["rec"] == 1, f"after packet 1: failsafe={k1['fs']} streak={k1['rec']}")
    r.ok(k2["fs"] == 1 and k2["rec"] == 2, f"after packet 2: failsafe={k2['fs']} streak={k2['rec']}")
    r.ok(k3["fs"] == 0 and k3["armed"] == 1, f"after packet 3: failsafe={k3['fs']} armed={k3['armed']}")

    time.sleep(1.0)                               # back into failsafe
    g1, g2 = one(), one()
    r.ok(g2["rec"] == 2, f"gap test: streak {g1['rec']} -> {g2['rec']}")
    time.sleep(1.2)                               # longer than the 500 ms timeout
    g3 = one()
    r.ok(g3["fs"] == 1 and g3["rec"] == 1,
         f"packet 3 after a 1.2 s gap: failsafe={g3['fs']} streak={g3['rec']} "
         "(before the fix this re-armed the vehicle)")
    time.sleep(1.0)
    return r


def test_wrap(c: Console, ctx: Ctx) -> Result:
    r = Result("DMA exact-wrap aliasing",
               "a 256-byte burst between two IDLE events is lost with IDLE-only draining, and kept with HT/TC")
    if not ctx.loopback:
        return r.skip("no loopback")
    c.command("x", "B:inj off")
    time.sleep(0.3)
    res = {}
    for key, mode in (("v", "legacy"), ("w", "fix")):
        m = c.mark()
        c.key(key)
        hit = c.wait(lambda ln: ln.startswith("B:wrap"), 3, m)
        res[mode] = ints(kv(hit[1])) if hit else None
        time.sleep(0.3)
    if r.ok(res["legacy"] is not None, "legacy-mode burst measured"):
        got = res["legacy"]["got"]
        r.note(f"IDLE-only (pre-fix) behaviour: {got} of 256 bytes delivered")
        r.ok(got == 0, "defect reproduced: the whole 256-byte burst vanished, with no error counted anywhere")
    if r.ok(res["fix"] is not None, "fixed-mode burst measured"):
        r.ok(res["fix"]["got"] == 256, f"with HT/TC draining: {res['fix']['got']} of 256 bytes delivered")
    return r


def test_mpu(c: Console, ctx: Ctx, rounds: int) -> Result:
    r = Result("I2C master receive vs an MPU-6050, under preemption",
               "1/2/6/14-byte reads stay correct while comms_task preempts the I2C task mid-transfer")
    if ctx.loopback:
        c.command("a", "B:inj armed")             # comms_task busy ~11 ms of every 20
    results = {}
    for key, mode in (("i", "legacy"), ("I", "rm0390")):
        tot = {"n": 0, "ewho": 0, "etmp": 0, "eacc": 0, "e14": 0, "i2cerr": 0, "stale": 0}
        t14 = []
        for _ in range(rounds):
            m = c.mark()
            c.key(key)
            hit = c.wait(lambda ln: ln.startswith("B:mpu2") or ln.startswith("B:mpu absent"), 30, m)
            if hit is None or "absent" in hit[1]:
                for _, ln in c.since(m):
                    if ln.startswith("B:i2c"):
                        r.note(ln)
                if hit:
                    r.note(hit[1])
                c.command("x", "B:inj off")
                return r.skip("no MPU-6050 answering at 0x68 on PB8/PB9")
            lines = [ln for _, ln in c.since(m)]
            for ln in lines:
                if ln.startswith("B:i2c") or ln.startswith("B:mpu3"):
                    r.note(ln)
            a = ints(kv(next(ln for ln in lines if ln.startswith("B:mpu m="))))
            b = ints(kv(hit[1]))
            for k in ("n", "ewho", "etmp", "eacc", "e14"):
                tot[k] += a[k]
            if isinstance(b.get("pwr"), int) and b["pwr"] & 0x40:
                r.ok(False, f"{mode}: PWR_MGMT_1 = 0x{b['pwr']:02X} after waking: still asleep, the wake write's data byte was lost")
            tot["i2cerr"] += b["i2cerr"]
            tot["stale"] += b["stale"]
            ctx.mpu = a.get("who")
            lo, hi = (int(x) for x in str(b["t14"]).split("-"))
            t14 += [lo, hi]
        tot["t14"] = (min(t14), max(t14))
        results[mode] = tot
    c.command("x", "B:inj off")
    r.note(f"WHO_AM_I = 0x{ctx.mpu:02X}")
    for mode, t in results.items():
        bad = t["ewho"] + t["etmp"] + t["eacc"] + t["e14"] + t["i2cerr"]
        r.note(f"{mode}: {t['n']} cycles x 4 reads; wrong WHO {t['ewho']}, implausible temp {t['etmp']}, "
               f"accel {t['eacc']}, 14-byte {t['e14']}, bus errors {t['i2cerr']}, stale-RXNE {t['stale']}; "
               f"14-byte read {t['t14'][0]}..{t['t14'][1]} us")
        t["bad"] = bad
    r.ok(results["rm0390"]["bad"] == 0 and results["rm0390"]["stale"] == 0,
         f"i2c_read() (RM0390): 0 errors expected, got {results['rm0390']['bad']} errors, {results['rm0390']['stale']} stale bytes")
    leg = results["legacy"]
    if leg["bad"] or leg["stale"]:
        r.note(f"i2c_read_legacy() failed {leg['bad']} reads and left {leg['stale']} stale bytes "
               "under the same load: the RM0390 sequence is needed")
    else:
        r.note("i2c_read_legacy() showed no errors in this run")
    lo = results["rm0390"]["t14"][0]
    r.note(f"14-byte burst floor {lo} us vs ~1.55 ms computed for 100 kHz: confirms the I2C clock, CCR = 225")
    return r


def test_soak(c: Console, ctx: Ctx, seconds: float) -> Result:
    r = Result(f"Soak, {seconds:.0f} s, commands in and telemetry out at 50 Hz",
               "no CRC errors, drops, failsafe trips or resets under sustained load")
    if not ctx.loopback:
        return r.skip("no loopback")
    c.command("a", "B:inj armed")
    wait_ctl(c, lambda k: k["fs"] == 0, 3)
    c.command("z", "B:zeroed")
    t0 = time.monotonic()
    a = bench_report(c)
    while time.monotonic() - t0 < seconds:
        time.sleep(min(5, seconds))
        print(f"    soak {time.monotonic() - t0:4.0f}/{seconds:.0f} s", end="\r", flush=True)
    print(" " * 30, end="\r")
    rep = bench_report(c)
    inj, rx, ctl = rep["inj"], rep["rx"], rep["ctl"]
    r.ok(rep["clk"]["tick"] > a["clk"]["tick"], "no reset during the soak (tick count kept rising)")
    r.ok(inj["tbad"] == 0, f"{inj['tlm']} telemetry frames, {inj['tbad']} CRC failures")
    r.ok(inj["rtbad"] == 0 and inj["poison"] == 0, f"{inj['rtok']} commands round-tripped exactly, {inj['rtbad']} wrong")
    r.ok(rep["fs_ms"]["n"] == 0 and ctl["fs"] == 0, "link never dropped")
    r.ok(rx["ringdrop"] == 0 and ctl["qdrop"] == 0, f"ring drops {rx['ringdrop']}, command-queue drops {ctl['qdrop']}")
    timing_block(r, rep, "soak")
    c.command("x", "B:inj off")
    return r


def test_stacks(c: Console, ctx: Ctx) -> Result:
    r = Result("Stack high-water marks after the error paths ran",
               "measured worst case per task, after failsafe, junk, bad CRC, I2C absence and console reports")
    s = stack_audit(c)
    if not r.ok(s is not None, "stack audit"):
        return r
    for name, v in s.items():
        if name.startswith("_"):
            continue
        r.ok(v["free"] >= 128, f"{name:8s} used {v['used']:4d} B of {v['alloc']} B, {v['free']} B never touched")
    rep = bench_report(c)
    if rep:
        msp = rep["msp"]
        r.ok(msp["used"] < msp["size"], f"MSP (handlers) used {msp['used']} B of {msp['size']} B")
        r.ok(msp["heap_free"] == msp["heap_min"], f"heap still free == min-ever ({msp['heap_free']} B) after all tests")
    h = health(c)
    if h:
        r.ok(h.get("con_drops", -1) == 0, f"console lines dropped over the whole run: {h.get('con_drops')}")
    return r


def test_pwm(c: Console, ctx: Ctx, interactive: bool) -> Result:
    r = Result("Eight PWM outputs, measured by the chip",
               "each pin carries its own channel at 1 us resolution and a 20 ms frame (TIM2 capture on PA15)")
    if not interactive:
        return r.skip("--no-pwm")
    c.command("P", "B:pwm signature=1")
    measured = 0
    try:
        for ch, (pin, where) in enumerate(PWM_PINS):
            exp = 1100 + 100 * ch
            print(f"\n  >>> Put the PA15 jumper (CN7 pin 17) on {pin} ({where}) for thruster {ch + 1}, "
                  "then press Enter (s = skip this pin) ", end="", flush=True)
            if input().strip().lower() == "s":
                r.note(f"ch{ch} {pin}: skipped by operator")
                continue
            measured += 1
            c.block("p", "B:pwm", "B:pwm")       # discard the window that spans the move
            time.sleep(1.2)
            got = c.block("p", "B:pwm", "B:pwm")
            if not r.ok(got is not None, f"ch{ch} {pin}: capture report"):
                continue
            k = ints(kv(got[0]))
            lo, hi = (int(x) for x in str(k["hi"]).split("-"))
            plo, phi = (int(x) for x in str(k["per"]).split("-"))
            if k["n"] == 0:
                r.ok(False, f"ch{ch} {pin}: no edges on PA15 (jumper on the right pin?)")
                continue
            r.ok(abs(lo - exp) <= 1 and abs(hi - exp) <= 1,
                 f"ch{ch} {pin}: high time {lo}..{hi} us over {k['n']} pulses (expect {exp}: this channel and no other)")
            r.ok(abs(plo - 20000) <= 1 and abs(phi - 20000) <= 1, f"ch{ch} {pin}: period {plo}..{phi} us (expect 20000)")
    finally:
        c.command("P", "B:pwm signature=0")
    if measured == 0:
        return r.skip("all eight pins skipped by the operator")
    time.sleep(0.3)
    c.block("p", "B:pwm", "B:pwm")
    time.sleep(1.2)
    got = c.block("p", "B:pwm", "B:pwm")
    if got:
        k = ints(kv(got[0]))
        r.ok(k["n"] > 0 and str(k["hi"]) == "1500-1500", f"signature off: back to neutral, {k['hi']} us")
    return r


def test_fault_latch(c: Console, ctx: Ctx) -> Result:
    r = Result("Fault latch survives a warm reset", "a configASSERT records file, line and PC in .noinit, reported on the next boot")
    m = c.mark()
    c.key("F")
    c.wait(lambda ln: ln.startswith("B:fault"), 3, m)
    if ctx.iwdg:
        r.note("watchdog enabled: the halted board should reset itself")
        hit = c.wait(lambda ln: ln == "BOOT OK", 6, m)
        if hit is None:
            # BKPT halts the core instead of faulting if the debug unit is
            # still enabled from flashing, and the IWDG is frozen on halt.
            r.note("no self-reset in 6 s: core halted at the breakpoint (debug still enabled since flashing; "
                   "unplug/replug USB before a run to avoid this)")
            print("\n  >>> Board halted at the breakpoint. Press RESET (B2) now. Waiting 30 s ...", flush=True)
            hit = c.wait(lambda ln: ln == "BOOT OK", 30, m)
    else:
        print("\n  >>> Board halted on purpose. Press RESET (B2) now. Waiting 30 s ...", flush=True)
        hit = c.wait(lambda ln: ln == "BOOT OK", 30, m)
    if not r.ok(hit is not None, "board came back"):
        return r
    lat = c.wait(lambda ln: "LATCHED FAULT" in ln, 3, hit[2])
    r.ok(lat is not None, "'*** LATCHED FAULT FROM PREVIOUS RUN ***' printed")
    lines = [ln for _, ln in c.since(hit[2])][:14]
    kind = next((ln for ln in lines if ln.strip().startswith("kind")), "")
    at = next((ln for ln in lines if ln.strip().startswith("at")), "")
    r.ok("configASSERT" in kind, kind.strip())
    r.ok("bench.c" in at, at.strip())
    time.sleep(6)                                 # let the boot audit finish
    return r


def test_iwdg(c: Console, ctx: Ctx, runs: int) -> Result:
    r = Result("Watchdog resets a hung control loop",
               "only control_task refreshes the IWDG: hang it and the board resets inside the LSI-derived window")
    if not ctx.iwdg:
        return r.skip("IWDG disabled in this build (add -DENABLE_IWDG to the Debug defines to run it)")
    lo = 1000 * IWDG_COUNTS / LSI_MAX
    hi = 1000 * IWDG_COUNTS / LSI_MIN
    times = []
    for i in range(runs):
        time.sleep(6)                             # past the post-boot audit
        m = c.mark()
        c.key("W")
        ack = c.wait(lambda ln: ln.startswith("B:hang"), 3, m)
        if not r.ok(ack is not None, f"run {i + 1}: hang acknowledged"):
            continue
        boot = c.wait(lambda ln: ln == "BOOT OK", 5, ack[2])
        if not r.ok(boot is not None, f"run {i + 1}: board reset itself"):
            continue
        ms = 1000 * (boot[0] - ack[0])
        times.append(ms)
        rst = c.wait(lambda ln: ln.startswith("RESET:"), 2, boot[2])
        r.ok(rst is not None and "IWDG" in rst[1], f"run {i + 1}: reset cause '{rst[1] if rst else '?'}'")
    if times:
        r.ok(all(lo * 0.95 <= t <= hi + 50 for t in times),
             "hang -> reset " + ", ".join(f"{t:.0f}" for t in times)
             + f" ms (window {lo:.0f}..{hi:.0f} ms from the LSI spec; this includes ~20 ms of boot to 'BOOT OK')")
        f_lsi = IWDG_COUNTS / (sum(times) / len(times) / 1000)
        r.note(f"implied LSI frequency on this chip: {f_lsi / 1000:.1f} kHz (spec 17..47, typ 32)")
    time.sleep(6)
    return r


# ---------------------------------------------------------------------------
# Fake board for --selftest: answers keys with plausible lines so the
# script's control flow and parsing can be checked without hardware.
# ---------------------------------------------------------------------------
class FakeSerial:
    def __init__(self):
        self.out = bytearray()
        self.lock = threading.Lock()
        self.t0 = time.monotonic()
        self.fs, self.armed, self.rec, self.inj, self.sig = 1, 0, 0, "off", 0
        self.z = 0.0
        self.fs_list: list[int] = []
        self.last_cmd = None
        self.tlm = 0
        self._pending = []
        threading.Thread(target=self._tick, daemon=True).start()
        threading.Timer(1.0, lambda: self._boot(cause="POR PIN BOR")).start()   # "RESET pressed"

    def _emit(self, s):
        with self.lock:
            self.out += (s + "\r\n").encode()

    def _boot(self, cause="PIN", latch=False):
        self._emit("BOOT OK")
        self._emit("CLK: OK (HSE bypass, 180 MHz)")
        self._emit("RESET: " + cause)
        if latch:
            for ln in ("*** LATCHED FAULT FROM PREVIOUS RUN ***", "  kind : configASSERT",
                       "  at   : bench.c:880", "  pc   : 0x08001234"):
                self._emit(ln)
        self._emit("SD: NO CARD (CMD0 no response)")
        self._emit("IWDG: ENABLED (~1 s, kicked by control_task only)")
        self._emit("UARTS CONFIGURED (IRQs enabled by their tasks)")
        self._emit("*  BENCH_HIL BUILD - laptop test hooks in    *")
        self._emit("Bar30: ABSENT (address NACK (no device))")
        self._health()
        self._stack()

    def _ms(self):
        return int(1000 * (time.monotonic() - self.t0))

    def _tick(self):
        while True:
            time.sleep(0.02)
            now = self._ms()
            if self.inj in ("armed", "junk") or self._pending:
                if self._pending:
                    self._pending.pop()
                self.last_cmd = now
                if self.fs:
                    self.rec += 1
                    if self.rec >= 3:
                        self.fs, self.rec = 0, 0
                if not self.fs:
                    self.armed = 1 if self.inj == "armed" or self.inj == "off" else 0
            if self.last_cmd is not None and now - self.last_cmd > 500:
                if not self.fs:
                    self.fs, self.armed, self.rec = 1, 0, 0
                    self.fs_list.append(now - self.last_cmd + random.randint(1, 15))
                elif self.rec:
                    self.rec = 0
            self.tlm += 1

    def _health(self):
        for ln in ("--- HEALTH ---", "clk       : OK (HSE bypass, 180 MHz)", "heap free : 8704 B of 22528 B",
                   "heap min  : 8704 B ever free", "log drops : 0 records", "con drops : 0 lines",
                   "sensors   : VN200=ABSENT BNO085=ABSENT DVL=ABSENT Bar30=ABSENT", "link      : ok  armed=0  recovery=0/3",
                   "cmd pkts  : 0 valid, 0 dropped (queue full)", "wdg/ramp  : IWDG=ARMED  slew_limit=ramping",
                   "sd blocks : 0 ok, 0 err", "log stage : 0 recs, 0 blks, 0 post drops", "--- END ---"):
            self._emit(ln)

    def _stack(self):
        self._emit("--- STACK HWM (free words = min ever seen) ---")
        for n in ("Control", "Comms", "IMU", "DVL", "Bar30", "SPIOwner", "Logging", "Dummy"):
            self._emit(f"{n:<8s} alloc= 256 w  free= 150 w ( 600 B)  used<= 424 B")
        self._emit("--- END ---")

    def write(self, b: bytes):
        k = b.decode()
        now = self._ms()
        tlm = self.tlm
        if k == "b":
            c = int(now * 180000) & 0xFFFFFFFF
            self._emit(f"B:clk tick={now} us={now * 1000} cyc={c}")
            self._emit("B:t7 n=500 min=3599990 max=3600012 avg=3600000")
            self._emit("B:wake n=500 min=1500 max=2700 avg=1700")
            self._emit("B:exec n=500 min=5000 max=9000 avg=6000")
            self._emit("B:cmdlat n=400 min=1600000 max=1700000 avg=1650000")
            n = len(self.fs_list)
            fmin = min(self.fs_list) if n else 0
            fmax = max(self.fs_list) if n else 0
            self._emit(f"B:fs_ms n={n} min={fmin} max={fmax} avg={fmax}")
            self._emit(f"B:inj sent={tlm} tlm={tlm} tbad=0 rtok={tlm if self.inj != 'off' else 0} rtbad=0 poison=0")
            self._emit(f"B:rx dma={tlm * 124} ringdrop=0 dmaev={tlm} idleev={tlm}")
            self._emit(f"B:ctl fs={self.fs} armed={self.armed} rec={self.rec} valid=10 qdrop=0")
            self._emit("B:cpu win_us=10000000 Ctl=12 Com=80 IMU=0 DVL=0 B30=1 SPI=0 Log=0 Dum=5")
            self._emit("B:cpu idle=900 (permille of window)")
            self._emit("B:msp used=312 size=1024 heap_free=8704 heap_min=8704")
            self._emit("B:end")
        elif k == "z":
            self.tlm = 0
            self.fs_list = []
            self._emit("B:zeroed")
        elif k in "adxej1":
            self.inj = {"a": "armed", "d": "disarmed", "x": "off", "e": "off", "j": "junk", "1": self.inj}[k]
            if k == "1":
                self._pending.append(1)
            self._emit({"a": "B:inj armed", "d": "B:inj disarmed", "x": "B:inj off", "e": "B:inj badcrc",
                        "j": "B:inj junk", "1": "B:inj one"}[k])
        elif k in "vw":
            self._emit(f"B:wrap sent=256 got={0 if k == 'v' else 256} mode={'legacy' if k == 'v' else 'fix'}")
        elif k in "iI":
            mode = "legacy" if k == "i" else "rm0390"
            self._emit(f"B:mpu queued {mode}")
            e = 3 if k == "i" else 0
            self._emit(f"B:mpu m={mode} n=500 who=0x68 ewho={e} etmp=0 eacc={e} e14=0")
            self._emit(f"B:mpu2 m={mode} i2cerr=0 stale={e} t14=1712-1890 pwr=0x00")
        elif k == "P":
            self.sig ^= 1
            self._emit(f"B:pwm signature={self.sig}")
        elif k == "p":
            self._emit("B:pwm n=60 hi=1500-1500 per=20000-20000 sig=0" if not self.sig
                       else "B:pwm n=60 hi=1100-1100 per=20000-20000 sig=1")
        elif k == "K":
            self._emit("B:comms suspended")
            self.inj = "off"
        elif k == "k":
            self._emit("B:comms resumed")
        elif k == "h":
            self._health()
        elif k == "s":
            self._stack()
        elif k == "F":
            self._emit("B:fault tripping configASSERT")
            threading.Timer(1.0, lambda: self._boot(cause="IWDG PIN", latch=True)).start()
        elif k == "W":
            self._emit("B:hang control_task")
            threading.Timer(1.0, lambda: self._boot(cause="IWDG PIN")).start()

    def read(self, n):
        time.sleep(0.01)
        with self.lock:
            d = bytes(self.out[:n])
            del self.out[:n]
        return d

    def close(self):
        pass


# ---------------------------------------------------------------------------
def find_port() -> str:
    import serial.tools.list_ports
    ports = list(serial.tools.list_ports.comports())
    st = [p for p in ports if p.vid == 0x0483 or "STLink" in (p.description or "") or "STM" in (p.manufacturer or "")]
    if len(st) == 1:
        return st[0].device
    listing = "\n".join(f"  {p.device}: {p.description}" for p in ports) or "  (none)"
    sys.exit(("No" if not st else "More than one") + " ST-LINK port found. Ports seen:\n" + listing
             + "\nPass --port explicitly.")


def firmware_commit() -> str:
    try:
        here = os.path.dirname(os.path.abspath(__file__))
        return subprocess.check_output(["git", "-C", here, "describe", "--always", "--dirty"],
                                       stderr=subprocess.DEVNULL, text=True).strip()
    except Exception:
        return "unknown"


def write_report(path, results, port, started, ctx):
    lines = [
        "# Hardware-in-the-loop report: TIBURON-AUV-FreeRTOS", "",
        f"- Date: {started:%Y-%m-%d %H:%M}",
        f"- Board: NUCLEO-F446RE ({port}); USART1 loopback: {'yes' if ctx.loopback else 'no'}; "
        f"MPU-6050: {'WHO_AM_I 0x%02X' % ctx.mpu if ctx.mpu else 'no'}; IWDG: {'enabled' if ctx.iwdg else 'disabled'}",
        f"- Firmware commit: `{firmware_commit()}`",
        f"- Host: {platform.system()} {platform.release()}, Python {platform.python_version()}", "",
        "| Test | Result | What it shows |", "|---|---|---|",
    ]
    for r in results:
        lines.append(f"| {r.name} | **{r.status}** | {r.proves} |")
    lines.append("")
    for r in results:
        lines += [f"## {r.name}: {r.status}", ""] + [f"- {d}" for d in r.details] + [""]
    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--no-reset", action="store_true", help="skip the boot test (no RESET press)")
    ap.add_argument("--no-pwm", action="store_true", help="skip the jumper-moving PWM test")
    ap.add_argument("--soak", type=float, default=60)
    ap.add_argument("--mpu-rounds", type=int, default=2, help="500-cycle MPU runs per read method")
    ap.add_argument("--selftest", action="store_true", help="run against a built-in fake board")
    ap.add_argument("--report", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "hil_rtos_report.md"))
    args = ap.parse_args()

    if args.selftest:
        ser, port = FakeSerial(), "fake"
        args.no_pwm = True
        args.soak = min(args.soak, 3)
    else:
        try:
            import serial
        except ImportError:
            sys.exit("pyserial is required:  pip install pyserial")
        port = args.port or find_port()
        try:
            ser = serial.Serial(port, args.baud, timeout=0.05)
        except serial.SerialException as e:
            sys.exit(f"Could not open {port}: {e}\n(Close CubeIDE's serial console or any terminal using it.)")
        ser.reset_input_buffer()

    started = dt.datetime.now()
    print(f"TIBURON FreeRTOS HIL  port={port}  firmware={firmware_commit()}")
    c = Console(ser)
    ctx = Ctx()
    results = []
    S = lambda name, fn: (name, fn)
    steps = [
        S("boot", lambda: test_boot(c, ctx, args.selftest or not args.no_reset)),
        S("clock", lambda: test_clock(c, ctx, 10 if not args.selftest else 1)),
        S("timing idle", lambda: test_timing_idle(c, ctx, 10 if not args.selftest else 1)),
        S("heap", lambda: test_heap(c, ctx)),
        S("loopback detect", lambda: test_loopback_detect(c, ctx)),
        S("telemetry", lambda: test_telemetry(c, ctx, 10 if not args.selftest else 1)),
        S("injection", lambda: test_injection(c, ctx, 10 if not args.selftest else 1)),
        S("junk", lambda: test_junk(c, ctx, 5 if not args.selftest else 1)),
        S("bad crc", lambda: test_badcrc(c, ctx)),
        S("failsafe", lambda: test_failsafe(c, ctx, 3)),
        S("failsafe, comms dead", lambda: test_failsafe_comms_dead(c, ctx)),
        S("recovery", lambda: test_recovery(c, ctx)),
        S("dma wrap", lambda: test_wrap(c, ctx)),
        S("mpu", lambda: test_mpu(c, ctx, args.mpu_rounds)),
    ]
    if args.soak > 0:
        steps.append(S("soak", lambda: test_soak(c, ctx, args.soak)))
    steps += [
        S("pwm", lambda: test_pwm(c, ctx, not args.no_pwm)),
        S("stacks", lambda: test_stacks(c, ctx)),
        S("fault latch", lambda: test_fault_latch(c, ctx)),
        S("iwdg", lambda: test_iwdg(c, ctx, 3)),
    ]
    try:
        for name, step in steps:
            try:
                r = step()
            except (KeyError, TypeError, StopIteration, ValueError) as e:
                # A report that came back incomplete. Record it and keep going.
                r = Result(name, "did not complete")
                r.ok(False, f"unexpected board output ({type(e).__name__}: {e}); see the console log")
            results.append(r)
            print(f"\n[{r.status}] {r.name}")
            for d in r.details:
                print("    " + d)
    except KeyboardInterrupt:
        print("\ninterrupted")
    finally:
        log = os.path.splitext(args.report)[0] + "_console.log"
        with open(log, "w", encoding="utf-8") as f:
            f.write("\n".join(f"{t:10.3f}  {ln}" for t, ln in c.since(0)))
        c.close()

    write_report(args.report, results, port, started, ctx)
    ran = [r for r in results if r.status != "SKIP"]
    failed = [r for r in ran if r.status == "FAIL"]
    print(f"\n{len(ran) - len(failed)}/{len(ran)} tests passed, {len(results) - len(ran)} skipped. "
          f"Report: {args.report}")
    sys.exit(1 if failed or not ran else 0)


if __name__ == "__main__":
    main()

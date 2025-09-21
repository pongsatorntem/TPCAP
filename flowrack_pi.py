#!/usr/bin/env python3
# flowrack_pi.py — Serial->Logic->POST (20 req/s), with ESP watchdog

import sys, time, json, threading, argparse, random, subprocess
import asyncio
import aiohttp
import serial
from collections import deque





# --- Debug switches ---
VERBOSE_EDGE = True       # print every rising/falling edge routing
VERBOSE_SNAPSHOT = False  # set True if you want to see snapshot repairs
VERBOSE_SERIAL = False    # print each JSON line from ESP (spammy)

# --- Light counters for health ---
edges_seen = 0
snapshots_seen = 0
enqueued_count = 0
posted_count = 0
_consecutive_post_fails = 0



# ===== States (must match server) =====
COUNT_UP=1; COUNT_DOWN=2; HOLD=3; NO_HOLD=4; ITEM_IN=5
SLOT_UNDEFINED=0; SLOT_RECEIVE=1; SLOT_SEND=2

# ===== Timings =====
HOLD_MS = 10_000                # HOLD threshold (ms) while state==1
SAFETY_CLEAR_MS = 60_000        # resend NO_HOLD every 60s while state==0
ESP_TIMEOUT_MS = 5_000          # if no lines from ESP for 5s -> reset try
ESP_MAX_RESETS = 3              # max consecutive resets before reopening port

# ===== Post rate-limiting =====
POST_RPS = 20                   # <= 20 requests per second
MIN_POST_INTERVAL = 1.0 / POST_RPS


current_station_code = None
_pending_hp = None
ingest_q = None          # will be created in main()
_main_loop = None        # event loop handle for thread-safe post

def _ingest_try_put(obj):
    # Thread-safe: called via _main_loop.call_soon_threadsafe
    try:
        ingest_q.put_nowait(obj)
    except asyncio.QueueFull:
        log("⚠️ ingest_q full; dropping line")

class SlotState:
    __slots__ = ("transfer","row","col","flowrack",
                 "last_state","on_since","off_since",
                 "last_hold_sent_on_since","last_no_hold_sent_at","safety_period")
    def __init__(self, transfer, row, col, flowrack):
        self.transfer = transfer
        self.row = row
        self.col = col
        self.flowrack = flowrack
        self.last_state = 0
        self.on_since = None
        self.off_since = None
        self.last_hold_sent_on_since = None
        self.last_no_hold_sent_at = None
        self.safety_period = SAFETY_CLEAR_MS

def log(*a):
    print(*a, flush=True)

# ===== Global containers =====
slots = {}              # idx(1..32) -> SlotState
MAX_POST_Q = 2000       # เพดานกัน RAM ล้นเวลาเน็ตล่มนานๆ
post_q = deque()        # payload queue for posting
running_num = 1

# Updated on every received line from ESP
_last_esp_seen_ms = int(time.time()*1000)
_esp_resets = 0

# Serial holder (shared with reader thread)
_ser_lock = threading.Lock()
_ser = None

def enqueue(flowrack, state, row, col, station_code):
    global running_num, enqueued_count
    payload = {
        "station_code": str(station_code),
        "flowrack_name": flowrack,
        "state": int(state),
        "row": int(row),
        "col": int(col),
        "running_num": running_num,
    }
    running_num += 1

    # --- dedupe NO_HOLD: ถ้าหางคิวเป็น NO_HOLD ซ้ำ slot เดิม ให้ข้าม ---
    if state == NO_HOLD and post_q:
        tail = post_q[-1]
        if (tail.get("state") == NO_HOLD and
            tail.get("station_code") == payload["station_code"] and
            tail.get("flowrack_name") == payload["flowrack_name"] and
            tail.get("row") == payload["row"] and
            tail.get("col") == payload["col"]):
            return  # ไม่ต้อง enqueue ซ้ำ

    # --- bounded queue: ถ้าเต็ม ให้ทิ้งตัวเก่าออกก่อน ---
    if len(post_q) >= MAX_POST_Q:
        drop_n = len(post_q) - MAX_POST_Q + 1
        for _ in range(drop_n):
            post_q.popleft()
        log(f"🗑️  post_q overflow — dropped {drop_n} oldest")

    post_q.append(payload)
    enqueued_count += 1
    log(f"➡️  ENQ  #{enqueued_count:06d}  {payload}")

async def network_watchdog(server_ip="192.168.1.13", iface="wlan0"):
    fail = 0
    while True:
        # quick ping with 1s deadline
        rc = await asyncio.create_subprocess_exec(
            "ping","-c","1","-W","1", server_ip,
            stdout=asyncio.subprocess.DEVNULL, stderr=asyncio.subprocess.DEVNULL
        )
        await rc.wait()
        if rc.returncode == 0:
            fail = 0
        else:
            fail += 1
            if fail >= 2:  # ~2s of failures
                log("📶 wifi down — nudging reconnect")
                # lightweight nudge; no root needed for reconnect command may vary by distro
                asyncio.create_task(asyncio.create_subprocess_exec("wpa_cli","-i",iface,"reconnect"))
                fail = 0
        await asyncio.sleep(1)

async def wifi_nudge_soft(iface="wlan0"):
    # quick, no-root action often enough to kick a roam
    try:
        await asyncio.create_subprocess_exec("wpa_cli","-i",iface,"reconnect")
    except Exception:
        pass

async def wifi_recover_hard(iface="wlan0", gateway="192.168.1.2"):
    # harder recover; may need sudo for some systems
    cmds = [
        ["wpa_cli","-i",iface,"reconfigure"],
        ["ip","link","set",iface,"down"],
        ["ip","link","set",iface,"up"],
        ["dhcpcd","-n",iface],
        ["ping","-c","1","-W","1",gateway],  # quick check
    ]
    for cmd in cmds:
        try:
            p = await asyncio.create_subprocess_exec(*cmd,
                    stdout=asyncio.subprocess.DEVNULL,
                    stderr=asyncio.subprocess.DEVNULL)
            await p.wait()
        except Exception:
            pass


async def post_worker(session, post_url, dry_run=False, iface="wlan0", gateway="192.168.1.2"):
    """
    Policy B: if NOT 200 or any exception, drop ALL queued events.
    Add escalation on consecutive failures: soft -> hard -> restart -> reboot.
    """
    global _consecutive_post_fails, posted_count
    last_post = 0.0
    while True:
        if not post_q:
            await asyncio.sleep(0.01)
            continue

        # rate-limit to <= POST_RPS
        now = time.monotonic()
        wait = (last_post + MIN_POST_INTERVAL) - now
        if wait > 0:
            await asyncio.sleep(wait)

        payload = post_q[0]

        # Dry-run: just print and drop
        if dry_run:
            print("📦 WOULD POST:", payload, flush=True)
            post_q.popleft()
            last_post = time.monotonic()
            _consecutive_post_fails = 0
            continue

        try:
            # Short timeout → faster failover
            async with session.post(post_url, json=payload, timeout=5) as r:
                _ = await r.text()
                if r.status == 200:
                    print("✅ POST 200", payload, flush=True)
                    posted_count += 1
                    post_q.popleft()
                    last_post = time.monotonic()
                    _consecutive_post_fails = 0
                else:
                    print(f"❌ POST {r.status} — drop ALL (Policy B)", flush=True)
                    print(f"[DROP] dropping {len(post_q)} events", flush=True)
                    post_q.clear()
                    last_post = time.monotonic()
                    _consecutive_post_fails += 1
        except Exception as e:
            print("❌ POST exception — drop ALL (Policy B):", repr(e), flush=True)
            print(f"[DROP] dropping {len(post_q)} events", flush=True)
            post_q.clear()
            last_post = time.monotonic()
            _consecutive_post_fails += 1
            await asyncio.sleep(0.5)  # tiny breather

        # === Escalation based on consecutive failures ===
        if _consecutive_post_fails == 3:
            print("📶 POST fails >=3 → soft Wi-Fi nudge", flush=True)
            asyncio.create_task(wifi_nudge_soft(iface))
        elif _consecutive_post_fails == 6:
            print("📶 POST fails >=6 → hard Wi-Fi recover", flush=True)
            asyncio.create_task(wifi_recover_hard(iface, gateway))
        elif _consecutive_post_fails == 10:
            print("🔁 POST fails >=10 → ask systemd to restart service", flush=True)
            try:
                await asyncio.create_subprocess_exec("systemctl","--user","restart","flowrack.service")
            except Exception:
                pass
        elif _consecutive_post_fails >= 20:
            print("🧯 POST fails >=20 → rebooting Pi", flush=True)
            await asyncio.create_subprocess_exec("sudo","/sbin/reboot")
async def get_wifi_metrics(iface="wlan0"):
    """Return (rssi_dbm:int|None, ssid:str|None)."""
    try:
        p = await asyncio.create_subprocess_exec(
            "iw","dev",iface,"link",
            stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.DEVNULL
        )
        out,_ = await p.communicate()
        text = out.decode("utf-8","ignore")
        if "Not connected" in text:
            return None, None
        rssi = None; ssid = None
        for line in text.splitlines():
            line = line.strip()
            if line.startswith("SSID:"):
                ssid = line.split("SSID:",1)[1].strip()
            elif line.startswith("signal:"):
                # e.g., "signal: -58 dBm"
                try:
                    rssi = int(line.split()[1])
                except Exception:
                    pass
        return rssi, ssid
    except Exception:
        return None, None

async def wifi_monitor(iface="wlan0", prefer_ssid=None, weak_threshold=-75):
    """Every 5s: if RSSI weak or not associated, nudge Wi-Fi; if wrong SSID, rejoin."""
    while True:
        rssi, ssid = await get_wifi_metrics(iface)
        if ssid is None:
            print("📶 Not associated — Wi-Fi reconnect", flush=True)
            asyncio.create_task(wifi_nudge_soft(iface))
        else:
            print(f"📶 Wi-Fi SSID={ssid} RSSI={rssi} dBm", flush=True)
            if prefer_ssid and ssid != prefer_ssid:
                print(f"📶 on {ssid} (not {prefer_ssid}) — nudge roam", flush=True)
                asyncio.create_task(wifi_nudge_soft(iface))
            elif rssi is not None and rssi <= weak_threshold:
                print(f"📶 weak RSSI {rssi} dBm — nudge roam", flush=True)
                asyncio.create_task(wifi_nudge_soft(iface))
        await asyncio.sleep(5)

def handle_change(ts_ms:int, idx:int, state:int, station_code:str):
    global edges_seen
    s = slots.get(idx)
    if not s:
        return
    prev = s.last_state
    if state == prev:
        return

    s.last_state = state
    edges_seen += 1
    if VERBOSE_EDGE:
        log(f"[EDGE] idx={idx:02d} {prev}->{state} tr={'SEND' if s.transfer==SLOT_SEND else 'RECV'} ts={ts_ms}")

    if s.transfer == SLOT_SEND:
        # SEND: count-up on 0 -> 1
        if prev == 0 and state == 1:
            enqueue(s.flowrack, COUNT_UP, s.row, s.col, station_code)

    else:  # RECEIVE
        if prev == 0 and state == 1:
            # Item flows in
            s.on_since = ts_ms
            s.off_since = None
            s.last_hold_sent_on_since = None
            s.last_no_hold_sent_at = None
            # spread safety NO_HOLD per slot (reduce burst)
            s.safety_period = SAFETY_CLEAR_MS + ((idx * 137) % 5000)
            enqueue(s.flowrack, ITEM_IN, s.row, s.col, station_code)

        elif prev == 1 and state == 0:
            # Clear immediately on leaving sensor
            s.off_since = ts_ms
            s.on_since = None
            s.last_hold_sent_on_since = None
            enqueue(s.flowrack, NO_HOLD, s.row, s.col, station_code)
            s.last_no_hold_sent_at = ts_ms

def periodic_rules(ts_ms:int, station_code:str):
    for idx, s in slots.items():
        if s.transfer != SLOT_RECEIVE:
            continue

        # HOLD once per ON stretch
        if s.last_state == 1 and s.on_since is not None:
            if (ts_ms - s.on_since) >= HOLD_MS:
                if s.last_hold_sent_on_since != s.on_since:
                    log(f"[HOLD] idx={idx:02d} row={s.row} col={s.col} since={s.on_since} now={ts_ms}")
                    enqueue(s.flowrack, HOLD, s.row, s.col, station_code)
                    s.last_hold_sent_on_since = s.on_since

        # Safety NO_HOLD while OFF, every safety_period
        if s.last_state == 0:
            last = s.last_no_hold_sent_at
            per = s.safety_period or SAFETY_CLEAR_MS
            if last is None or (ts_ms - last) >= per:
                log(f"[SAFE] NO_HOLD idx={idx:02d} row={s.row} col={s.col} period={per}ms")
                enqueue(s.flowrack, NO_HOLD, s.row, s.col, station_code)
                s.last_no_hold_sent_at = ts_ms

def apply_snapshot(ts_ms:int, low16:int, high16:int, station_code:str):
    global snapshots_seen
    snapshots_seen += 1
    combined = ((high16 & 0xFFFF) << 16) | (low16 & 0xFFFF)
    if VERBOSE_SNAPSHOT:
        log(f"[SNAP] low=0x{low16:04X} high=0x{high16:04X} t={ts_ms}")

    for i in range(32):
        bit = 1 if ((combined >> i) & 0x1) else 0
        idx = i + 1
        s = slots.get(idx)
        if not s:
            continue

        if s.last_state != bit:
            if VERBOSE_SNAPSHOT:
                log(f"[REPAIR] idx={idx:02d} last={s.last_state} snap={bit} -> synth edge at t={ts_ms}")
            handle_change(ts_ms, idx, bit, station_code)

        # Ensure timers are plausible if we just learned state from snapshot
        if bit == 1:
            if s.on_since is None:
                s.on_since = ts_ms
                s.off_since = None
        else:
            if s.off_since is None:
                s.off_since = ts_ms


def serial_reader_thread(port: str, baud: int):
    global _ser, _last_esp_seen_ms, _main_loop, ingest_q
    while True:
        try:
            with _ser_lock:
                _ser = serial.Serial(port, baud, timeout=1)
                try:
                    _ser.dtr = False
                    _ser.rts = False
                except Exception:
                    pass
            log(f"📟 Serial opened: {port} @ {baud}")
            while True:
                line = _ser.readline()
                if not line:
                    continue
                _last_esp_seen_ms = int(time.time()*1000)
                try:
                    text = line.decode("utf-8", "ignore").strip()
                    if not text:
                        continue
                    if len(text) > 256:
                        # ignore pathological / garbled lines
                        continue
                    if VERBOSE_SERIAL:
                        log(f"[SER] {text}")
                    obj = json.loads(text)
                    if _main_loop and ingest_q is not None:
                        _main_loop.call_soon_threadsafe(_ingest_try_put, obj)
                except json.JSONDecodeError:
                    continue
        except Exception as e:
            log("⚠️ Serial error, retry in 2s:", repr(e))
            time.sleep(2)


def try_reset_esp():
    # Toggle DTR/RTS to reset (works on most NodeMCU-style boards)
    with _ser_lock:
        if _ser is None:
            return
        try:
            _ser.dtr = False
            _ser.rts = True
            time.sleep(0.12)
            _ser.dtr = True
            _ser.rts = False
            time.sleep(0.12)
            _ser.dtr = False
            _ser.rts = False
        except Exception as e:
            log("reset toggle failed:", repr(e))

async def esp_watchdog():
    global _esp_resets
    while True:
        now = int(time.time()*1000)
        silent = now - _last_esp_seen_ms
        if silent > ESP_TIMEOUT_MS:
            _esp_resets += 1
            log(f"⚠️ ESP silent {silent}ms, reset try #{_esp_resets}")
            try_reset_esp()
            await asyncio.sleep(2.0)
            # escalate after several tries: close & reopen port (reader thread will reopen)
            if _esp_resets >= ESP_MAX_RESETS:
                log("❌ Still silent; forcing serial reopen")
                with _ser_lock:
                    try:
                        if _ser:
                            _ser.close()
                    except Exception:
                        pass
                _esp_resets = 0
        else:
            _esp_resets = 0

        # <<< ADD HERE (before final sleep)
        if (now // 10000) != ((now-1000) // 10000):
            log(f"[HEALTH] edges={edges_seen} snaps={snapshots_seen} "
                f"enq={enqueued_count} posted={posted_count} q={len(post_q)}")

        await asyncio.sleep(1.0)


async def load_mapping(session, get_url:str):
    async with session.get(get_url, timeout=10) as r:
        r.raise_for_status()
        doc = await r.json()
    mapping = {}
    for fr in doc.get("data", []):
        name = fr["name"]
        slot_style      = fr["slot_style"]
        slot_transfer   = fr["slot_transfer_type"]
        slot_read_index = fr["slot_read_index"]
        rows = len(slot_style)
        cols = len(slot_style[0]) if rows else 0
        for rr in range(rows):
            for cc in range(cols):
                if int(slot_style[rr][cc]) == 0:
                    continue
                idx = int(slot_read_index[rr][cc])
                transfer = SLOT_RECEIVE if str(slot_transfer[rr][cc]).lower()=="receive" else SLOT_SEND
                mapping[idx] = (name, rr, cc, transfer)
    return mapping

def build_argparser():
    ap = argparse.ArgumentParser()
    ap.add_argument("--serial", default="/dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--server-ip", default="192.168.1.13")
    ap.add_argument("--server-port", type=int, default=4499)
    ap.add_argument("--station-code", type=str, default="1")
    ap.add_argument("--dry-run", action="store_true", help="print payloads instead of POST")
    ap.add_argument("--wifi-iface", default="wlan0")
    ap.add_argument("--prefer-ssid", default=None)
    ap.add_argument("--weak-threshold", type=int, default=-75)
    ap.add_argument("--gateway", default="192.168.1.2")

    return ap

async def main():
    args = build_argparser().parse_args()
    base = f"http://{args.server_ip}:{args.server_port}"
    GET_URL  = f"{base}/station/initial-flowrack-name?station_code={args.station_code}"
    POST_URL = f"{base}/station/item_counter"
    
    global ingest_q, _main_loop
    _main_loop = asyncio.get_running_loop()
    ingest_q = asyncio.Queue(maxsize=2000)

    async def serial_consumer():
        global _pending_hp, current_station_code
        while True:
            obj = await ingest_q.get()
            ts_ms = int(time.time()*1000)
            try:
                if "hp" in obj:
                    hp = str(int(obj["hp"]))
                    log(f"🆔 HP from ESP = {hp} (will reload mapping)")
                    _pending_hp = hp
                elif "snapshot" in obj:
                    low = int(obj["snapshot"][0]); high = int(obj["snapshot"][1])
                    apply_snapshot(ts_ms, low, high, current_station_code or args.station_code)
                else:
                    idx = int(obj.get("idx", 0)); st = int(obj.get("state", 0))
                    if 1 <= idx <= 32:
                        handle_change(ts_ms, idx, st, current_station_code or args.station_code)
            except Exception as e:
                log("[PARSE] bad line:", repr(e), obj)
            finally:
                ingest_q.task_done()   


    
    # Start serial reader thread; it will push to ingest_q
    th = threading.Thread(target=serial_reader_thread, args=(args.serial, args.baud), daemon=True)
    th.start()

    # Start consumer
    asyncio.create_task(serial_consumer())
    # Networking session
    conn = aiohttp.TCPConnector(
        limit=8,
        limit_per_host=8,
        ttl_dns_cache=300,
        keepalive_timeout=15,
        force_close=False
    )
    timeout = aiohttp.ClientTimeout(total=10, connect=5, sock_connect=5, sock_read=5)
    async with aiohttp.ClientSession(connector=conn, timeout=timeout) as session:
        asyncio.create_task(wifi_monitor(iface=args.wifi_iface, prefer_ssid=args.prefer_ssid, weak_threshold=args.weak_threshold))
        asyncio.create_task(network_watchdog(server_ip=args.server_ip, iface=args.wifi_iface))

        # helper: load mapping for a station code
        async def load_for_station_code(st_code:str):
            base = f"http://{args.server_ip}:{args.server_port}"
            get_url = f"{base}/station/initial-flowrack-name?station_code={st_code}"
            mapping = await load_mapping(session, get_url)
            return mapping

        # initial load (from CLI)
        global current_station_code
        current_station_code = str(args.station_code)
        try:
            mapping = await load_for_station_code(current_station_code)
            if not mapping:
                log("❌ Empty mapping from server"); return
            slots.clear()
            for idx,(fr,r,c,t) in mapping.items():
                slots[idx] = SlotState(transfer=t, row=r, col=c, flowrack=fr)
            log(f"✅ Mapping loaded for station_code={current_station_code}: {len(slots)} active sensors")
            for idx,(fr,r,c,t) in sorted(mapping.items()):
                kind = "RECV" if t==SLOT_RECEIVE else "SEND"
                log(f"  - idx={idx:02d}  {kind}  fr={fr}  row={r} col={c}")
        except Exception as e:
            log("❌ Failed to load mapping:", repr(e))
            return

        # Start tasks
        
        asyncio.create_task(post_worker(
            session, POST_URL, dry_run=args.dry_run,
            iface=args.wifi_iface, gateway=args.gateway
        ))
        asyncio.create_task(esp_watchdog())

        # Periodic rules loop (50 Hz) + hot-reload when ESP announces HP
        try:
            while True:
                # hot-reload mapping if ESP sent a new HP
                if _pending_hp is not None and _pending_hp != current_station_code:
                    try:
                        log(f"🔄 Reload mapping for station_code={_pending_hp}")
                        newmap = await load_for_station_code(_pending_hp)
                        if newmap:
                            slots.clear()
                            for idx,(fr,r,c,t) in newmap.items():
                                slots[idx] = SlotState(transfer=t, row=r, col=c, flowrack=fr)
                            current_station_code = _pending_hp
                            log(f"✅ Mapping reloaded: station_code={current_station_code} slots={len(slots)}")
                            for idx,(fr,r,c,t) in sorted(newmap.items()):
                                kind = "RECV" if t==SLOT_RECEIVE else "SEND"
                                log(f"  - idx={idx:02d}  {kind}  fr={fr}  row={r} col={c}")
                        else:
                            log("⚠️ New mapping was empty; keeping old mapping")
                    except Exception as e:
                        log("❌ Reload failed:", repr(e))
                    finally:
                        # consume once
                        _pending_hp = None

                ts_ms = int(time.time()*1000)
                periodic_rules(ts_ms, current_station_code)
                await asyncio.sleep(0.02)
        except KeyboardInterrupt:
            log("bye")

if __name__ == "__main__":
    if sys.platform.startswith("win"):
        # Windows users: py -3 flowrack_pi.py --serial COM3 --dry-run
        pass
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass

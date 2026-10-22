"""Passage complet avec le vrai code spot_doors contre le simulateur du module (HTTP réel)."""
import json, os, subprocess, sys, time, shutil
from pathlib import Path
ROOT = Path("/home/claude/spot_doors_v2"); sys.path.insert(0, str(ROOT))
TOKEN = "demo-token-soda-2026-abcdef"; PORT = 8091
os.environ["SPOT_DOOR_TOKEN"] = TOKEN
from spot_doors.config import DoorConfig, VisionConfig
from spot_doors.transport import HttpDoorTransport, DoorTransportError
from spot_doors.vision import NodeInputObserver
from spot_doors.controller import DoorPassageController
from spot_doors.journal import Journal

logdir = Path("/home/claude/wifi/run/logs"); shutil.rmtree(logdir, ignore_errors=True)
sim_log = open("/home/claude/wifi/run/simulateur.log", "w")
sim = subprocess.Popen([sys.executable, str(ROOT / "tools/door_simulator.py"), "--door-id", "P01", "--port", str(PORT),
                        "--open-time", "4", "--hold", "25"], stdout=sim_log, stderr=subprocess.STDOUT, env=dict(os.environ))
time.sleep(1.0)
t0 = time.time()
try:
    door = DoorConfig(door_id="P01", name="Porte pilote (simulée)", host="127.0.0.1", port=PORT,
                      open_timeout_s=20.0, hold_open_s=25.0, crossing_time_s=8.0, safety_margin_s=5.0,
                      vision=VisionConfig(stable_free_s=1.0, period_s=0.2))
    tr = HttpDoorTransport()
    st = tr.status(door)
    print("STATUS", st)
    ctrl = DoorPassageController(door, tr, NodeInputObserver(tr, door), Journal(logdir))
    res = ctrl.open_and_confirm(); print("OPEN_AND_CONFIRM", res)
    res2 = ctrl.ensure_time_to_cross(); print("ENSURE", res2)
    st2 = tr.status(door); print("STATUS_APRES", st2)
    # Essai mauvais jeton
    os.environ["SPOT_DOOR_TOKEN"] = "mauvais-jeton-0000000000"
    before = st2.pulse_count
    try:
        tr.open(door); print("BAD_TOKEN: ACCEPTED ?!")
    except DoorTransportError as e:
        print("BAD_TOKEN_REFUSED", e)
    os.environ["SPOT_DOOR_TOKEN"] = TOKEN
    st3 = tr.status(door); print("STATUS_APRES_MAUVAIS_JETON pulse_count", before, "->", st3.pulse_count)
finally:
    print("T_TOTAL", round(time.time() - t0, 2)); print("T0", t0)
    sim.terminate(); sim.wait(5); sim_log.close()

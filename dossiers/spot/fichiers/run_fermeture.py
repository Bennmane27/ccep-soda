"""Passage + fermeture commandée avec le vrai code spot_doors contre le simulateur (HTTP réel)."""
import json, os, subprocess, sys, time, shutil
from pathlib import Path
ROOT = Path("/home/claude/spot_doors_v2"); sys.path.insert(0, str(ROOT))
TOKEN = "demo-token-soda-2026-abcdef"; PORT = 8092
os.environ["SPOT_DOOR_TOKEN"] = TOKEN
from spot_doors.config import DoorConfig, VisionConfig
from spot_doors.transport import HttpDoorTransport
from spot_doors.vision import NodeInputObserver
from spot_doors.controller import DoorPassageController
from spot_doors.journal import Journal

out = Path("/home/claude/wifi/run_close"); logdir = out / "logs"; shutil.rmtree(logdir, ignore_errors=True)
sim_log = open(out / "simulateur.log", "w")
sim = subprocess.Popen([sys.executable, str(ROOT / "tools/door_simulator.py"), "--door-id", "P06", "--port", str(PORT),
                        "--open-time", "4", "--close-mode", "commande", "--close-time", "6"],
                       stdout=sim_log, stderr=subprocess.STDOUT, env=dict(os.environ))
time.sleep(1.0)
try:
    vis = VisionConfig(method="input", stable_free_s=1.0, period_s=0.2)
    door = DoorConfig(door_id="P06", name="Porte à fermeture commandée (simulée)", host="127.0.0.1", port=PORT,
                      open_timeout_s=20.0, hold_open_s=25.0, crossing_time_s=8.0, safety_margin_s=5.0,
                      close_mode="commande", close_time_s=6.0, closed_stable_s=2.0,
                      vision=vis, close_vision=vis)
    tr = HttpDoorTransport()
    obs = NodeInputObserver(tr, door)
    j = Journal(logdir)
    ctrl = DoorPassageController(door, tr, obs, j, close_observer=obs)
    print("OPEN", ctrl.open_and_confirm()); print("ENSURE", ctrl.ensure_time_to_cross())
    time.sleep(3.0)  # franchissement (raccourci) jusqu'à la zone X
    print("STATUS_AVANT_FERMETURE", tr.status(door))
    t_zone = time.time()
    res = ctrl.close_and_confirm(); print("CLOSE", res)
    print("STATUS_APRES", tr.status(door))
    print("T_ZONE", t_zone)
finally:
    sim.terminate(); sim.wait(5); sim_log.close()

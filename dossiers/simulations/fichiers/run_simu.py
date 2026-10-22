"""Simulation ngspice du redresseur du boîtier de porte (Projet Soda).
24 V AC armoire -> fusible F1 -> câble 5 m -> pont GBU4G -> C1 470 µF 63 V -> carte ESP32-RELAY02 (7-60 V).
"""
import subprocess, json, math, os
import numpy as np

DIODE = ".model DGBU D(IS=1.4n N=1.6 RS=0.025 BV=400 CJO=50p)"  # Vf ~0,75 V à 0,1 A ; ~1,0 V à 4 A (fiche GBU4G : 1,0 V max à 4 A)

def netlist(vrms, r_tr, p_load, t_on_deg=90.0, tstop=0.3, tstep=20e-6, c_uF=470, esr=0.15):
    vpk = vrms * math.sqrt(2)
    f = 50.0
    # mise sous tension à l'angle t_on_deg (90° = crête : pire courant d'appel)
    td = 0.0
    phase = t_on_deg
    return f"""* redresseur boitier porte
.option reltol=1e-4
Vs ac1 0 SIN(0 {vpk:.4f} {f} {td} 0 {phase})
Rtr ac1 a2 {r_tr}
Ltr a2 a3 2m
Rf a3 a4 0.12
Rw a4 p1 0.18
Rw2 n1 0 0.18
D1 p1 dcp DGBU
D2 n1 dcp DGBU
D3 dcn p1 DGBU
D4 dcn n1 DGBU
{DIODE}
C1 dcp c1x {c_uF}u IC=0
Resr c1x dcn {esr}
* charge : carte à puissance constante (convertisseur 7-60 V), coupée sous 6 V
Bload dcp dcn I = {p_load}/max(V(dcp,dcn),6)
.tran {tstep} {tstop} 0 {tstep} UIC
.control
run
wrdata out.txt V(dcp,dcn) I(Vs)
.endc
.end
"""

def run(case, tag):
    cir=f"c_{tag}.cir"; out=f"out_{tag}.txt"
    with open(cir, "w") as fh:
        fh.write(netlist(**case["p"]).replace("out.txt", out))
    r = subprocess.run(["ngspice", "-b", cir], capture_output=True, text=True)
    if not os.path.exists(out):
        print(r.stdout[-2000:], r.stderr[-2000:]); raise SystemExit
    d = np.loadtxt(out)
    os.remove(out)
    t, v, i = d[:, 0], d[:, 1], -d[:, 3]   # courant sortant de la source = courant du fusible
    return t, v, i

def stats(t, v, i, tstop):
    ss = t > tstop - 0.1                 # 5 dernières périodes
    vmax, vmin = v[ss].max(), v[ss].min()
    irms_ss = math.sqrt(np.trapezoid(i[ss] ** 2, t[ss]) / (t[ss][-1] - t[ss][0]))
    w = t < 0.02                         # première période : courant d'appel
    i2t = float(np.trapezoid(i[w] ** 2, t[w]))
    return dict(vmoy=float(v[ss].mean()), vmax=float(vmax), vmin=float(vmin), ondulation=float(vmax - vmin),
                ipk_appel=float(np.abs(i[w]).max()), i2t_appel=i2t, irms_regime=irms_ss)

cases = [
  # nom, Vrms secondaire, R transfo, puissance carte
  dict(nom="Nominal : 24 V AC, carte au repos (1 W)", p=dict(vrms=24.0, r_tr=1.0, p_load=1.0)),
  dict(nom="Pointe : 24 V AC, Wi-Fi en émission + relais collé (2,6 W)", p=dict(vrms=24.0, r_tr=1.0, p_load=2.6)),
  dict(nom="Transfo peu chargé : 26,4 V AC (+10 %), carte au repos", p=dict(vrms=26.4, r_tr=1.0, p_load=1.0)),
  dict(nom="Pire cas haut : 29 V AC (transfo peu chargé + secteur +10 %), carte presque à vide (0,3 W)", p=dict(vrms=29.0, r_tr=1.0, p_load=0.3)),
  dict(nom="Pire cas bas : 21,6 V AC (secteur −10 %), pointe 2,6 W", p=dict(vrms=21.6, r_tr=1.0, p_load=2.6)),
  dict(nom="Courant d'appel pire cas : mise sous tension à la crête, 29 V AC, transfo puissant (0,3 Ω)", p=dict(vrms=29.0, r_tr=0.3, p_load=1.0, t_on_deg=90.0)),
]
res = []
import sys
sel=[int(x) for x in sys.argv[1:]] or list(range(len(cases)))
for k,c in enumerate(cases):
    if k not in sel: continue
    t, v, i = run(c, k)
    s = stats(t, v, i, c["p"].get("tstop", 0.3))
    s["nom"] = c["nom"]; s["params"] = c["p"]
    res.append(s)
    np.savez_compressed(f"trace_{k+1}.npz", t=t, v=v, i=i)
    print(f"{c['nom']}\n   Vmoy={s['vmoy']:.2f} V  Vmax={s['vmax']:.2f}  Vmin={s['vmin']:.2f}  ondulation={s['ondulation']:.2f} V  "
          f"Ipk appel={s['ipk_appel']:.1f} A  I2t={s['i2t_appel']:.3f} A2s  Irms régime={s['irms_regime']*1000:.0f} mA")
json.dump(res, open("resultats_"+"_".join(map(str,sel))+".json", "w"), ensure_ascii=False, indent=1)

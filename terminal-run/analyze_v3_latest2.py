import json, statistics

log_file = '/home/bluedstar/SimpleRobot/terminal-run/plot_logger/cascade_unified_2026-07-29_16-41-00/raw_state.jsonl'
lines = []
with open(log_file, 'r') as f:
    for line in f:
        if line.strip():
            try:
                d = json.loads(line)
                if 'state' in d: lines.append(d['state'])
            except: pass

print(f"Frames: {len(lines)}")
if not lines: exit(0)
print(f"Version: {lines[0].get('version')}")

all_v    = [l.get('v_cmd',0) for l in lines]
all_vdes = [l.get('v_des', l.get('v_target',0)) for l in lines]
all_ex   = [l.get('epsilon_x_mm',0) for l in lines]
all_th   = [l.get('theta_rad',0) for l in lines]
all_om   = [l.get('omega_cmd',0) for l in lines]
all_cs   = [l.get('curve_severity',0) for l in lines]
all_kap  = [l.get('kappa_m',0) for l in lines]
all_ff   = [l.get('omega_ff',0) for l in lines]
all_fb   = [l.get('omega_fb',0) for l in lines]
fps_vals = [l.get('fps_est',0) for l in lines if l.get('fps_est',0) > 0]

print(f"\n=== FPS ===")
if fps_vals: print(f"fps: avg={statistics.mean(fps_vals):.1f}  min={min(fps_vals):.1f}  max={max(fps_vals):.1f}")

print(f"\n=== SPEED PROFILE ===")
print(f"v_cmd:  min={min(all_v):.3f}  max={max(all_v):.3f}  avg={statistics.mean(all_v):.3f}")
print(f"v_des:  min={min(all_vdes):.3f}  max={max(all_vdes):.3f}  avg={statistics.mean(all_vdes):.3f}")
print(f"High speed (>0.28) frames: {sum(1 for v in all_v if v > 0.28)} / {len(lines)}")

abs_ex = [abs(x) for x in all_ex]
abs_th = [abs(x) for x in all_th]
print(f"\n=== ERROR PROFILE ===")
print(f"e_x_mm: avg={statistics.mean(abs_ex):.1f}  max={max(abs_ex):.1f}  p90={sorted(abs_ex)[int(len(abs_ex)*0.90)]:.1f}")
print(f"theta:  avg={statistics.mean(abs_th):.3f}  max={max(abs_th):.3f}  p90={sorted(abs_th)[int(len(abs_th)*0.90)]:.3f}")

print(f"\n=== CURVE SEVERITY ===")
print(f"cs: avg={statistics.mean(all_cs):.3f}  max={max(all_cs):.3f}  p90={sorted(all_cs)[int(len(all_cs)*0.90)]:.3f}")
print(f"kappa max_abs={max(abs(k) for k in all_kap):.3f}")

# Find v drop around curve entry
print(f"\n=== THẲNG→CUA PHANH ANALYSIS (entry_v > 0.22, then error > 50mm) ===")
for i in range(3, len(lines)):
    pv  = lines[i-3].get('v_cmd',0)
    cv  = lines[i].get('v_cmd',0)
    cs  = lines[i].get('curve_severity',0)
    ex  = abs(lines[i].get('epsilon_x_mm',0))
    if pv > 0.22 and cs > 0.45 and ex > 50:
        vd  = lines[i].get('v_des',lines[i].get('v_target',0))
        th  = lines[i].get('theta_rad',0)
        om  = lines[i].get('omega_cmd',0)
        om_lim = lines[i].get('omega_limit',0)
        kap = lines[i].get('kappa_m',0)
        fps = lines[i].get('fps_est',0)
        print(f"  fr{i}: prev_v={pv:.3f} → cs={cs:.2f} kap={kap:.2f} | ex={ex:.0f}mm th={th:.3f} v={cv:.3f}(des={vd:.3f}) om={om:.3f}(lim={om_lim:.3f}) fps={fps:.1f}")

# Oscillation events
print(f"\n=== OSCILLATION EVENTS ===")
events = []
in_ev = False; ev = []
for l in lines:
    if abs(l.get('epsilon_x_mm',0)) > 40 or abs(l.get('theta_rad',0)) > 0.15:
        if not in_ev: in_ev=True; ev=[]
        ev.append(l)
    else:
        if in_ev: events.append(ev); in_ev=False
if in_ev: events.append(ev)
print(f"Found {len(events)} events")

for i, ev in enumerate(events[:7]):
    max_ex = max(abs(e.get('epsilon_x_mm',0)) for e in ev)
    max_th = max(abs(e.get('theta_rad',0)) for e in ev)
    max_cs = max(e.get('curve_severity',0) for e in ev)
    avg_v  = statistics.mean(e.get('v_cmd',0) for e in ev)
    entry_v = ev[0].get('v_cmd',0)
    omegas = [e.get('omega_cmd',0) for e in ev]
    flips = sum(1 for a,b in zip(omegas, omegas[1:]) if a*b < 0)
    print(f"\n  Ev{i+1}({len(ev)}fr,{flips}flip) entry_v={entry_v:.3f} avg_v={avg_v:.3f} max_ex={max_ex:.0f}mm max_th={max_th:.3f} max_cs={max_cs:.2f}")
    for j, e in enumerate(ev):
        if j % max(1,len(ev)//5)==0 or j==len(ev)-1:
            vd = e.get('v_des', e.get('v_target',0))
            print(f"    [{j}] ex={e.get('epsilon_x_mm',0):6.1f} th={e.get('theta_rad',0):6.3f} v={e.get('v_cmd',0):.3f}(des={vd:.3f}) om={e.get('omega_cmd',0):.3f} ff={e.get('omega_ff',0):.3f} cs={e.get('curve_severity',0):.2f} kap={e.get('kappa_m',0):.2f}")


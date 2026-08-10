import json, statistics

log_file = '/home/bluedstar/SimpleRobot/terminal-run/plot_logger/cascade_unified_2026-07-29_16-30-26/raw_state.jsonl'

lines = []
with open(log_file, 'r') as f:
    for line in f:
        if line.strip():
            try:
                data = json.loads(line)
                if 'state' in data:
                    lines.append(data['state'])
            except: pass

print(f"Total frames: {len(lines)}")
if not lines: exit(0)

print(f"Version: {lines[0].get('version')}")

# Overall stats
all_v    = [l.get('v_cmd', 0) for l in lines]
all_ex   = [l.get('epsilon_x_mm', 0) for l in lines]
all_th   = [l.get('theta_rad', 0) for l in lines]
all_om   = [l.get('omega_cmd', 0) for l in lines]
all_cs   = [l.get('curve_severity', 0) for l in lines]
all_kap  = [l.get('kappa_m', 0) for l in lines]
all_vdes = [l.get('v_des', l.get('v_target', 0)) for l in lines]
all_ff   = [l.get('omega_ff', 0) for l in lines]
all_fb   = [l.get('omega_fb', 0) for l in lines]

print(f"\n=== SPEED PROFILE ===")
print(f"v_cmd:  min={min(all_v):.3f}  max={max(all_v):.3f}  avg={statistics.mean(all_v):.3f}")
print(f"v_des:  min={min(all_vdes):.3f}  max={max(all_vdes):.3f}  avg={statistics.mean(all_vdes):.3f}")
print(f"High speed (>0.30) frames: {sum(1 for v in all_v if v > 0.30)} / {len(lines)}")

print(f"\n=== ERROR PROFILE ===")
abs_ex = [abs(x) for x in all_ex]
abs_th = [abs(x) for x in all_th]
print(f"e_x_mm: avg={statistics.mean(abs_ex):.1f}  max={max(abs_ex):.1f}  p90={sorted(abs_ex)[int(len(abs_ex)*0.90)]:.1f}")
print(f"theta:  avg={statistics.mean(abs_th):.3f}  max={max(abs_th):.3f}  p90={sorted(abs_th)[int(len(abs_th)*0.90)]:.3f}")

print(f"\n=== CURVE PROFILE ===")
print(f"curve_severity: avg={statistics.mean(all_cs):.3f}  max={max(all_cs):.3f}  p90={sorted(all_cs)[int(len(all_cs)*0.90)]:.3f}")
print(f"kappa: max_abs={max(abs(k) for k in all_kap):.3f}")

print(f"\n=== STEERING ===")
abs_om = [abs(x) for x in all_om]
abs_ff = [abs(x) for x in all_ff]
abs_fb = [abs(x) for x in all_fb]
print(f"omega_cmd: avg={statistics.mean(abs_om):.3f}  max={max(abs_om):.3f}")
print(f"omega_ff:  avg={statistics.mean(abs_ff):.3f}  max={max(abs_ff):.3f}")
print(f"omega_fb:  avg={statistics.mean(abs_fb):.3f}  max={max(abs_fb):.3f}")

# --- KEY PROBLEM: straight→curve transitions ---
print(f"\n=== STRAIGHT→CURVE TRANSITIONS (PHANH VÀO CUA) ===")
# Find frames where v was high (>0.25) and then severity jumped
for i in range(2, len(lines)):
    prev_v  = lines[i-2].get('v_cmd', 0)
    curr_cs = lines[i].get('curve_severity', 0)
    curr_v  = lines[i].get('v_cmd', 0)
    prev_cs = lines[i-2].get('curve_severity', 0)
    # Detect: was fast, now in strong curve
    if prev_v > 0.25 and curr_cs > 0.50 and abs(lines[i].get('epsilon_x_mm',0)) > 40:
        ex = lines[i].get('epsilon_x_mm', 0)
        th = lines[i].get('theta_rad', 0)
        vd = lines[i].get('v_des', lines[i].get('v_target', 0))
        omega_lim = lines[i].get('omega_limit', 0)
        print(f"  Frame {i}: prev_v={prev_v:.3f} -> cs={curr_cs:.2f} | ex={ex:.1f}mm th={th:.3f} v={curr_v:.3f} v_des={vd:.3f} om_lim={omega_lim:.3f}")

# Zigzag events
print(f"\n=== OSCILLATION EVENTS ===")
events = []
in_ev = False
ev = []
for l in lines:
    ex = abs(l.get('epsilon_x_mm', 0))
    th = abs(l.get('theta_rad', 0))
    if ex > 40 or th > 0.15:
        if not in_ev: in_ev = True; ev = []
        ev.append(l)
    else:
        if in_ev: events.append(ev); in_ev = False
if in_ev: events.append(ev)

print(f"Found {len(events)} events")
for i, ev in enumerate(events[:8]):
    max_ex = max(abs(e.get('epsilon_x_mm',0)) for e in ev)
    max_th = max(abs(e.get('theta_rad',0)) for e in ev)
    max_cs = max(e.get('curve_severity',0) for e in ev)
    avg_v  = statistics.mean(e.get('v_cmd',0) for e in ev)
    entry_v = ev[0].get('v_cmd',0)
    omegas = [e.get('omega_cmd',0) for e in ev]
    flips = sum(1 for a,b in zip(omegas, omegas[1:]) if a*b < 0)
    om_lim = min(e.get('omega_limit',99) for e in ev)
    print(f"\n  Ev{i+1} ({len(ev)}fr, {flips}flip) entry_v={entry_v:.3f} avg_v={avg_v:.3f} max_ex={max_ex:.0f}mm max_th={max_th:.3f} max_cs={max_cs:.2f} om_lim={om_lim:.3f}")
    for j, e in enumerate(ev):
        if j % max(1, len(ev)//4) == 0 or j == len(ev)-1:
            vdes = e.get('v_des', e.get('v_target',0))
            print(f"    [{j}] ex={e.get('epsilon_x_mm',0):6.1f} th={e.get('theta_rad',0):6.3f} v={e.get('v_cmd',0):.3f}(des={vdes:.3f}) om={e.get('omega_cmd',0):.3f} ff={e.get('omega_ff',0):.3f} cs={e.get('curve_severity',0):.2f}")


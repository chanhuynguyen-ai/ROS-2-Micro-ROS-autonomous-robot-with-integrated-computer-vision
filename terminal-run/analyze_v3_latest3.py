import json, statistics

log_file = '/home/bluedstar/SimpleRobot/terminal-run/plot_logger/cascade_unified_2026-07-30_14-29-11/raw_state.jsonl'
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

# Look for large errors > 150mm where speed increases too soon
print(f"\n=== PREMATURE ACCELERATION INCIDENTS ===")
for i in range(2, len(lines)):
    v_prev2 = lines[i-2].get('v_cmd', 0)
    v_curr = lines[i].get('v_cmd', 0)
    ex_curr = abs(lines[i].get('epsilon_x_mm', 0))
    th_curr = abs(lines[i].get('theta_rad', 0))
    if v_curr > v_prev2 and v_curr > 0.20 and (ex_curr > 100 or th_curr > 0.3):
        om = lines[i].get('omega_cmd',0)
        cs = lines[i].get('curve_severity',0)
        print(f"  Frame {i}: Accelerated {v_prev2:.3f} -> {v_curr:.3f} while error is HIGH (ex={ex_curr:.0f}mm, th={th_curr:.2f}, cs={cs:.2f})")

# Look for initial off-center turn (large ex, small v)
print(f"\n=== OFF-CENTER EVENTS ===")
events = []
in_ev = False; ev = []
for l in lines:
    if abs(l.get('epsilon_x_mm',0)) > 60 or abs(l.get('theta_rad',0)) > 0.25:
        if not in_ev: in_ev=True; ev=[]
        ev.append(l)
    else:
        if in_ev: events.append(ev); in_ev=False
if in_ev: events.append(ev)

for i, ev in enumerate(events[:5]):
    max_ex = max(abs(e.get('epsilon_x_mm',0)) for e in ev)
    max_th = max(abs(e.get('theta_rad',0)) for e in ev)
    max_cs = max(e.get('curve_severity',0) for e in ev)
    avg_v  = statistics.mean(e.get('v_cmd',0) for e in ev)
    entry_v = ev[0].get('v_cmd',0)
    print(f"\n  Ev{i+1}({len(ev)}fr) entry_v={entry_v:.3f} avg_v={avg_v:.3f} max_ex={max_ex:.0f}mm max_th={max_th:.3f} max_cs={max_cs:.2f}")
    for j, e in enumerate(ev):
        if j % max(1,len(ev)//4)==0 or j==len(ev)-1:
            vd = e.get('v_des', e.get('v_target',0))
            print(f"    [{j:3d}] ex={e.get('epsilon_x_mm',0):6.1f} th={e.get('theta_rad',0):6.3f} v={e.get('v_cmd',0):.3f}(des={vd:.3f}) om={e.get('omega_cmd',0):.3f} ff={e.get('omega_ff',0):.3f} cs={e.get('curve_severity',0):.2f}")


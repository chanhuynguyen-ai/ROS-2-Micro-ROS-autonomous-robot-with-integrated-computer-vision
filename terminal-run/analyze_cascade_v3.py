import json, statistics

log_file = '/home/bluedstar/SimpleRobot/terminal-run/plot_logger/cascade_unified_2026-07-29_16-20-55/raw_state.jsonl'

lines = []
with open(log_file, 'r') as f:
    for line in f:
        if line.strip():
            try:
                data = json.loads(line)
                if 'state' in data:
                    lines.append(data['state'])
            except:
                pass

print(f"Total frames: {len(lines)}")
if not lines:
    exit(0)

# Overall stats
all_ex = [abs(l.get('epsilon_x_mm', 0)) for l in lines]
all_theta = [abs(l.get('theta_rad', 0)) for l in lines]
all_omega_cmd = [abs(l.get('omega_cmd', 0)) for l in lines]
all_v_cmd = [l.get('v_cmd', 0) for l in lines]
all_omega_ff = [l.get('omega_ff', 0) for l in lines]
all_omega_fb = [l.get('omega_fb', 0) for l in lines]
all_curve_sev = [l.get('curve_severity', 0) for l in lines]
all_kappa = [l.get('kappa_m', 0) for l in lines]

print(f"\n=== OVERALL STATS ===")
print(f"v_cmd:   min={min(all_v_cmd):.3f}  max={max(all_v_cmd):.3f}  avg={statistics.mean(all_v_cmd):.3f}")
print(f"e_x_mm:  avg={statistics.mean(all_ex):.1f}  max={max(all_ex):.1f}  p90={sorted(all_ex)[int(len(all_ex)*0.9)]:.1f}")
print(f"theta:   avg={statistics.mean(all_theta):.3f}  max={max(all_theta):.3f}  p90={sorted(all_theta)[int(len(all_theta)*0.9)]:.3f}")
print(f"omega_cmd: avg={statistics.mean(all_omega_cmd):.3f}  max={max(all_omega_cmd):.3f}")
print(f"omega_ff:  avg={statistics.mean([abs(x) for x in all_omega_ff]):.3f}  max={max([abs(x) for x in all_omega_ff]):.3f}")
print(f"omega_fb:  avg={statistics.mean([abs(x) for x in all_omega_fb]):.3f}  max={max([abs(x) for x in all_omega_fb]):.3f}")
print(f"curve_sev: avg={statistics.mean(all_curve_sev):.3f}  max={max(all_curve_sev):.3f}")
print(f"kappa:     max_abs={max([abs(x) for x in all_kappa]):.3f}")

# Modes
outer_modes = {}
for l in lines:
    m = l.get('outer_mode', 'unknown')
    outer_modes[m] = outer_modes.get(m, 0) + 1
print(f"\n=== OUTER MODES ===")
for m, c in sorted(outer_modes.items(), key=lambda x: -x[1]):
    print(f"  {m}: {c} frames ({100*c//len(lines)}%)")

# Oscillation events (sign flip on omega_cmd within short window)
print(f"\n=== OSCILLATION / ZIG-ZAG EVENTS ===")
events = []
in_ev = False
ev = []
for l in lines:
    ex = abs(l.get('epsilon_x_mm', 0))
    th = abs(l.get('theta_rad', 0))
    if ex > 40 or th > 0.15:
        if not in_ev:
            in_ev = True
            ev = []
        ev.append(l)
    else:
        if in_ev:
            events.append(ev)
            in_ev = False
if in_ev:
    events.append(ev)

print(f"Found {len(events)} events")
for i, ev in enumerate(events[:6]):
    max_ex = max(abs(e.get('epsilon_x_mm', 0)) for e in ev)
    max_th = max(abs(e.get('theta_rad', 0)) for e in ev)
    max_om = max(abs(e.get('omega_cmd', 0)) for e in ev)
    min_om_lim = min(e.get('omega_limit', 99) for e in ev)
    curve_sev = max(e.get('curve_severity', 0) for e in ev)
    avg_v = statistics.mean(e.get('v_cmd', 0) for e in ev)
    mode_set = set(e.get('outer_mode') for e in ev)
    # sign flips of omega
    omegas = [e.get('omega_cmd', 0) for e in ev]
    flips = sum(1 for a, b in zip(omegas, omegas[1:]) if a*b < 0)
    print(f"\n  Event {i+1} ({len(ev)} frames, {flips} sign-flips) modes={mode_set}")
    print(f"    max_ex={max_ex:.1f}mm  max_th={max_th:.3f}  max_om={max_om:.3f}  om_lim={min_om_lim:.3f}")
    print(f"    curve_sev={curve_sev:.3f}  avg_v={avg_v:.3f}")
    print("    Timeline:")
    for j, e in enumerate(ev):
        if j % max(1, len(ev)//5) == 0 or j == len(ev)-1:
            om = e.get('omega_cmd', 0)
            ff = e.get('omega_ff', 0)
            fb = e.get('omega_fb', 0)
            cs = e.get('curve_severity', 0)
            print(f"      [{j:3d}] ex={e.get('epsilon_x_mm',0):6.1f} th={e.get('theta_rad',0):6.3f} v={e.get('v_cmd',0):.3f} om={om:.3f} ff={ff:.3f} fb={fb:.3f} cs={cs:.2f}")

# Inner loop stats during high-error
print(f"\n=== INNER WHEEL STATS (high-error frames) ===")
high_err = [l for l in lines if abs(l.get('epsilon_x_mm',0)) > 40]
if high_err:
    lwe = [abs(l.get('left_wheel_error',0)) for l in high_err]
    rwe = [abs(l.get('right_wheel_error',0)) for l in high_err]
    lpc = [abs(l.get('left_pd_correction',0)) for l in high_err]
    rpc = [abs(l.get('right_pd_correction',0)) for l in high_err]
    print(f"  wheel_error: L_avg={statistics.mean(lwe):.4f}  R_avg={statistics.mean(rwe):.4f}")
    print(f"  pd_correction: L_avg={statistics.mean(lpc):.4f}  R_avg={statistics.mean(rpc):.4f}")


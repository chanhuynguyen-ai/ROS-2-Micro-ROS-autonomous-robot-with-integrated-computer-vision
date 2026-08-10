import json

log_file = '/home/bluedstar/SimpleRobot/terminal-run/plot_logger/hybrid_unified_2026-07-30_17-35-40/raw_state.jsonl'
lines = []
for line in open(log_file):
    if line.strip():
        try:
            d = json.loads(line)
            if 'state' in d: lines.append(d['state'])
        except: pass

if not lines: exit(0)
print(f"Frames: {len(lines)}")
print(f"Version: {lines[0].get('version')}")

print("\n--- SAMPLE OF EVENTS (High Lateral Error) ---")
for l in lines:
    if abs(l.get('e_lat_raw_mm', 0)) > 50 or abs(l.get('theta_raw_rad', 0)) > 0.15:
        print(f"e_lat={l.get('e_lat_raw_mm',0):.1f} th={l.get('theta_raw_rad',0):.3f} "
              f"om_pd={l.get('omega_pd',0):.3f} om_bs={l.get('omega_bs',0):.3f} "
              f"om_fb={l.get('omega_fb',0):.3f} om_tgt={l.get('omega_target',0):.3f} "
              f"om_cmd={l.get('omega_cmd',0):.3f}")

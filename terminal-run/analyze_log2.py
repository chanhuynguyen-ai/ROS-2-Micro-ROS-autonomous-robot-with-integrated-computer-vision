import json

log_file = '/home/bluedstar/SimpleRobot/terminal-run/plot_logger/PD_controller_unified_2026-07-29_15-28-04/raw_debug.jsonl'

lines = []
with open(log_file, 'r') as f:
    for line in f:
        if line.strip():
            lines.append(json.loads(line))

print(f"Total frames: {len(lines)}")
if len(lines) == 0:
    exit(0)

max_ex = max((abs(e.get('epsilon_x_mm', 0)) for e in lines), default=0)
max_theta = max((abs(e.get('theta_rad', 0)) for e in lines), default=0)
modes = set(e.get('mode') for e in lines)

print(f"Modes encountered: {modes}")
print(f"Max abs e_x: {max_ex:.1f} mm")
print(f"Max abs theta: {max_theta:.3f} rad")
print("First 10 frames:")
for l in lines[:10]:
    print(f"  mode={l.get('mode')} v={l.get('v_cmd')} om={l.get('omega_cmd')} om_lim={l.get('omega_limit')} e_x={l.get('epsilon_x_mm')} th={l.get('theta_rad')}")

print("Last 10 frames:")
for l in lines[-10:]:
    print(f"  mode={l.get('mode')} v={l.get('v_cmd')} om={l.get('omega_cmd')} om_lim={l.get('omega_limit')} e_x={l.get('epsilon_x_mm')} th={l.get('theta_rad')}")


import json

log_file = '/home/bluedstar/SimpleRobot/terminal-run/plot_logger/cascade_unified_2026-07-29_15-37-36/raw_state.jsonl'

lines = []
with open(log_file, 'r') as f:
    for line in f:
        if line.strip():
            try:
                data = json.loads(line)
                if 'state' in data:
                    lines.append(data['state'])
            except Exception:
                pass

print(f"Total frames: {len(lines)}")
if not lines:
    exit(0)

# Find major events
in_event = False
event = []
events = []
for l in lines:
    is_high_error = abs(l.get('epsilon_x_mm', 0)) > 40 or abs(l.get('theta_rad', 0)) > 0.15
    is_planner_turn = 'turn' in l.get('outer_mode', '') or 'curve' in l.get('outer_mode', '')
    
    if is_high_error or is_planner_turn:
        if not in_event:
            in_event = True
            event = []
        event.append(l)
    else:
        if in_event:
            events.append(event)
            in_event = False
            
if in_event:
    events.append(event)
    
print(f"Found {len(events)} deviation events.")

for i, ev in enumerate(events[:5]):
    print(f"\n--- Event {i+1} (Length: {len(ev)} frames) ---")
    max_ex = max((abs(e.get('epsilon_x_mm', 0)) for e in ev), default=0)
    max_theta = max((abs(e.get('theta_rad', 0)) for e in ev), default=0)
    max_omega_cmd = max((abs(e.get('omega_cmd', 0)) for e in ev), default=0)
    min_omega_limit = min((e.get('omega_limit', 999) for e in ev), default=999)
    
    print(f"Max lateral error: {max_ex:.1f} mm")
    print(f"Max heading error: {max_theta:.3f} rad")
    print(f"Max omega cmd: {max_omega_cmd:.3f}")
    print(f"Min omega limit: {min_omega_limit:.3f}")
    
    print("Timeline (sample every 5 frames):")
    for j, e in enumerate(ev):
        if j % 5 == 0 or j == len(ev)-1:
            mode = str(e.get('outer_mode', ''))[:15]
            ex = e.get('epsilon_x_mm',0)
            th = e.get('theta_rad',0)
            v = e.get('v_cmd',0)
            om = e.get('omega_cmd',0)
            vl_cmd = e.get('v_left_cmd',0)
            vr_cmd = e.get('v_right_cmd',0)
            print(f"  [{j:3d}] {mode:15s} ex={ex:6.1f} th={th:6.3f} v={v:.3f} om={om:.3f} L={vl_cmd:.3f} R={vr_cmd:.3f}")


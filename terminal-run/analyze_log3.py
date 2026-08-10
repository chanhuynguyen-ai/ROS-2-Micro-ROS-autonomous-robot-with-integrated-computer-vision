import json

log_file = '/home/bluedstar/SimpleRobot/terminal-run/plot_logger/PD_controller_unified_2026-07-29_15-28-04/raw_debug.jsonl'

lines = []
with open(log_file, 'r') as f:
    for line in f:
        if line.strip():
            try:
                data = json.loads(line)
                if 'debug' in data:
                    lines.append(data['debug'])
                else:
                    lines.append(data)
            except Exception:
                pass

print(f"Total frames: {len(lines)}")

def analyze(lines):
    in_event = False
    event = []
    events = []
    for l in lines:
        is_high_error = abs(l.get('epsilon_x_mm', 0)) > 40 or abs(l.get('theta_rad', 0)) > 0.12
        is_curve = 'curve' in str(l.get('mode', ''))
        is_large = 'large' in str(l.get('mode', ''))
        
        if is_high_error or is_curve or is_large:
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
        
    print(f"Found {len(events)} major deviation/curve events.")
    
    for i, ev in enumerate(events[:5]):
        print(f"\n--- Event {i+1} (Length: {len(ev)} frames) ---")
        modes = set(e.get('mode') for e in ev)
        max_ex = max((abs(e.get('epsilon_x_mm', 0)) for e in ev), default=0)
        max_theta = max((abs(e.get('theta_rad', 0)) for e in ev), default=0)
        max_omega_cmd = max((abs(e.get('omega_cmd', 0)) for e in ev), default=0)
        min_omega_limit = min((e.get('omega_limit', 999) for e in ev), default=999)
        max_delta_v = max((abs(e.get('delta_v_cmd', 0)) for e in ev), default=0)
        
        print(f"Modes: {modes}")
        print(f"Max lateral error: {max_ex:.1f} mm")
        print(f"Max heading error: {max_theta:.3f} rad")
        print(f"Max omega cmd: {max_omega_cmd:.3f}")
        print(f"Min omega limit: {min_omega_limit:.3f}")
        print(f"Max delta_V cmd: {max_delta_v:.3f} m/s")
        
        if ev:
            print(f"Entry v_cmd: {ev[0].get('v_cmd', 0):.3f}")
            print(f"Exit v_cmd: {ev[-1].get('v_cmd', 0):.3f}")
        
        print("Timeline (sample every 5 frames):")
        for j, e in enumerate(ev):
            if j % 5 == 0 or j == len(ev)-1:
                mode_str = str(e.get('mode'))[:15]
                print(f"  [{j:3d}] mode={mode_str:15s} e_x={e.get('epsilon_x_mm',0):6.1f} th={e.get('theta_rad',0):6.3f} v={e.get('v_cmd',0):.3f} om={e.get('omega_cmd',0):.3f} om_lim={e.get('omega_limit',0):.3f} dV={e.get('delta_v_cmd',0):.3f}")

analyze(lines)

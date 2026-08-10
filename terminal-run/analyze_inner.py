import json

log_file = '/home/bluedstar/SimpleRobot/terminal-run/plot_logger/cascade_unified_2026-07-30_14-56-58/raw_state.jsonl'
for line in open(log_file):
    if line.strip():
        try:
            d = json.loads(line)['state']
            if abs(d.get('epsilon_x_mm',0)) > 150:
                print(f"ex={d['epsilon_x_mm']:.0f} th={d['theta_rad']:.3f} v={d['v_cmd']:.3f} om={d['omega_cmd']:.3f}")
                print(f"  vL_ref={d.get('v_left_ref',0):.3f} vR_ref={d.get('v_right_ref',0):.3f}")
                print(f"  vL_mea={d.get('v_left_measured',0):.3f} vR_mea={d.get('v_right_measured',0):.3f}")
                print(f"  vL_cmd={d.get('v_left_cmd',0):.3f} vR_cmd={d.get('v_right_cmd',0):.3f}")
                print(f"  L_err={d.get('left_wheel_error',0):.3f} L_pd={d.get('left_pd_correction',0):.3f}")
        except: pass

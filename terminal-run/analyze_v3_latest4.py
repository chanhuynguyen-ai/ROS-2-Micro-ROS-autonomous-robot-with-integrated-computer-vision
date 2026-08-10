import json

log_file = '/home/bluedstar/SimpleRobot/terminal-run/plot_logger/cascade_unified_2026-07-30_14-56-58/raw_state.jsonl'
lines = []
for line in open(log_file):
    if line.strip():
        try:
            d = json.loads(line)
            if 'state' in d: lines.append(d['state'])
        except: pass

events = []
in_ev = False; ev = []
for l in lines:
    if abs(l.get('e_used_mm',0)) > 60 or abs(l.get('theta_used_rad',0)) > 0.25:
        if not in_ev: in_ev=True; ev=[]
        ev.append(l)
    else:
        if in_ev: events.append(ev); in_ev=False
if in_ev: events.append(ev)

for i, ev in enumerate(events[:5]):
    print(f"\nEv{i+1}:")
    for j, e in enumerate(ev):
        if j % max(1,len(ev)//4)==0 or j==len(ev)-1:
            vd = e.get('v_des', e.get('v_target',0))
            print(f"  [{j:3d}] raw_ex={e.get('epsilon_x_mm',0):6.1f} used_ex={e.get('e_used_mm',0):6.1f} used_th={e.get('theta_used_rad',0):6.3f} v={e.get('v_cmd',0):.3f} om={e.get('omega_cmd',0):.3f} cs={e.get('curve_severity',0):.2f}")

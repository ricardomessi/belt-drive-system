import math
import itertools

pulleys = {
    'CRK': {'x': 0.00, 'y': 0.00, 'pitch_dia': 160.14, 'eff_dia': 158.14, 'cw': True},
    'FAN': {'x': 6.00, 'y': 213.50, 'pitch_dia': 122.99, 'eff_dia': 120.99, 'cw': True},
    'IDR': {'x': -122.00, 'y': 235.00, 'pitch_dia': 77.40, 'eff_dia': 79.40, 'cw': False},
    'ALT': {'x': -255.00, 'y': 373.20, 'pitch_dia': 62.14, 'eff_dia': 60.14, 'cw': True},
    'AC':  {'x': -265.00, 'y': 189.00, 'pitch_dia': 121.31, 'eff_dia': 119.31, 'cw': True},
    'TEN': {'x': -153.25, 'y': 95.96, 'pitch_dia': 77.40, 'eff_dia': 79.40, 'cw': False}
}

routing = ['CRK', 'FAN', 'IDR', 'ALT', 'AC', 'TEN']

def get_all_tangents(p1, p2):
    r1 = p1['eff_dia'] / 2.0
    r2 = p2['eff_dia'] / 2.0
    
    dx = p2['x'] - p1['x']
    dy = p2['y'] - p1['y']
    d = math.hypot(dx, dy)
    gamma = math.atan2(dy, dx)
    
    tangents = []
    
    # Outer tangents
    val_out = (r1 - r2) / d
    if abs(val_out) <= 1.0:
        beta_out = math.acos(val_out)
        for sign in [-1, 1]:
            phi_n = gamma + sign * beta_out
            tangents.append({'type': 'outer', 'phi': phi_n + math.pi/2, 't1': phi_n, 't2': phi_n})
            tangents.append({'type': 'outer', 'phi': phi_n - math.pi/2, 't1': phi_n + math.pi, 't2': phi_n + math.pi})
            
    # Inner tangents
    val_in = (r1 + r2) / d
    if abs(val_in) <= 1.0:
        beta_in = math.acos(val_in)
        for sign in [-1, 1]:
            phi_n = gamma + sign * beta_in
            tangents.append({'type': 'inner', 'phi': phi_n + math.pi/2, 't1': phi_n, 't2': phi_n + math.pi})
            tangents.append({'type': 'inner', 'phi': phi_n - math.pi/2, 't1': phi_n + math.pi, 't2': phi_n})
            
    for t in tangents:
        t['phi'] = (t['phi'] + math.pi) % (2 * math.pi) - math.pi
        t['t1'] = (t['t1'] + math.pi) % (2 * math.pi) - math.pi
        t['t2'] = (t['t2'] + math.pi) % (2 * math.pi) - math.pi
        t['p1_t'] = (p1['x'] + r1 * math.cos(t['t1']), p1['y'] + r1 * math.sin(t['t1']))
        t['p2_t'] = (p2['x'] + r2 * math.cos(t['t2']), p2['y'] + r2 * math.sin(t['t2']))
        t['len'] = math.hypot(t['p2_t'][0] - t['p1_t'][0], t['p2_t'][1] - t['p1_t'][1])
        
    return tangents

# Target span lengths
pdf_spans = {
    'CRK': 212.8, 'FAN': 82.5, 'IDR': 178.7, 'ALT': 182.1, 'AC': 106.2, 'TEN': 136.3
}

# Generate matched tangents for each span
matches_per_span = {}
for i in range(6):
    n1 = routing[i]
    n2 = routing[(i + 1) % 6]
    tangents = get_all_tangents(pulleys[n1], pulleys[n2])
    target_len = pdf_spans[n1]
    matches = [t for t in tangents if abs(t['len'] - target_len) < 1.0]
    matches_per_span[n1] = matches

options = [matches_per_span[k] for k in routing]

print("Searching for the combination that gives a total belt length close to 1577.3 mm...")
best_comb = None
min_len_diff = 9999.0
best_total_len = 0.0
best_wraps = {}

for combination in itertools.product(*options):
    spans = {routing[j]: combination[j] for j in range(6)}
    
    # Calculate wrap angles and total belt length
    total_len = 0.0
    wraps = {}
    
    for j in range(6):
        name = routing[j]
        p = pulleys[name]
        r = p['eff_dia'] / 2.0
        
        span_in = spans[routing[(j - 1) % 6]]
        span_out = spans[name]
        
        theta_in = span_in['t2']
        theta_out = span_out['t1']
        
        if p['cw']:
            wrap = (theta_in - theta_out) % (2 * math.pi)
        else:
            wrap = (theta_out - theta_in) % (2 * math.pi)
            
        wrap_deg = math.degrees(wrap)
        wraps[name] = wrap_deg
        
        total_len += span_out['len'] + r * wrap
        
    diff = abs(total_len - 1577.3)
    if diff < min_len_diff:
        min_len_diff = diff
        best_comb = spans
        best_total_len = total_len
        best_wraps = wraps

print(f"\nFOUND COMBINATION! Total Length = {best_total_len:.2f} mm (error = {min_len_diff:.2f} mm)")
print("Wrap Angles:")
for name in routing:
    print(f"  {name}: {best_wraps[name]:.2f}° (target: {name} wrap in PDF)")
print("Span Lengths:")
for name in routing:
    print(f"  {name} -> {routing[(routing.index(name) + 1) % 6]}: {best_comb[name]['len']:.2f} mm")

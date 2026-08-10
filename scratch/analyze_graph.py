def parse_param(param_path):
    layers = []
    with open(param_path, "r") as f:
        lines = f.readlines()
    
    magic = lines[0].strip()
    counts = lines[1].strip().split()
    
    for line in lines[2:]:
        parts = line.strip().split()
        if not parts:
            continue
        layer_type = parts[0]
        layer_name = parts[1]
        input_count = int(parts[2])
        output_count = int(parts[3])
        
        inputs = parts[4:4+input_count]
        outputs = parts[4+input_count:4+input_count+output_count]
        
        layers.append({
            "type": layer_type,
            "name": layer_name,
            "inputs": inputs,
            "outputs": outputs
        })
    return layers

def trace_back_bounded(layers, target_output, max_depth, depth=0, visited=None):
    if visited is None:
        visited = set()
    if depth > max_depth:
        return []
    
    head_layers = []
    
    for layer in layers:
        if target_output in layer["outputs"]:
            if layer["name"] not in visited:
                visited.add(layer["name"])
                head_layers.append((layer, depth))
                for inp in layer["inputs"]:
                    head_layers.extend(trace_back_bounded(layers, inp, max_depth, depth + 1, visited))
            break
            
    return head_layers

def main():
    param_path = "/home/goln/SimpleSysIDV/models/best_ncnn_model/model-opt.param"
    layers = parse_param(param_path)
    
    # Trace bounded depth of 6
    out0_layers = trace_back_bounded(layers, "out0", max_depth=6)
    out1_layers = trace_back_bounded(layers, "out1", max_depth=6)
    
    print("\n=== Bounded Trace for out0 (depth <= 6) ===")
    for l, d in out0_layers:
        print(f"Depth {d}: Type={l['type']}, Name={l['name']}, Inputs={l['inputs']}, Outputs={l['outputs']}")
        
    print("\n=== Bounded Trace for out1 (depth <= 6) ===")
    for l, d in out1_layers:
        print(f"Depth {d}: Type={l['type']}, Name={l['name']}, Inputs={l['inputs']}, Outputs={l['outputs']}")

if __name__ == "__main__":
    main()

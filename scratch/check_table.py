def main():
    table_path = "/home/goln/SimpleSysIDV/models/best_ncnn_model/model.table"
    with open(table_path, "r") as f:
        lines = f.readlines()
        
    print(f"Table contains {len(lines)} lines.")
    
    nan_count = 0
    zero_count = 0
    inf_count = 0
    total_values = 0
    
    for i, line in enumerate(lines):
        # Ignore comments
        if line.startswith("#"):
            continue
            
        parts = line.strip().split()
        if not parts:
            continue
            
        layer_name = parts[0]
        scales = parts[1:]
        
        for val_str in scales:
            total_values += 1
            val = float(val_str)
            if val != val:  # NaN check
                nan_count += 1
                print(f"NaN found in layer: {layer_name} (line {i})")
            elif val == 0.0:
                zero_count += 1
            elif val == float('inf') or val == float('-inf'):
                inf_count += 1
                print(f"Inf found in layer: {layer_name} (line {i})")
                
    print(f"Summary of {total_values} scale values:")
    print(f"NaNs: {nan_count}")
    print(f"Infs: {inf_count}")
    print(f"Zeros: {zero_count}")

if __name__ == "__main__":
    main()

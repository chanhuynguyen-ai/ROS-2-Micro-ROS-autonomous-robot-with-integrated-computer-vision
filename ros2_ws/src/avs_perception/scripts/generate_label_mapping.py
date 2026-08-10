import sys
import json
import os

def main():
    if len(sys.argv) != 3:
        print("Usage: generate_label_mapping.py <input_json> <output_hpp>")
        sys.exit(1)

    input_json = sys.argv[1]
    output_hpp = sys.argv[2]

    try:
        with open(input_json, 'r') as f:
            mapping = json.load(f)
    except Exception as e:
        print(f"Error reading {input_json}: {e}")
        sys.exit(1)

    # Prepare labels
    label_constants = []
    class_names_array = []
    max_label = -1
    
    # Sort keys as integers
    sorted_keys = sorted([int(k) for k in mapping.keys()])
    
    for k in sorted_keys:
        class_name = mapping[str(k)]
        var_name = class_name.replace("-", "_").upper()
        label_constants.append(f"constexpr int LABEL_{var_name} = {k};")
        max_label = max(max_label, k)

    for i in range(max_label + 1):
        if str(i) in mapping:
            class_names_array.append(f'        "{mapping[str(i)]}"')
        else:
            class_names_array.append(f'        "unknown_{i}"')

    hpp_content = """#ifndef AVS_PERCEPTION_LABEL_MAPPING_HPP
#define AVS_PERCEPTION_LABEL_MAPPING_HPP

#include <string>
#include <vector>

namespace avs_perception {

"""
    hpp_content += "\n".join(label_constants) + "\n\n"
    hpp_content += "inline const std::vector<std::string> CLASS_NAMES = {\n"
    hpp_content += ",\n".join(class_names_array)
    hpp_content += "\n};\n\n"
    hpp_content += "} // namespace avs_perception\n\n#endif // AVS_PERCEPTION_LABEL_MAPPING_HPP\n"

    # Make directory if not exists
    os.makedirs(os.path.dirname(output_hpp), exist_ok=True)

    with open(output_hpp, 'w') as f:
        f.write(hpp_content)

if __name__ == "__main__":
    main()

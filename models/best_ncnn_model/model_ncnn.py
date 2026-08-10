import os

import numpy as np
import ncnn
import torch

# The exporter writes absolute paths here; resolve next to this file instead so
# the snippet also runs from another checkout or inside the container.
MODEL_DIR = os.path.dirname(os.path.abspath(__file__))

def test_inference():
    torch.manual_seed(0)
    in0 = torch.rand(1, 3, 320, 320, dtype=torch.float)
    out = []

    with ncnn.Net() as net:
        net.load_param(os.path.join(MODEL_DIR, "model.ncnn.param"))
        net.load_model(os.path.join(MODEL_DIR, "model.ncnn.bin"))

        with net.create_extractor() as ex:
            ex.input("in0", ncnn.Mat(in0.squeeze(0).numpy()).clone())

            _, out0 = ex.extract("out0")
            out.append(torch.from_numpy(np.array(out0)).unsqueeze(0))
            _, out1 = ex.extract("out1")
            out.append(torch.from_numpy(np.array(out1)).unsqueeze(0))

    if len(out) == 1:
        return out[0]
    else:
        return tuple(out)

if __name__ == "__main__":
    print(test_inference())

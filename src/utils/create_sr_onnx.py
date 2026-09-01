#!/usr/bin/env python3
"""
Create a lightweight ESPCN-like SR x4 model in ONNX format.
Uses only ONNX primitives - no PyTorch or TensorFlow needed.
Architecture: Conv(5x5,64) -> ReLU -> Conv(3x3,32) -> ReLU -> Conv(3x3,48) -> PixelShuffle(4)
Input:  [1, 3, H, W]  (float32, 0-1)
Output: [1, 3, H*4, W*4] (float32, 0-1)
"""
import numpy as np
import onnx
from onnx import helper, TensorProto, numpy_helper
import argparse
import os

def make_initializer(name, shape):
    """Xavier-uniform initialization"""
    fan_in = shape[1] * shape[2] * shape[3] if len(shape) == 4 else shape[0]
    fan_out = shape[0] * shape[2] * shape[3] if len(shape) == 4 else shape[1]
    limit = np.sqrt(6.0 / (fan_in + fan_out))
    data = np.random.uniform(-limit, limit, shape).astype(np.float32)
    return numpy_helper.from_array(data, name=name)

def make_bias(name, shape):
    data = np.zeros(shape, dtype=np.float32)
    return numpy_helper.from_array(data, name=name)

def create_espcn_onnx(output_path, scale=4):
    # Weights
    inits = []
    # Conv1: 3 -> 64, 5x5
    inits.append(make_initializer("conv1_w", [64, 3, 5, 5]))
    inits.append(make_bias("conv1_b", [64]))
    # Conv2: 64 -> 32, 3x3
    inits.append(make_initializer("conv2_w", [32, 64, 3, 3]))
    inits.append(make_bias("conv2_b", [32]))
    # Conv3: 32 -> 3*scale*scale=48, 3x3
    out_ch = 3 * scale * scale
    inits.append(make_initializer("conv3_w", [out_ch, 32, 3, 3]))
    inits.append(make_bias("conv3_b", [out_ch]))

    # Graph nodes
    nodes = []
    # Conv1 + ReLU
    nodes.append(helper.make_node("Conv", ["input", "conv1_w", "conv1_b"], ["conv1_out"],
                                  kernel_shape=[5,5], pads=[2,2,2,2]))
    nodes.append(helper.make_node("Relu", ["conv1_out"], ["relu1_out"]))
    # Conv2 + ReLU
    nodes.append(helper.make_node("Conv", ["relu1_out", "conv2_w", "conv2_b"], ["conv2_out"],
                                  kernel_shape=[3,3], pads=[1,1,1,1]))
    nodes.append(helper.make_node("Relu", ["conv2_out"], ["relu2_out"]))
    # Conv3
    nodes.append(helper.make_node("Conv", ["relu2_out", "conv3_w", "conv3_b"], ["conv3_out"],
                                  kernel_shape=[3,3], pads=[1,1,1,1]))
    # PixelShuffle via Depth2Space
    nodes.append(helper.make_node("DepthToSpace", ["conv3_out"], ["output"],
                                  blocksize=scale, mode="CRD"))

    # I/O
    inp = helper.make_tensor_value_info("input", TensorProto.FLOAT, [1, 3, "H", "W"])
    out = helper.make_tensor_value_info("output", TensorProto.FLOAT, [1, 3, "H4", "W4"])

    graph = helper.make_graph(nodes, "ESPCN_x4", [inp], [out], initializer=inits)
    model = helper.make_model(graph, opset_imports=[helper.make_opsetid("", 13)])
    model.ir_version = 8

    onnx.checker.check_model(model)
    os.makedirs(os.path.dirname(output_path) or ".", exist_ok=True)
    onnx.save(model, output_path)
    print(f"Saved ONNX SR model to {output_path}")
    # Print model size
    size_kb = os.path.getsize(output_path) / 1024
    print(f"Model size: {size_kb:.1f} KB")

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", default="model/SR/espcn_x4.onnx")
    parser.add_argument("--scale", type=int, default=4)
    args = parser.parse_args()
    create_espcn_onnx(args.output, args.scale)

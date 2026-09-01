#!/usr/bin/env python3
"""Convert an OpenCV dnn_superres .pb (FSRCNN/ESPCN) to ONNX with dynamic HxW.
Usage:
    python pb2onnx_sr.py --pb model/SR/FSRCNN_x4.pb --onnx model/SR/fsrcnn_x4.onnx
"""
import argparse, os
import tensorflow.compat.v1 as tf
import tf2onnx
tf.disable_v2_behavior()


def convert(pb_path: str, onnx_path: str):
    with tf.io.gfile.GFile(pb_path, "rb") as f:
        graph_def = tf.GraphDef()
        graph_def.ParseFromString(f.read())

    with tf.Graph().as_default() as graph:
        # Typical SR .pb graphs from OpenCV Zoo use NHWC input named "IteratorGetNext:0"
        # or "input:0". Try common names.
        tf.import_graph_def(graph_def, name="")
        input_name = None
        for candidate in ("IteratorGetNext:0", "input:0", "x:0", "Placeholder:0"):
            try:
                graph.get_tensor_by_name(candidate)
                input_name = candidate
                break
            except KeyError:
                continue
        if input_name is None:
            # Fallback: pick first placeholder op
            for op in graph.get_operations():
                if op.type == "Placeholder":
                    input_name = op.outputs[0].name
                    break
        if input_name is None:
            raise RuntimeError("Could not find graph input tensor")

        # Find output: last node of float type that is a conv/deconv/pixelshuffle.
        output_name = None
        for candidate in ("NCHW_output:0", "output:0", "conv2d_3/BiasAdd:0",
                          "NHWC_output:0"):
            try:
                graph.get_tensor_by_name(candidate)
                output_name = candidate
                break
            except KeyError:
                continue
        if output_name is None:
            # last operation output
            ops = [op for op in graph.get_operations()
                   if op.type not in ("NoOp", "Const", "Identity")]
            output_name = ops[-1].outputs[0].name
        print(f"Input:  {input_name}")
        print(f"Output: {output_name}")

        # OpenCV SR .pb typically uses NHWC float32 input.
        # Use a placeholder fixed size; we will rewrite dims to dynamic after.
        model_proto, _ = tf2onnx.convert.from_graph_def(
            graph_def,
            input_names=[input_name],
            output_names=[output_name],
            opset=13,
            shape_override={input_name: [1, 64, 64, 1]},
        )
    # Rewrite H,W to dynamic symbolic dims in the ONNX graph.
    # Input  is NHWC [1, H,   W,   1]  -> positions 1,2 dynamic
    # Output is NCHW [1, 1, 4H, 4W]    -> positions 2,3 dynamic
    def mark_dyn(tensor, idxs, names):
        d = tensor.type.tensor_type.shape.dim
        for i, n in zip(idxs, names):
            d[i].ClearField("dim_value"); d[i].dim_param = n
    for t in model_proto.graph.input:
        mark_dyn(t, [1, 2], ["H", "W"])
    for t in model_proto.graph.output:
        mark_dyn(t, [2, 3], ["H4", "W4"])
    os.makedirs(os.path.dirname(onnx_path) or ".", exist_ok=True)
    with open(onnx_path, "wb") as f:
        f.write(model_proto.SerializeToString())
    print(f"Saved ONNX: {onnx_path}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--pb", required=True)
    ap.add_argument("--onnx", required=True)
    args = ap.parse_args()
    convert(args.pb, args.onnx)

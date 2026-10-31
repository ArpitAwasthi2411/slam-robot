#!/usr/bin/env python3
"""Shape checks for the phone detector models (used by fetch_phone_detector.sh).

    python3 scripts/check_phone_model.py onnx   web/app/vendor/yolo11n_320.onnx
    python3 scripts/check_phone_model.py tflite web/app/vendor/yolo11n_320.tflite

The app's NativeDetector.java expects the TFLite model as input [1, 320, 320, 3] (NHWC, RGB 0..1) and output
[1, 84, 2100] or [1, 2100, 84] (cx, cy, w, h, 80 class scores; boxes in 0..1 or in pixels, both handled).
"""
import os
import sys


def check_onnx(path):
    import onnx
    m = onnx.load(path)
    ins = [[d.dim_value for d in i.type.tensor_type.shape.dim] for i in m.graph.input]
    outs = [[d.dim_value for d in o.type.tensor_type.shape.dim] for o in m.graph.output]
    assert ins == [[1, 3, 320, 320]] and outs == [[1, 84, 2100]], (ins, outs)
    print('onnx check: input', ins[0], 'output', outs[0])


def check_tflite(path):
    assert os.path.getsize(path) > 1_000_000, 'TFLite model missing or too small'
    try:
        from ai_edge_litert.interpreter import Interpreter
    except ImportError:
        from tensorflow.lite import Interpreter
    it = Interpreter(model_path=path)
    it.allocate_tensors()
    i = it.get_input_details()[0]['shape'].tolist()
    o = it.get_output_details()[0]['shape'].tolist()
    assert i == [1, 320, 320, 3] and o in ([1, 84, 2100], [1, 2100, 84]), (i, o)
    print('tflite check: input', i, 'output', o)


if __name__ == '__main__':
    {'onnx': check_onnx, 'tflite': check_tflite}[sys.argv[1]](sys.argv[2])

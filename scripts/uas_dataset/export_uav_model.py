#!/usr/bin/env python3
"""
Export the fine-tuned UAS detector to ONNX and to TensorRT (plan 3.2).

The weights of the run are exported to ONNX, and the ONNX model is exported to
a TensorRT engine when the environment has the TensorRT of Ultralytics. The
script prints what it managed to do, so a run without TensorRT is not a
failure: the ONNX model is what the detection node can always load.

Usage (inside the container, with the workspace sourced):

    python3 scripts/uas_dataset/export_uav_model.py --weights runs/uav_yolov8n/weights/best.pt
"""

import argparse
import os
import shutil
import sys


def main():
    """Export the model."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--weights', required=True,
                        help='the weights of the trained detector')
    parser.add_argument('--out', default=None,
                        help='the directory the exports are written to')
    parser.add_argument('--imgsz', type=int, default=1280,
                        help='the size of the input of the model')
    parser.add_argument('--half', action='store_true',
                        help='export the model with half precision')
    parser.add_argument('--no-tensorrt', action='store_true',
                        help='do not try to build a TensorRT engine')
    args = parser.parse_args()

    from ultralytics import YOLO

    model = YOLO(args.weights)
    directory = args.out or os.path.dirname(os.path.abspath(args.weights))
    os.makedirs(directory, exist_ok=True)

    onnx_path = model.export(
        format='onnx', imgsz=args.imgsz, half=args.half, simplify=False)
    print('The ONNX model is at ' + str(onnx_path), flush=True)

    exported = {'onnx': str(onnx_path)}

    if not args.no_tensorrt:
        try:
            engine = model.export(
                format='engine', imgsz=args.imgsz, half=args.half, device=0)
            print('The TensorRT engine is at ' + str(engine), flush=True)
            exported['tensorrt'] = str(engine)
        except Exception as error:
            print('The TensorRT export failed: ' + str(error), flush=True)

    # The detection node takes the path of the model as a parameter, so the
    # models are copied next to each other with the name the configuration
    # uses.
    for name in ('uav_yolov8n.onnx', 'uav_yolov8n.engine'):
        source = exported.get('engine' if name.endswith('.engine') else 'onnx')
        if source and os.path.exists(source):
            shutil.copy(source, os.path.join(directory, name))
            print('Copied to ' + os.path.join(directory, name), flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())

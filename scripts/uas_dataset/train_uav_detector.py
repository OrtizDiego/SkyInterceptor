#!/usr/bin/env python3
"""
Fine-tune a YOLOv8n detector with a single class, uav, on the auto-labelled
dataset of the simulator (plan 3.2).

The script trains the detector, evaluates it on the held-out split of the
dataset and writes the numbers it measured into the run directory. The
evaluation is done at the confidence threshold the detection node uses
(0.3 by default, the value of config/perception_params.yaml), so the numbers
say what the node will see in the simulator.

Usage (inside the container, with the workspace sourced):

    python3 scripts/uas_dataset/train_uav_detector.py --data /tmp/uas_dataset

The GPU of the image is built for a newer CUDA driver than the host has, so
the training falls back to the CPU unless the compatible build is put in front
of it:

    PYTHONPATH=/opt/torch124 python3 scripts/uas_dataset/train_uav_detector.py ...
"""

import argparse
import json
import os
import sys
import types


def stub_matplotlib():
    """
    Make the training work without a usable matplotlib.

    The matplotlib of the image is compiled against the NumPy 1 C API and
    cannot be imported with the NumPy 2 of the image. Ultralytics only needs
    it to look up the font of the plots, so a stub with an empty font list is
    enough: the font is downloaded instead.
    """
    try:
        import matplotlib  # noqa: F401
        return False
    except Exception:
        pass
    matplotlib = types.ModuleType('matplotlib')
    font_manager = types.ModuleType('matplotlib.font_manager')
    font_manager.fontManager = types.SimpleNamespace(ttflist=[])
    matplotlib.font_manager = font_manager
    sys.modules['matplotlib'] = matplotlib
    sys.modules['matplotlib.font_manager'] = font_manager
    return True


STUBBED_MATPLOTLIB = stub_matplotlib()

import torch  # noqa: E402
from ultralytics import YOLO  # noqa: E402


def iou(box_a, box_b):
    """The intersection over union of two boxes in (x1, y1, x2, y2)."""
    left = max(box_a[0], box_b[0])
    top = max(box_a[1], box_b[1])
    right = min(box_a[2], box_b[2])
    bottom = min(box_a[3], box_b[3])
    if right <= left or bottom <= top:
        return 0.0
    intersection = (right - left) * (bottom - top)
    area_a = (box_a[2] - box_a[0]) * (box_a[3] - box_a[1])
    area_b = (box_b[2] - box_b[0]) * (box_b[3] - box_b[1])
    union = area_a + area_b - intersection
    return intersection / union if union > 0.0 else 0.0


def read_labels(path, width, height):
    """The boxes of a label file, in pixels."""
    boxes = []
    if not os.path.exists(path):
        return boxes
    with open(path, 'r', encoding='utf-8') as handle:
        for line in handle:
            parts = line.split()
            if len(parts) != 5:
                continue
            centre_x = float(parts[1]) * width
            centre_y = float(parts[2]) * height
            box_width = float(parts[3]) * width
            box_height = float(parts[4]) * height
            boxes.append((
                centre_x - box_width / 2.0, centre_y - box_height / 2.0,
                centre_x + box_width / 2.0, centre_y + box_height / 2.0))
    return boxes


def evaluate(model, data, split, threshold, imgsz, device):
    """
    Count the true positives, the false positives and the false negatives of
    the detector on one split of the dataset, at a confidence threshold.

    A detection is a true positive when it matches a label with an
    intersection over union of at least 0.5.
    """
    root = os.path.dirname(os.path.abspath(data)) if os.path.isfile(data) else data
    with open(data, 'r', encoding='utf-8') as handle:
        content = handle.read()
    image_dir = os.path.join(root, 'images', split)
    label_dir = os.path.join(root, 'labels', split)
    if not os.path.isdir(image_dir):
        print('No ' + split + ' images in ' + image_dir, file=sys.stderr)
        return None

    true_positive = 0
    false_positive = 0
    false_negative = 0
    positives = 0
    for name in sorted(os.listdir(image_dir)):
        if not name.endswith('.png'):
            continue
        image = os.path.join(image_dir, name)
        label = os.path.join(
            label_dir, name.replace('.png', '.txt'))
        from PIL import Image as PilImage
        with PilImage.open(image) as handle:
            width, height = handle.size
        truth = read_labels(label, width, height)
        positives += len(truth)
        results = model.predict(
            image, conf=threshold, imgsz=imgsz, device=device, verbose=False)
        detected = []
        for result in results:
            for box in result.boxes:
                corners = box.xyxy[0].tolist()
                detected.append((corners[0], corners[1], corners[2], corners[3]))
        used = set()
        for truth_box in truth:
            best = -1.0
            best_index = -1
            for index, detected_box in enumerate(detected):
                if index in used:
                    continue
                overlap = iou(truth_box, detected_box)
                if overlap > best:
                    best = overlap
                    best_index = index
            if best >= 0.5:
                used.add(best_index)
                true_positive += 1
            else:
                false_negative += 1
        false_positive += len(detected) - len(used)
    return {
        'threshold': threshold,
        'images': len([n for n in os.listdir(image_dir) if n.endswith('.png')]),
        'labels': positives,
        'true_positive': true_positive,
        'false_positive': false_positive,
        'false_negative': false_negative,
        'precision': (true_positive / (true_positive + false_positive)
                      if true_positive + false_positive > 0 else 0.0),
        'recall': (true_positive / positives if positives > 0 else 0.0),
    }


def main():
    """Train the detector and measure it."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--data', required=True,
                        help='the dataset directory (with data.yaml in it)')
    parser.add_argument('--weights', default='yolov8n.pt',
                        help='the checkpoint the fine-tune starts from')
    parser.add_argument('--epochs', type=int, default=40)
    parser.add_argument('--imgsz', type=int, default=1280)
    parser.add_argument('--batch', type=int, default=8)
    parser.add_argument('--device', default='0',
                        help='the device: a number for a GPU, "cpu" for the CPU')
    parser.add_argument('--seed', type=int, default=0)
    parser.add_argument('--threshold', type=float, default=0.3,
                        help='the confidence threshold the node uses')
    parser.add_argument('--out', default=None,
                        help='the directory the run is written to')
    parser.add_argument('--name', default='uav_yolov8n')
    parser.add_argument('--workers', type=int, default=0,
                        help='the workers of the data loader. The default is '
                             'zero: the shared memory of the container is '
                             'too small for the workers of PyTorch')
    parser.add_argument('--plots', action='store_true',
                        help='write the plots of the run (they need a '
                             'matplotlib that works with the NumPy of the '
                             'image)')
    args = parser.parse_args()

    if not torch.cuda.is_available() and args.device != 'cpu':
        print('No GPU is visible to this PyTorch build '
              f'({torch.__version__}, built for CUDA {torch.version.cuda}); '
              'the training runs on the CPU. Put the compatible build in '
              'front with PYTHONPATH=/opt/torch124.', file=sys.stderr)
        args.device = 'cpu'
    device = args.device
    if device != 'cpu':
        print('The training runs on the GPU ' +
              torch.cuda.get_device_name(0), flush=True)
    else:
        print('The training runs on the CPU', flush=True)

    model = YOLO(args.weights)
    results = model.train(
        data=os.path.join(args.data, 'data.yaml'),
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        device=device,
        seed=args.seed,
        project=args.out,
        name=args.name,
        exist_ok=True,
        val=True,
        workers=args.workers,
        plots=args.plots and not STUBBED_MATPLOTLIB,
        verbose=True,
    )

    run = args.out or 'runs/detect'
    directory = os.path.join(run, args.name)
    best = os.path.join(directory, 'weights', 'best.pt')
    if not os.path.exists(best):
        best = os.path.join(directory, 'weights', 'last.pt')
    trained = YOLO(best)

    metrics = trained.val(
        data=os.path.join(args.data, 'data.yaml'),
        imgsz=args.imgsz,
        device=device,
        workers=args.workers,
        verbose=False,
    )
    summary = {
        'weights': best,
        'imgsz': args.imgsz,
        'epochs': args.epochs,
        'batch': args.batch,
        'device': device,
        'map50': float(metrics.box.map50),
        'map50_95': float(metrics.box.map),
        'precision': float(metrics.box.mp),
        'recall': float(metrics.box.mr),
    }
    counted = evaluate(
        trained, os.path.join(args.data, 'data.yaml'), 'val',
        args.threshold, args.imgsz, device)
    if counted is not None:
        summary['at_threshold'] = counted
    with open(os.path.join(directory, 'summary.json'), 'w',
              encoding='utf-8') as handle:
        json.dump(summary, handle, indent=2)
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == '__main__':
    sys.exit(main())

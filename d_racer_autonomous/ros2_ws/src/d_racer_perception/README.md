# d_racer_perception

Experimental perception package for D-Racer.

This package currently contains only a removable test node:

- `yolo_detect_test_node`

It is for checking model runtime, camera-topic wiring, and debug output on
the D3G board. It does not publish final driving commands.

> Lane detection is handled separately with an OpenCV-based pipeline; YOLO here
> is used only for object detection.

## Download Models On D3G

```bash
cd ~/SEA-ME_Hackathon_SMT/d_racer_autonomous/ros2_ws/src/d_racer_perception
bash scripts/download_test_models.sh
```

By default, the script downloads the Ultralytics `yolo26n` pretrained detection
model, saves it as `models/yolo_detect_test.pt`, and exports an NCNN model
(`imgsz=320`) for faster on-board inference:

- `models/yolo_detect_test.pt` (torch)
- `models/yolo_detect_test_ncnn_model/` (NCNN — used by the node)

To use a team-selected URL instead:

```bash
DETECT_MODEL_URL="https://example.com/detect.pt" \
bash scripts/download_test_models.sh
```

## On-Board Performance (D3-G, TCC8050 / Cortex-A72 x4)

Measured with `scripts/bench_yolo.py` on the D3-G board:

| Backend | imgsz | Latency (p50) | FPS | CPU |
|---------|-------|---------------|-----|-----|
| torch CPU | 640 | 419 ms | 2.4 | ~1 core |
| torch CPU | 320 | 151 ms | 6.6 | ~1 core |
| **NCNN** | 320 | **64 ms** | **15.5** | ~1 core |
| NCNN | 416 | 99 ms | 10.0 | ~1 core |

NCNN uses ~1 core, leaving ~3 cores for OpenCV lane detection, planning, and
control. The node defaults to the NCNN model at `imgsz=320`.

## Build

```bash
cd ~/SEA-ME_Hackathon_SMT/d_racer_autonomous/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select d_racer_perception
source install/setup.bash
```

## Run Test Node

```bash
ros2 launch d_racer_perception yolo_detect_test.launch.py
```

Debug image topic:

- `perception/test/yolo_detect/debug/compressed`

Stats topic:

- `perception/test/yolo_detect/stats`

## Clean Removal After Testing

If the team decides not to keep this test node, remove this whole package:

```bash
rm -rf d_racer_autonomous/ros2_ws/src/d_racer_perception
```

Then rebuild the workspace.

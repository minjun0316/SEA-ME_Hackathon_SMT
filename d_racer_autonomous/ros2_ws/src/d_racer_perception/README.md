# d_racer_perception

Experimental perception package for D-Racer.

This package currently contains only removable test nodes:

- `yolo_detect_test_node`
- `yolo_seg_test_node`

They are for checking model runtime, camera-topic wiring, and debug output on
the D3G board. They do not publish final driving commands.

## Download Models On D3G

```bash
cd ~/SEA-ME_Hackathon_SMT/d_racer_autonomous/ros2_ws/src/d_racer_perception
bash scripts/download_test_models.sh
```

By default, the script downloads small Ultralytics pretrained test models and
saves them as:

- `models/yolo_detect_test.pt`
- `models/yolo_seg_test.pt`

To use team-selected URLs instead:

```bash
DETECT_MODEL_URL="https://example.com/detect.pt" \
SEG_MODEL_URL="https://example.com/seg.pt" \
bash scripts/download_test_models.sh
```

## Build

```bash
cd ~/SEA-ME_Hackathon_SMT/d_racer_autonomous/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select d_racer_perception
source install/setup.bash
```

## Run Test Nodes

```bash
ros2 launch d_racer_perception yolo_detect_test.launch.py
ros2 launch d_racer_perception yolo_seg_test.launch.py
```

Debug image topics:

- `perception/test/yolo_detect/debug/compressed`
- `perception/test/yolo_seg/debug/compressed`

Stats topics:

- `perception/test/yolo_detect/stats`
- `perception/test/yolo_seg/stats`

## Clean Removal After Testing

If the team decides not to keep these test nodes, remove this whole package:

```bash
rm -rf d_racer_autonomous/ros2_ws/src/d_racer_perception
```

Then rebuild the workspace.

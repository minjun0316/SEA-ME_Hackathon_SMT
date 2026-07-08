# Models

Trained model weights are placed here on the D3G board.

Expected files:

- `best.pt` (torch weights, trained on the SEA-ME track dataset:
  `checker`, `green`, `red`, `stop_line`)
- `best_ncnn_model/` (NCNN model used by the node by default, `imgsz=320`)

Do not commit model weights (see `.gitignore`). Export the NCNN model from
`best.pt` with:

```bash
python3 -c "from ultralytics import YOLO; YOLO('best.pt').export(format='ncnn', imgsz=320)"
```

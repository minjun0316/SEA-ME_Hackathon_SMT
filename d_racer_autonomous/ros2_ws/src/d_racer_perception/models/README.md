# Test Models

Downloaded model files are placed here on the D3G board.

Expected files:

- `yolo_detect_test.pt` (torch weights)
- `yolo_detect_test_ncnn_model/` (NCNN model used by the node)

Do not commit model weights. Run:

```bash
bash scripts/download_test_models.sh
```

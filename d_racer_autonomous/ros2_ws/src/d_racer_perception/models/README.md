# Models

D3G 보드에만 존재하는 학습 가중치. **git에 커밋 안 함**(`.gitignore`).

이 디렉토리에는 **라이브 모델만** 둔다. `.bak_*` 를 여기 쌓지 말 것 —
과거 모델은 `~/model_archive/<날짜>_<데이터셋>/` 에 보존한다(폴더마다 `WHAT.txt` 로 정체 기록).

```
best.pt              torch 가중치(학습 산출물 원본)
best_ncnn_model/     ← 런타임이 실제로 읽는 것. imgsz=320 export
```

## 파일명은 고정이다

`install → build → src` 심링크가 **파일명 단위로** 걸려 있다. 이름을 바꾸면 심링크가 끊겨
`colcon build`가 강제된다. 이름을 두고 내용만 갈아끼우면 **노드 재시작만** 하면 된다.

## 현재 모델 (2026-07-15)

YOLO26n · `smsmt_smt_coco_combined` · epochs=50 · 학습 imgsz=640 / NCNN export 320.

| id | 클래스 | 쓰는 곳 |
|----|--------|---------|
| 0 | `green` | 신호등 초록 → 출발 |
| 1 | `left` | 좌지시 팻말 → SIGN_BRANCH |
| 2 | `red` | 신호등 빨강 → 종료 |
| 3 | `right` | 우지시 팻말 → SIGN_BRANCH |

**클래스 id 순서가 계약이다.** `config/mission_cues.yaml` 의
`tl_class_green/red/left/right` 가 이 순서를 그대로 박아두고 있다. 재학습에서 순서가 바뀌면
**에러 없이 조용히 오작동**한다(예: 빨간불을 좌지시 팻말로 읽음). 아래 스크립트가 자동 대조한다.

정지선/체커는 이 모델이 담당하지 않는다(흰선 폐루프 + 정지선 노랑 HSV).
별도 팻말 모델도 미사용(`sign_model_path: ''`) — 방향은 이 통합 모델이 낸다.

## 교체 방법

```bash
./scripts/swap_model.sh ~/새로학습한.pt              # 아카이브→복사→export→검증
./scripts/swap_model.sh ~/새로학습한.pt --dry-run    # 클래스 검사만, 안 바꿈
```

수동으로 한다면 최소한 이건 지킬 것:

```bash
python3 -c "from ultralytics import YOLO; YOLO('best.pt').export(format='ncnn', imgsz=320)"
```

- `.pt` 만 복사하면 **반영 안 된다** — 런타임은 NCNN을 읽는다.
- `imgsz=320` 필수. 기본 640으로 뽑으면 9.6fps → 1.35fps.
- 모델 파일은 리빌드 불필요(심링크). 단 **`mission_cues.yaml` 을 고쳤다면 `colcon build` 필요** —
  yaml은 install이 복사본이라 안 하면 조용히 무시된다.

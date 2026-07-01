# YOLOv26n-seg (세그멘테이션 추론)

D-Racer 스케일카용 YOLOv26n-seg 모델 관련 코드·가중치를 모아두는 폴더입니다.
**학습은 노트북에서만** 진행하고, **D3G는 여기 코드+가중치를 `git pull`로 받아 실행**합니다.

## 폴더 구조

| 경로 | 용도 | git |
|------|------|-----|
| `inference/` | D3G가 실행할 추론 코드 | ✅ 올림 |
| `models/best.pt` | 배포용 가중치 (D3G가 이걸로 추론) | ✅ 올림 |
| `train/` | 학습 스크립트 (노트북 전용) | ✅ 코드만 올림 |
| `datasets/` | 학습 데이터셋 (이미지·라벨) | 🚫 gitignore (대용량) |
| `runs/` | 학습 결과·로그·중간 체크포인트 | 🚫 gitignore |

## 작업 흐름

```
[노트북]  학습/추론코드 수정 → best.pt 갱신 → git add . → git commit → git push
[D3G]     git pull   ← 최신 코드+가중치 받아서 실행
```

## 주의

- ⛔ 데이터셋·학습 로그(`datasets/`, `runs/`)는 올리지 마세요. `.gitignore`로 이미 막아뒀습니다.
- ✅ 배포용 가중치는 항상 **`models/best.pt`** 한 경로에 두고 덮어쓰세요. (D3G 코드가 이 경로를 바라봄)
- 재학습해서 성능이 좋아진 가중치가 나오면 `models/best.pt`를 교체 → commit → push 하면 D3G가 `git pull`로 받습니다.

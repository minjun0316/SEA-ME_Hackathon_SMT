# 팀 협업 가이드 (SEA-ME Hackathon 2026 / SMT)

D-Racer 스케일카 자율주행 프로젝트 팀 레포 사용법입니다. 처음 시작하는 조원은 이 문서부터 읽어주세요.

## 1. 처음 한 번만 하는 준비

1. GitHub에서 본인 계정으로 로그인 → 레포 초대 이메일/알림 **수락(Accept)**
2. 본인 **Personal Access Token(PAT)** 발급 (토큰은 절대 공유 금지, 각자 발급)
   - https://github.com/settings/tokens/new → `repo` 권한 체크 → 생성 → 값 복사 보관
3. 레포 내려받기 (clone):
   ```
   git clone https://github.com/minjun0316/SEA-ME_Hackathon_SMT.git
   ```
   - 처음 push할 때 아이디/비밀번호를 물으면 비밀번호 자리에 **PAT**를 붙여넣으세요.
4. 커밋에 쓸 이름·이메일 설정 (본인 걸로):
   ```
   git config --global user.name "본인이름"
   git config --global user.email "GitHub에 등록된 이메일"
   ```

## 2. 매번 작업하는 흐름

```
git pull                  # ① 시작 전: 팀원들 최신 작업 받기
# ... 코드 작업 ...
git add .                 # ② 바뀐 파일 담기
git commit -m "한 일 요약" # ③ 내 PC에 저장
git pull                  # ④ push 직전 한 번 더 (충돌 예방)
git push                  # ⑤ GitHub에 업로드
```

## 2-1. D3G(실차 실행용 머신)에서 코드 받기

D3G도 **노트북과 동일한 full(전체) 저장소**로 두고, 똑같이 `git pull`로 받습니다.
(예전엔 용량을 아끼려 shallow로 받았지만, 전체 히스토리가 1MB도 안 돼 이득이 없고
`git pull`이 "divergent branches" 오류로 자꾸 막혀서, 노트북과 동일하게 통일했습니다.)

```
git pull                  # 노트북과 똑같이. 이 한 줄이면 끝.
```

- ⛔ D3G에서는 **받기(pull)만** 하세요. 개발·커밋·push는 **노트북에서만** 합니다.
- 대회 중 튜닝: `[노트북] 수정→commit→push` → `[D3G] git pull` 로 최신 코드를 실행하면 됩니다.
- 만약 D3G가 예전에 shallow(얕게)로 받아둔 상태라면, 아래를 **한 번만** 실행해 full로 되돌린 뒤
  그다음부터는 `git pull`을 쓰면 됩니다:
  ```
  git fetch --unshallow origin    # 잘라냈던 과거 히스토리 전부 복원
  git reset --hard origin/dev     # 원격 최신과 완전히 동일하게 맞춤
  ```

> 요약: **노트북·D3G 모두 `git pull`** (D3G는 받기만, 커밋·push는 노트북에서)

## 3. 주의사항 (꼭 지켜주세요)

- ⛔ **대용량 파일 커밋 금지**: 이미지/영상/`.bag`/빌드 결과물 등 큰 파일은 올리지 마세요.
  레포가 무거워집니다. (실제로 가이드 이미지 174MB 때문에 한 번 정리했어요.)
- ⛔ **토큰(PAT) 공유 금지**: 각자 본인 토큰을 발급해서 쓰세요.
- ⛔ **`git push --force` 금지**: 다른 사람 작업이 날아갈 수 있어요. 막히면 먼저 상의.
- ✅ **커밋 메시지는 명확하게**: "수정" ❌ → "카메라 노드 노출값 조정" ⭕
- ✅ **push 전에 `git pull`**: 충돌을 줄여줍니다.
- ✅ **충돌(conflict)이 나면** 혼자 끙끙대지 말고 팀 채팅에 공유하기.

## 4. 자주 쓰는 명령어

| 명령어 | 설명 |
|---|---|
| `git status` | 지금 뭐가 바뀌었는지 확인 |
| `git pull` | 최신 내용 받기 (노트북·D3G 공통) |
| `git add .` | 바뀐 파일 전부 담기 |
| `git commit -m "메시지"` | 저장(스냅샷) |
| `git push` | GitHub에 업로드 |
| `git log --oneline` | 커밋 기록 보기 |

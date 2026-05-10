# Git/GitHub 협업 가이드

본 문서는 팀 프로젝트에서 GitHub 브랜치를 생성하고, 작업하고, 병합하는 기본 흐름을 정리한 문서이다.

---

## 1. 브랜치 구조

본 프로젝트에서는 다음과 같은 브랜치 구조를 사용한다.

```text
main
 └── dev
      ├── feature/본인이름
      ├── feature/팀원1
      ├── feature/팀원2
      └── hotfix
```

### 브랜치 역할

| 브랜치 | 역할 |
|---|---|
| `main` | 최종 안정 버전. 발표, 제출, 배포용 브랜치 |
| `dev` | 팀원 작업물을 통합하는 개발 브랜치 |
| `feature/본인이름` | 각 팀원이 개별 작업을 진행하는 브랜치 |
| `hotfix` | 긴급 수정이 필요한 경우 사용하는 브랜치 |

---

## 2. 기본 원칙

팀 작업 시 아래 원칙을 지킨다.

1. `main` 브랜치에 직접 push하지 않는다.
2. `dev` 브랜치에 직접 작업하지 않는다.
3. 작업 시작 전 `dev` 브랜치를 최신 상태로 만든다.
4. `dev`에서 본인 `feature` 브랜치를 생성한다.
5. 각자 `feature` 브랜치에서 작업한다.
6. 작업 완료 후 GitHub에 `feature` 브랜치를 push한다.
7. Pull Request를 통해 `feature` 브랜치를 `dev`에 merge한다.
8. `dev`에서 충분히 테스트한 후 `main`에 merge한다.
9. `__pycache__`, `.pyc` 파일은 GitHub에 올리지 않는다.

---

## 3. 현재 브랜치 확인

현재 내가 어떤 브랜치에 있는지 확인하려면 다음 명령어를 사용한다.

```bash
git branch
```

예시:

```text
  main
* dev
  feature/DaeHyeon
```

`*` 표시가 붙은 브랜치가 현재 위치한 브랜치이다.

---

## 4. GitHub 원격 브랜치 확인

GitHub에 있는 원격 브랜치 목록을 확인하려면 다음 명령어를 사용한다.

```bash
git branch -r
```

예시:

```text
origin/main
origin/dev
origin/feature/DaeHyeon
origin/feature/팀원1
origin/hotfix
```

주의할 점은 다음과 같다.

```bash
git branch
```

위 명령어는 로컬 브랜치만 보여준다.

```bash
git branch -r
```

위 명령어는 GitHub에 있는 원격 브랜치를 보여준다.

---

## 5. GitHub 브랜치 정보 최신화

팀원이 GitHub에서 브랜치를 새로 만들었거나 삭제했는데 내 컴퓨터에 반영되지 않았을 경우 다음 명령어를 사용한다.

```bash
git fetch --all --prune
```

또는 간단히 다음 명령어를 사용할 수 있다.

```bash
git fetch --prune
```

그 후 원격 브랜치 목록을 다시 확인한다.

```bash
git branch -r
```

---

## 6. 브랜치 이동

브랜치를 이동할 때는 다음 명령어를 사용한다.

```bash
git checkout 브랜치이름
```

또는 최신 Git 명령어를 사용할 수 있다.

```bash
git switch 브랜치이름
```

예시:

```bash
git checkout dev
```

```bash
git checkout main
```

---

## 7. 원격 브랜치를 로컬로 가져오기

GitHub에는 브랜치가 있지만 내 컴퓨터에는 해당 브랜치가 없을 수 있다.

예를 들어 GitHub에 `origin/dev`가 있는데 로컬에 `dev`가 없다면 다음 명령어를 사용한다.

```bash
git checkout -b dev origin/dev
```

또는 다음 명령어를 사용할 수 있다.

```bash
git switch -c dev --track origin/dev
```

팀원 브랜치를 내 컴퓨터에서 확인하고 싶을 경우:

```bash
git checkout -b feature/팀원이름 origin/feature/팀원이름
```

예시:

```bash
git checkout -b feature/DaeHyeon origin/feature/DaeHyeon
```

---

## 8. 작업 시작 흐름

새 작업을 시작할 때는 반드시 `dev` 브랜치에서 시작한다.

```bash
git checkout dev
git pull origin dev
```

그 후 본인 작업 브랜치를 만든다.

```bash
git checkout -b feature/본인이름
```

예시:

```bash
git checkout -b feature/DaeHyeon
```

이렇게 하면 현재 `dev` 브랜치에 있는 코드 전체를 기준으로 새로운 작업 브랜치가 생성된다.

파일을 직접 복사할 필요는 없다.

---

## 9. 로컬 브랜치를 GitHub에 올리기

로컬에서 새로 만든 브랜치는 처음에는 GitHub에 존재하지 않는다.

처음 한 번은 다음 명령어로 GitHub에 올린다.

```bash
git push -u origin 브랜치이름
```

예시:

```bash
git push -u origin feature/DaeHyeon
```

`-u` 옵션은 로컬 브랜치와 원격 브랜치를 연결하는 역할을 한다.

이후부터는 간단히 다음 명령어만 사용해도 된다.

```bash
git push
```

---

## 10. 작업 내용 저장 및 push

파일을 수정한 뒤 현재 상태를 확인한다.

```bash
git status
```

수정한 파일을 Git에 추가한다.

```bash
git add .
```

커밋을 생성한다.

```bash
git commit -m "작업 내용 설명"
```

GitHub에 push한다.

```bash
git push
```

처음 push하는 브랜치라면 다음 명령어를 사용한다.

```bash
git push -u origin feature/본인이름
```

---

## 11. Pull Request 생성

작업이 끝나면 GitHub에서 Pull Request를 생성한다.

기본 방향은 다음과 같다.

```text
base: dev
compare: feature/본인이름
```

즉, 본인 feature 브랜치에서 작업한 내용을 `dev` 브랜치로 합치는 것이다.

```text
feature/본인이름 → dev
```

주의할 점은 `main`으로 바로 Pull Request를 만들지 않는 것이다.

팀 작업물은 먼저 `dev`에 모은 뒤, 안정화가 끝나면 `main`으로 병합한다.

---

## 12. dev 최신 내용 가져오기

작업 중 팀원이 `dev`에 새로운 내용을 merge했다면, 내 feature 브랜치에도 최신 `dev` 내용을 반영해야 할 수 있다.

현재 내 브랜치에서 다음 명령어를 실행한다.

```bash
git pull origin dev
```

또는 다음 방식도 사용할 수 있다.

```bash
git fetch origin
git merge origin/dev
```

---

## 13. 브랜치 삭제

### 13.1 로컬 브랜치 삭제

현재 브랜치는 삭제할 수 없으므로 먼저 다른 브랜치로 이동한다.

```bash
git checkout dev
```

그 후 삭제한다.

```bash
git branch -d 브랜치이름
```

예시:

```bash
git branch -d feature/test
```

삭제가 되지 않고 아래와 같은 메시지가 나올 수 있다.

```text
error: The branch 'feature/test' is not fully merged.
```

정말 필요 없는 브랜치라면 강제 삭제한다.

```bash
git branch -D 브랜치이름
```

예시:

```bash
git branch -D feature/test
```

---

### 13.2 GitHub에서 삭제된 브랜치 반영

GitHub에서 브랜치가 삭제되었는데 내 컴퓨터에 아직 보이는 경우 다음 명령어를 사용한다.

```bash
git fetch --prune
```

그 후 확인한다.

```bash
git branch -r
```

---

## 14. `.gitignore` 설정

Python 프로젝트에서는 `__pycache__` 폴더와 `.pyc` 파일을 GitHub에 올리지 않는다.

이 파일들은 Python 실행 시 자동으로 생성되는 캐시 파일이므로, 소스코드로 관리할 필요가 없다.

`.gitignore` 파일에 다음 내용을 추가한다.

```gitignore
__pycache__/
*.pyc
```

---

## 15. 이미 올라간 `__pycache__` 제거

`.gitignore`를 추가해도 이미 GitHub에 올라간 `__pycache__` 파일이 자동으로 삭제되지는 않는다.

이미 추적 중인 `__pycache__`를 Git 관리 대상에서 제거하려면 다음 명령어를 사용한다.

PowerShell 기준:

```powershell
Get-ChildItem -Recurse -Directory -Filter "__pycache__" | ForEach-Object { git rm -r --cached $_.FullName }
```

그 후 커밋하고 push한다.

```bash
git add .gitignore
git commit -m "Remove Python cache files and update gitignore"
git push
```

---

## 16. 자주 발생하는 문제

### 16.1 `git branch`를 했는데 main만 보이는 경우

`git branch`는 로컬 브랜치만 보여준다.

GitHub에 있는 브랜치를 확인하려면 다음 명령어를 사용한다.

```bash
git branch -r
```

원격 브랜치를 로컬로 가져오려면 다음 명령어를 사용한다.

```bash
git checkout -b 브랜치이름 origin/브랜치이름
```

예시:

```bash
git checkout -b dev origin/dev
```

---

### 16.2 브랜치 이동이 안 되는 경우

다음과 같은 에러가 발생할 수 있다.

```text
Your local changes to the following files would be overwritten by checkout.
Please commit your changes or stash them before you switch branches.
```

의미는 현재 브랜치에서 수정된 파일이 있는데, 다른 브랜치로 이동하면 해당 수정사항이 덮어써질 수 있어 Git이 이동을 막은 것이다.

필요 없는 변경사항이면 버린다.

```bash
git restore 파일경로
```

작업 내용을 저장하고 이동하려면 커밋한다.

```bash
git add .
git commit -m "Save current work"
git checkout 이동할브랜치
```

임시로 숨기고 이동하려면 stash를 사용한다.

```bash
git stash
git checkout 이동할브랜치
```

나중에 다시 복구하려면 다음 명령어를 사용한다.

```bash
git stash pop
```

---

### 16.3 push가 거절되는 경우

다음과 같은 상황이 발생할 수 있다.

```text
Updates were rejected because the remote contains work that you do not have locally.
```

의미는 GitHub 브랜치에 내가 아직 가져오지 않은 팀원 커밋이 있다는 것이다.

먼저 원격 내용을 가져온 뒤 push해야 한다.

```bash
git pull origin 브랜치이름
git push origin 브랜치이름
```

예시:

```bash
git pull origin dev
git push origin dev
```

---

### 16.4 `feature/이름` 브랜치 생성이 안 되는 경우

다음과 같은 에러가 발생할 수 있다.

```text
cannot lock ref 'refs/heads/feature/DaeHyeon'
'refs/heads/feature' exists
```

이는 이미 `feature`라는 브랜치가 있어서 `feature/DaeHyeon` 형식의 브랜치를 만들 수 없다는 뜻이다.

해결 방법은 기존 `feature` 브랜치를 삭제하거나, 브랜치 이름 형식을 바꾸는 것이다.

기존 `feature` 브랜치가 필요 없다면 삭제한다.

```bash
git branch -D feature
```

GitHub에서 삭제된 원격 브랜치 정보를 정리한다.

```bash
git fetch --prune
```

그 후 다시 브랜치를 생성한다.

```bash
git checkout -b feature/DaeHyeon
```

---

## 17. Git Graph 사용 권장

브랜치 구조를 시각적으로 확인하려면 VS Code 확장 프로그램인 `Git Graph`를 사용하는 것이 좋다.

설치 방법:

```text
VS Code Extensions → Git Graph 검색 → 설치
```

실행 방법:

```text
Ctrl + Shift + P
→ Git Graph: View Git Graph
```

Git Graph를 사용하면 브랜치가 어떻게 갈라졌고, 어떤 커밋이 어느 브랜치에 있는지 시각적으로 확인할 수 있다.

---

## 18. 자주 사용하는 명령어 요약

### 현재 상태 확인

```bash
git status
git branch
git branch -r
```

### GitHub 브랜치 정보 최신화

```bash
git fetch --all --prune
```

### dev 최신화

```bash
git checkout dev
git pull origin dev
```

### 새 작업 브랜치 생성

```bash
git checkout -b feature/본인이름
```

### 브랜치 GitHub에 최초 push

```bash
git push -u origin feature/본인이름
```

### 작업 저장 및 push

```bash
git add .
git commit -m "작업 내용"
git push
```

### 원격 브랜치 로컬로 가져오기

```bash
git checkout -b 브랜치이름 origin/브랜치이름
```

### 로컬 브랜치 삭제

```bash
git branch -d 브랜치이름
```

강제 삭제:

```bash
git branch -D 브랜치이름
```

### GitHub에서 삭제된 브랜치 정보 정리

```bash
git fetch --prune
```

---

## 19. 기본 작업 흐름 요약

새 작업을 시작할 때:

```bash
git checkout dev
git pull origin dev
git checkout -b feature/본인이름
```

작업 후 저장할 때:

```bash
git status
git add .
git commit -m "작업 내용"
git push -u origin feature/본인이름
```

추가 작업 후 다시 push할 때:

```bash
git add .
git commit -m "작업 내용"
git push
```

작업 완료 후 GitHub에서 Pull Request 생성:

```text
base: dev
compare: feature/본인이름
```

---

## 20. 핵심 정리

Git 브랜치는 폴더를 복사하는 방식이 아니다.

`dev` 브랜치에서 `feature/본인이름` 브랜치를 만들면, 그 시점의 `dev` 코드 상태를 기준으로 독립적인 작업 공간이 생성된다.

각 팀원은 자신의 feature 브랜치에서 작업하고, 작업이 끝나면 Pull Request를 통해 `dev`에 병합한다.

최종적으로 안정화된 `dev` 브랜치를 `main`으로 병합한다.
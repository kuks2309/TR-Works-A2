# SSH 접속 지시서 — **TR-Works-A2** PC에 접속하기 (같은 네트워크)

> **이 문서를 받은 Claude에게:** 대상 PC(**TR-Works-A2**)와 **같은 네트워크(같은 공유기/Wi‑Fi)** 에 있다는 전제로,
> 아래 정보를 사용해 SSH 접속하세요.

---

## 1. 접속 대상 정보

| 항목 | 값 |
|------|-----|
| PC 이름 | **TR-Works-A2** |
| 사용자명 (user) | `openarmx` |
| IP 주소 (LAN) | `192.168.1.5` |
| SSH 포트 | `22` |
| 인증 방식 | SSH 공개키 (권장) 또는 비밀번호 |

---

## 2. 접속 명령

```bash
ssh openarmx@192.168.1.5
```

키 파일을 직접 지정해야 할 경우:
```bash
ssh -i /접속하는쪽/개인키경로 openarmx@192.168.1.5
```

### (선택) 별칭 등록 — `ssh tr-works-a2` 한 줄로 접속
접속하는 쪽의 `~/.ssh/config` 에 추가하면 IP를 외우지 않아도 됩니다:
```
Host tr-works-a2
    HostName 192.168.1.5
    User openarmx
    Port 22
    # IdentityFile ~/.ssh/id_ed25519
```
이후부터는 `ssh tr-works-a2` 만 입력하면 됩니다.

---

## 3. 인증 (중요)

대상 PC는 **SSH 공개키 인증**을 사용합니다.
현재 `~/.ssh/authorized_keys` 에 다음 두 키가 등록되어 접속이 허용된 상태입니다:

- `tc@tc`
- `alien@tr-works`

### 접속하는 쪽(상대편)이 해야 할 일
위 두 키 중 하나의 **개인키**를 이미 가지고 있다면 바로 접속됩니다.
없다면 아래 절차로 키를 등록해야 합니다:

1. 접속하려는 머신에 SSH 키가 없으면 먼저 생성:
   ```bash
   ssh-keygen -t ed25519 -C "접속자식별용-코멘트"
   ```
2. 생성된 **공개키** 내용 확인:
   ```bash
   cat ~/.ssh/id_ed25519.pub
   ```
3. 이 공개키 한 줄을 대상 PC 관리자(openarmx)에게 전달
   → 관리자가 대상 PC에서 `authorized_keys` 에 추가:
   ```bash
   echo "여기에-전달받은-공개키-한줄" >> ~/.ssh/authorized_keys
   ```

---

## 4. 접속 확인 / 문제 해결 체크리스트

접속이 안 될 때 순서대로 점검:

```bash
# (1) 네트워크 도달 가능 여부
ping -c 3 192.168.1.5

# (2) 22번 포트 열림 여부
nc -vz 192.168.1.5 22

# (3) 상세 로그로 접속 시도 (인증 실패 원인 파악)
ssh -v openarmx@192.168.1.5
```

| 증상 | 원인/조치 |
|------|-----------|
| `Connection refused` | sshd 미실행 → 대상 PC에서 `systemctl status ssh` 확인 |
| `Connection timed out` / `No route to host` | 같은 네트워크가 아님 → 두 기기가 같은 공유기/Wi‑Fi에 연결됐는지 확인 |
| `Permission denied (publickey)` | 공개키 미등록 → 3번 절차로 키 등록 |
| `Host key verification failed` | `ssh-keygen -R 192.168.1.5` 로 기존 호스트키 제거 후 재시도 |

---

## 5. 첫 접속 시 호스트키 확인
처음 접속하면 fingerprint 확인 메시지가 뜹니다. 신뢰 가능한 상대일 경우 `yes` 입력:
```
The authenticity of host '192.168.1.5' can't be established.
ED25519 key fingerprint is SHA256:...
Are you sure you want to continue connecting (yes/no)? yes
```

---

### 요약 (한 줄)
> **TR-Works-A2** 접속: `ssh openarmx@192.168.1.5` (같은 네트워크).
> 접속 안 되면 공개키를 `openarmx` 관리자에게 보내 `authorized_keys` 등록 요청.

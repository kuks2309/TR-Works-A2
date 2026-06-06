#!/bin/bash
# up_follower_can.sh — follower CAN 인터페이스(기본: can2=right, can3=left) 기동 전용 스크립트.
#
# 목적: HIL(실기) bringup 시 follower 팔 CAN 버스를 1 Mbps(classic CAN 2.0)로 UP.
#       leader(can0/can1, 텔레옵 입력)는 건드리지 않는다.
#
# 특성:
#   - 멱등(idempotent): 이미 UP 인 인터페이스는 건너뜀 → 재실행/launch 자동호출에 안전.
#   - 비대화형 sudo: 프로젝트 관행(run_bimanual_moveit_with_can2.0.sh)과 동일하게
#     SUDO_PASSWORD 를 echo | sudo -S 로 전달. 기본값 "ff" (환경변수로 덮어쓰기 가능).
#
# 사용:
#   ./up_follower_can.sh                 # 기본 can2 can3
#   ./up_follower_can.sh can2 can3       # 명시
#   CAN_BITRATE=1000000 SUDO_PASSWORD=xxx ./up_follower_can.sh can2 can3
#
# 종료코드: 모든 대상 인터페이스가 UP 이면 0, 하나라도 실패/부재면 1.

set -u

BITRATE="${CAN_BITRATE:-1000000}"
SUDO_PASSWORD="${SUDO_PASSWORD:-ff}"

IFACES=("$@")
if [ ${#IFACES[@]} -eq 0 ]; then
  IFACES=(can2 can3)
fi

sudo_do() { echo "$SUDO_PASSWORD" | sudo -S "$@" 2>/dev/null; }

iface_state() { ip -br link show "$1" 2>/dev/null | awk '{print $2}'; }

rc=0
for IF in "${IFACES[@]}"; do
  if ! ip link show "$IF" &>/dev/null; then
    echo "[up_follower_can] $IF: 인터페이스 없음 (CAN 어댑터 미연결?) — 건너뜀"
    rc=1
    continue
  fi

  if [ "$(iface_state "$IF")" = "UP" ]; then
    echo "[up_follower_can] $IF: 이미 UP — 건너뜀"
    continue
  fi

  echo "[up_follower_can] $IF: bitrate ${BITRATE} 로 기동..."
  sudo_do ip link set "$IF" down
  sudo_do ip link set "$IF" type can bitrate "$BITRATE"
  sudo_do ip link set "$IF" up

  if [ "$(iface_state "$IF")" = "UP" ]; then
    echo "[up_follower_can] $IF: UP ✓ (bitrate ${BITRATE})"
  else
    echo "[up_follower_can] $IF: 기동 실패 ✗ (sudo 비번/배선/어댑터 확인)"
    rc=1
  fi
done

exit $rc

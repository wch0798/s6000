BALLY S6000 UNO R4 MINIMA V6.5.14 RAM AUTO-CLEAR
==================================================

이번 수정의 목적
----------------
V6.5.13에서 RFID 입금 1 credit 시도 후 S6000 CREDIT가 그대로 0이면:

  PRE=0 EXPECT=1 NOW=0

인데도 RECOVERY_REQUIRED가 계속 남아 5초마다 같은 LP1A recovery가 반복되는 버그가 있었습니다.

V6.5.14 정책
-------------
1) Transaction / recovery는 계속 RAM ONLY. EEPROM에는 거래 journal을 쓰지 않습니다.
2) 전원 OFF / RESET 시 진행 중 거래 RAM 상태는 무조건 사라집니다.
3) 부팅 시 과거 거래 EEPROM journal은 읽지 않습니다.
4) 입금/출금 value command는 recovery에서 blind resend하지 않습니다.
5) 명확한 NO-MOVEMENT 실패는 자동으로 RAM transaction을 완전 초기화합니다.

RFID 입금 자동 판정
-------------------
PRE=0, EXPECT=1 기준:

A. NOW=1
   -> 요청 금액이 정확히 들어옴
   -> 같은 RFID 카드 balance commit

B. NOW=0 이고 추가 LP1A 3회도 모두 0
   -> 금액 이동 없음이 확정
   -> RFID 변경 없음
   -> RAM transaction EMPTY
   -> input amount 0
   -> Recovery CLEAR
   -> 다음 거래 즉시 가능

C. NOW가 PRE도 EXPECT도 아님 또는 값이 불안정함
   -> 실제 상태가 애매함
   -> value command 재전송 안 함
   -> RAM recovery 유지
   -> 전원 RESET 시 RAM recovery는 사라짐

사용자 로그에 대한 예상 동작
----------------------------
기존:
  RFID RECOVERY PRE=0 EXPECT=1 NOW=0
  meter not exact ...
  (5초마다 반복)

V6.5.14:
  RFID RECOVERY PRE=0 EXPECT=1 NOW=0
  RFID RECOVERY: PRE == NOW and stable; value did not move.
  RFID RECOVERY: auto-clear RAM transaction; next transaction enabled.
  V6 RAM TX CLEAR: RFID DEPOSIT NO MOVEMENT

그 이후 같은 recovery 로그가 반복되면 안 됩니다.

12-bit 실제 키패드 값 유지
--------------------------
#=931
1=1081
9=1598
*=1738
2=1817
0=2000
8=2224
7=2356
4=2677
3=3109
6=3385
5=4095
NO KEY ~= 13..18

기존 UI 유지
------------
카드 없음 + # 2초 : TEST MODE 진입/복귀
0 1초 길게       : 입력 금액 CLEAR
* 1회            : 입금 확인 화면
* 2회            : 실제 입금
# 1회            : 출금 확인 화면
# 2회            : 실제 원하는 금액 출금

EEPROM 정책
-----------
거래/Recovery EEPROM write = 0
Verified S6000 profile DB EEPROM 저장은 기존대로 유지

첫 실기 테스트
--------------
1) 업로드 후 status 확인
   Build = S6000-R4-MINIMA-V6.5.14-RAM-AUTO-CLEAR
   Recovery = CLEAR

2) RFID 카드 태그
3) 1 -> * -> * 로 1 credit 입금 시도
4) 만약 S6000 CREDIT가 계속 0이면 아래 로그를 확인
   DEPOSIT FAILED: LP1A unchanged at PRE; no value movement proved.
   V6 RAM TX CLEAR: DEPOSIT NO MOVEMENT
5) 이후 5초마다 RFID RECOVERY가 반복되지 않아야 함
6) journal 명령을 실행하면 EMPTY가 정상

주의
----
이 버전은 '입금이 왜 S6000에 적용되지 않았는지'를 숨기지 않습니다.
0x8A가 실제로 실패하면 자동으로 거래 상태만 깨끗이 종료하고 RFID는 차감하지 않습니다.
입금 자체의 기계 설정/Bonusing 문제는 로그를 보고 별도로 진단해야 합니다.

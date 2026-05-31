## 📦 패키지 소개

`openarmx_teleop_vr_apk`는 OpenArmX의 VR 텔레오퍼레이션 APK 설치 패키지 저장소로, VR 디바이스 측 브릿지 애플리케이션 설치 파일을 집중적으로 보관하고 배포하기 위한 전용 저장소입니다. 사용자가 손쉽게 다운로드하여 디바이스 배포를 완료할 수 있도록 합니다.

## Pico

## 1) 디바이스 연결

1. 개발자 모드를 활성화하고 USB 디버깅 모드로 진입합니다.  
   개발자 모드 활성화: `설정 > 본 기기 정보 > 소프트웨어 버전 번호를 연속해서 탭`  
   USB 디버깅 활성화: `설정 > 개발자 옵션 > USB 디버깅`
2. USB Type-C 데이터 케이블을 사용해 Pico를 PC에 연결합니다.

## 2) Pico 브릿지 APK 설치

```bash
# ADB 도구 설치
sudo apt install adb

# APK가 위치한 디렉터리로 이동
cd <다운로드 디렉터리>

# 브릿지 소프트웨어 설치
adb install openarmx-vr-pico.apk
```

## Meta quest

설치 방법은 Pico와 유사합니다. 먼저 **개발자 모드**를 열고 디바이스를 PC에 연결한 다음, adb를 통해 소프트웨어를 설치합니다.

다만 Meta Quest의 설치 절차는 다소 복잡하므로, 중국 내 사용자는 다음 영상을 참고하실 수 있습니다. [개발자 모드 열기](https://www.bilibili.com/video/BV16hyLBpE6L?buvid=XU5420159AA4697154A0DC4C9BD238EE7A6BC&from_spmid=united.player-video-detail.relatedvideo.0&is_story_h5=false&mid=xX5%2BKHmnsR16JMhsd10cQH8FTQ%2FSZMtL1rElX6M3iMo%3D&plat_id=116&share_from=ugc&share_medium=android&share_plat=android&share_session_id=82fb3460-d9a1-4610-8286-987995bca399&share_source=WEIXIN&share_tag=s_i&spmid=united.player-video-detail.0.0&timestamp=1774775521&unique_k=MR4mqSC&up_id=32985573&vd_source=b22b744a9e6ff37e0464bd12a5e08df2)


## 라이선스

본 저작물은 Creative Commons 저작자표시-비영리-동일조건변경허락 4.0 국제 라이선스 (CC BY-NC-SA 4.0)에 따라 이용 허락됩니다.

Copyright (c) 2026 Chengdu Changshu Robot Co., Ltd. (成都长数机器人有限公司)

자세한 내용은 [LICENSE_CN.md](LICENSE) 파일을 참조하시거나 다음 링크를 방문하시기 바랍니다: http://creativecommons.org/licenses/by-nc-sa/4.0/

## 작성자

- **Li QingRan** (李青燃)
- 회사: Chengdu Changshu Robot Co., Ltd. (成都长数机器人有限公司)
- 웹사이트: https://openarmx.com/

## 버전

**현재 버전**: 6.0.0

## 감사의 말

본 패키지는 OpenArmX 로봇 플랫폼 생태계의 일부이며, 협동 로봇 분야의 연구 및 산업 응용을 위해 개발되었습니다.

---

## 📞 문의

### Chengdu Changshu Robot Co., Ltd. (成都长数机器人有限公司)
**Chengdu Changshu Robotics Co., Ltd.**

| 연락처 | 정보 |
|---------|------|
| 📧 이메일 | openarmrobot@gmail.com |
| 📱 전화/WeChat | +86-17746530375 |
| 🌐 공식 웹사이트 | <https://openarmx.com/> |
| 📍 주소 | 천진 경제기술개발구 서구 신예팔가 11호 화성기계공장 |
| 👤 담당자 | Mr. Wang |

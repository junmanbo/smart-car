# smart-car

자율주행 전기차 with 회생제동

- 운전자의 안전과 편리함을 위한 주행 보조기술 구현 및 성능 개선

## 목표

- 도로 각도 예측 모델 accuracy($R^2$) 80% 이상
- 전방충돌방지 객체 인식 모델 accuracy(mAP) 60% 이상

## 방법

### 데이터 수집

- 도로 이미지 데이터를 직접 촬영하여 총 1만장으로 구성
- train data 80% validation data 20% 나눠서 각각 8천장 2천장으로 구성

### 이미지 전처리 & 모델

- 도로 이미지
    - RGB에서 HSV영역으로 변환하고 Canny edge로 경계선 구분
    - Hough transform 으로 차선 인식 용이하도록 변경
    - CNN 모델 사용

| Parameter   | Value    |
|--------------- | --------------- |
| Epoch   | 75   |
| Batch size   | 100   |
| Learning rate   | 0.001   |
| Loss function   | MSE   |

![image processing road](https://github.com/junmanbo/smart-car/assets/60846847/98ca973a-1873-4ead-8aaa-e826b7d177a8)

- 장애물 이미지
    - 이미지 size 416x416 으로 resize
    - 객체가 있는 부분 좌표로 설정하고 label 설정
    - YOLOv4-tiny 모델 사용

![image processing object](https://github.com/junmanbo/smart-car/assets/60846847/7a03ae90-9230-4d5b-82fa-e4bf3f810339)

### 회생제동

- Relay switch
    - 릴레이 스위치를 이용해 모터 드라이버에서 입력되는 PWM 차단
    - PWM 차단 후 DC모터에서 나오는 전류를 정류회로에 인가

- 정류회로 + 평활 회로
    - Bridge rectifier - Capacitor - Boot converter - Regulator 를 통해 DC 5V로 변환

### Data Augmentation

- bright, blur, zoom, pan, flip 적용
- 반전 시킬시 각도 180에서 뺀값으로 label 설정

![data augmentation](https://github.com/junmanbo/smart-car/assets/60846847/b72dc996-c836-4377-ae51-51c199b799f3)

# Capstone_Design / 스마트 데스크

## Smart Desk

### 1. 프로젝트 주제 
- 영상 처리를 활용한 스마트 책상

  ![image](https://user-images.githubusercontent.com/84834776/193454225-df15d8ef-95fe-44a7-b22b-3278c8c1b21e.png)


### 2. 제작 기간 / 참여인원 
- 2021-09-01 ~ 2022-06-22 / 전자공학 전공 4명

### 3. 사용한 기술(기술 스택)
#### 언어 및 프레임워크

- Python
- OpenCV 라이브러리
- Matplotlib 라이브러리

#### 개발환경

- Rasberry Pi OS
- VisualStudio Code
- SSH
- Github

### 4. 나의 역할
- 팀장
- Team Communication Lead
- 가속도-자이로 센서 코드 작성
- 키 측정 알고리즘 코드 구현
- OpenCV를 활용한 사용자 얼굴 인식 코드 구현 
- 높낮이 조절을 위한 모터 제어 코드 작성 
- 디스플레이 동작 코드 작성 
- 센서 데이터 값 시각화 코드 

### 5. 핵심기능
1. 사용자의 얼굴 인식된 좌표값을 통해 사용자가 앉거나 일어서는 동작을 판단하고, 책상의 높낮이를 사용자의 키에 맞는 적정 높이까지 맞춤
2. 디스플레이와 버튼 조작을 통해 동작 모드 변경 (자동 높이 조정 모드, 수동 높이 조정 모드, 수면 체크 모드)

#### ※ 기능 구현
    
- 사용자 얼굴 인식
  OpenCV DNN + Caffe SSD 모델 얼굴 인식을 Pretrain한 모델과 입력 가중치를 활용하여 사용자 얼굴 인식

  <img width="390" alt="image" src="https://user-images.githubusercontent.com/84834776/193446428-40df8f29-f8b9-4b8d-a397-4be1e0daf84b.png" align="center">
  
  <img width="300" alt="image" src="https://user-images.githubusercontent.com/84834776/193446471-a8143655-d1b9-4d5a-bcde-0c2c423450f1.png" align="center">

- 인식된 얼굴 좌표값을 활용하여 키 측정 알고리즘

  책상 카메라와 사용자 얼굴이 인식되는 가장 가까운 거리와 가장 먼 거리를 측정 후 비례식을 세워 공학적 수치 계산으로 사용자의 현재 키를 예측하도록 작성


  > 키 측정 비례식 구상도

  ![image](https://user-images.githubusercontent.com/84834776/193446760-bb6b8901-12b0-4df7-be4b-1657374504e2.png)
  ![image](https://user-images.githubusercontent.com/84834776/193446745-f52ece2f-52bb-4df2-86c6-1242adbfbee3.png)

  > 구상도 코드 구현

        ``` python
        '''
        brief : 카메라 화면 기반 사용자 신장 유도식 
        note  : 사용자의 얼굴 인식된 폭의 길이와 실제 카메라와 사용자의 거리의 비례식을 세운 수식 코드화. 카메라가 인식할 수 있는 최대 거리.
        param : faceWidth(얼굴폭), pixelX(좌표X), pixelY(좌표Y), nowHeight(현재 카메라 높이)
        return: 계산된 신장높이
        '''
        def getUserHeight1(faceWidth, pixelX, pixelY, nowHeight):
          global faceWidthAverage
          faceWidthAverage[0] = faceWidth
          sumHeight = 0
          for i in range(len(faceWidthAverage)):
            sumHeight = faceWidthAverage[i] + sumHeight
          widthAverage = sumHeight / timeNum
          fullHorizontalAngle = cameraAngle
          fullVerticalAngle = fullHorizontalAngle * cameraHeight / cameraWidth
          faceDifference = faceWidthMax - faceWidthMin
          distanceDifference = userDistanceMax - userDistanceMin
          calUserDistance = ((faceWidthMax) - widthAverage) / faceDifference * distanceDifference + userDistanceMin
          userTopAngle = abs(pixelX - cameraWidth/2) / cameraWidth * fullHorizontalAngle
          userSideAngle = abs(cameraHeight/2 - pixelY) / cameraHeight * fullVerticalAngle
          userDistance = (calUserDistance / np.cos(userTopAngle * np.pi/180))/ np.cos(userSideAngle * np.pi / 180)
          gap = calUserDistance / userDistance
          calUserDistance = ((faceWidthMax - (1 - gap) * 80) - widthAverage) / faceDifference * distanceDifference + userDistanceMin
          userDistance = (calUserDistance / np.cos(userTopAngle * np.pi / 180)) / np.cos(userSideAngle * np.pi / 180)
          cameraUserAngle = (cameraHeight/2 - pixelY) / cameraHeight * fullVerticalAngle
          calHeight = np.sin((cameraUserAngle + deskAngle) * np.pi/180) * userDistance# abs(np.sin((cameraUserAngle + deskAngle) * np.pi/180))* 15
          for i in range(timeNum-1):#shift array
            faceWidthAverage[timeNum-1-i] = faceWidthAverage[timeNum-2-i]
          return nowHeight + calHeight
          ```
  > 사용자 키에 따른 적정 책상 높이
  
  ![image](https://user-images.githubusercontent.com/84834776/193454552-b700a539-1387-451e-95ea-50c5ccb171ee.png)
  
- H/W 센서 및 디스플레이
1. 현재 책상 높이 측정 센서 : 초음파 센서
2. 책상 높낮이 제어 시 한 쪽 부하 발생으로 인한 책상 기울림 방지를 위해 수평 측정 센서 : 자이로/가속도 센서
3. 높낮이 제어를 위한 모터 : DC 모터
4. 수면 알림용 센서 : 부저 센서

    > 센서 데이터 시각화 결과
    
    ※ 왼쪽 그래프 - 앉았을 때 <--------> 오른쪽 그래프- 일어섰을 때  
    
    ![image](https://user-images.githubusercontent.com/84834776/193454662-13e46f6c-9dd8-4d64-b080-019cfd9dd0df.png)
    
    왼쪽 위 그래프 : 얼굴 인식을 통해 계산된 현재 키높이   
    오른쪽 위 그래프 : 초음파 센서를 활용한 현재 책상 높이   
    왼쪽 아래 그래프 : 가속도 자이로 센서 값을 통한 책상 수평 유지    
    오른쪽 아래 그래프 : 책상 수평을 맞추기 위한 양쪽 다리 모터 PWM 값   
    
- 전체 설계 내용

  > 전체 S/W 플로우 차트
  
  ![image](https://user-images.githubusercontent.com/84834776/193454295-14be2ebb-865c-4adc-ab1c-2bffb74a20c6.png)

  > 회로
  
  ![image](https://user-images.githubusercontent.com/84834776/193454351-ded66ada-8d29-4061-b8ec-74814f57059f.png)

### 6. 트러블슈팅

- 재료비 지원 지연
  
  > Problem
  
  재료비 지원에 지연이 생겨 부품 마련이 다소 늦어짐.
  
  > Solution
  
  부품이 마련된 후 바로 구현하기 될 수 있는 환경을 구축. 프로젝트 주요 기능을 코드로 작성하고 테스트 코드를 추가하여 테스트 작업 진행.
  
  > 진행 일정 간트 차트
  
  ![image](https://user-images.githubusercontent.com/84834776/193455106-107d57a6-1b13-49f4-882a-c3f204aacec6.png)

  
- 카메라 위치 선정

  > Problem
  
  기존에 설계했던 카메라 위치는 책상 중앙 모서리에 위치하려 했으나, 일어나 있는 상태에서 앉는 자세로 바뀌었을 때, 사용자 얼굴을 끝까지 인식 못하는 문제 발생.
  
  > Solution
  
  책상 구조를 팀원간의 합의 하에 변경. 수직으로 지지대를 세워 카메라 높이를 올리는 방안 채택.

  > 책상 카메라 연결 구조
  
  ![image](https://user-images.githubusercontent.com/84834776/193455262-b0bfd75d-0cec-4bad-b31b-b31430aaf28a.png)
  
### 7. 동작 영상 YouTube 링크

  [시연영상 Click](https://youtu.be/TopgEwy2sZo)
  
    
    
    
    
    
    
    
    
    

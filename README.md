# Embedded_KPU

## pulse_generation

![step_gen](https://user-images.githubusercontent.com/46732674/106380541-60ad3200-63f6-11eb-86e2-c25a4a182836.PNG)

- 스텝모터의 각도, 시간을 설정하여 일정 시간동안 일정 각도를 회전하도록 펄스를 발생시킨다.
- 거리와 각도 값은 USART 통신을 통해 송신한다.

![embedded (11)](https://user-images.githubusercontent.com/46732674/106380722-90106e80-63f7-11eb-992a-7a7dbcb812d1.jpg)

## sensor_remote

![sen_re](https://user-images.githubusercontent.com/46732674/106380537-5e4ad800-63f6-11eb-8d9a-90b637392810.PNG)

- 지자기센서는 TIMER을 통해 250ms마다 AD변환을 하고, 가속도센서는 연속모드를 이용하여 계속 변환하여 두 결과값을 16진수로 표시한다.
- USART를 통해 두 센서값을 따로또는 같이 수신한다.

![embedded (9)](https://user-images.githubusercontent.com/46732674/106380719-8be45100-63f7-11eb-8ee2-8624aa503f4c.jpg)

## smart_watch

![smart_w](https://user-images.githubusercontent.com/46732674/106380538-5f7c0500-63f6-11eb-98e1-b0ca1666c638.PNG)

![smart_w2](https://user-images.githubusercontent.com/46732674/106380539-60149b80-63f6-11eb-9d91-af284a65523c.PNG)

- 4가지 기능이 있는 16진수 스마트 워치 - 알람기능, 스탑워치, 계산기, 온도계

![embedded (3)](https://user-images.githubusercontent.com/46732674/106380707-7ec76200-63f7-11eb-8d0c-42b9d44472db.jpg)

- 알람 시간이 될때 마다 알람이 울린다.

![embedded (1)](https://user-images.githubusercontent.com/46732674/106380703-7b33db00-63f7-11eb-8f41-e11625460e85.jpg)

- 스탑워치를 작동시키고, 기록 버튼을 누르면 기록이 되며 정지 버튼을 누르면 스탑워치가 정지된다.

![embedded (6)](https://user-images.githubusercontent.com/46732674/106380713-838c1600-63f7-11eb-916b-50b20e18a4f4.jpg)

- USART를 통해 숫자를 입력받아 계산을 한다.

![embedded (4)](https://user-images.githubusercontent.com/46732674/106380711-80912580-63f7-11eb-9d87-eb17201f6329.jpg)

- 온도에 따라 히터와 에어컨을 제어하는 시스템이다.

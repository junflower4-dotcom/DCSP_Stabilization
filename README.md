# DCSP Stabilization Loop

C와 NI-DAQmx를 이용한 짐벌 안정화 루프 실험 코드입니다.

## 파일 구성

- `main.c`: 프로그램 초기화, 제어 루프 실행, 데이터 기록 및 종료
- `mygimbal_runexp.c`: DAQ 입출력, 센서 처리, 제어기, 데이터 저장
- `mygimbal_runexp.h`: 공통 상수와 함수 선언
- `mygimbal_stabilization.c`: 안정화 실험 입력과 실험 정보
- `mygimbal_stabilization.h`: 샘플링, 실험 시간, 출력 파일 설정

## 주요 설정

- Sampling frequency: 200 Hz
- Gyro LPF cutoff frequency: 10 Hz
- Designation controller: PD
- Stabilization controller: PI
- Emergency stop key: `f`
- Output file: `Stabilization_Loop.txt`

## DAQ 채널

- `Dev4/ai0`: Disturbance input
- `Dev4/ai2`: Gyro voltage
- `Dev4/ai3`: Potentiometer voltage
- `Dev4/ao0`: Switch command
- `Dev4/ao1`: Motor command

## 실행 환경

- Windows
- Visual Studio
- NI-DAQmx driver and C development support
- NI DAQ device configured as `Dev4`

## 주의사항

실제 장비의 채널 설정과 코드의 DAQ 채널 설정이 일치하는지 확인한 뒤 실행해야 합니다.
생성되는 실험 결과 파일은 `.gitignore`에서 기본적으로 제외됩니다.

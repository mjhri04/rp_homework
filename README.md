## 실행 방법

## 1. 워크스페이스 들어가기

cd rp_ws/

## 2. 빌드하기

colcon build

## 3. 환경 설정

source install/setup.bash

## lauch 파일 실행

ros2 launch rp_homework rp_homework.laynch.py

## +graph 확인

# 새로운 터미널을 열고

rqt_graph

---------------------------------------------------------

# 1. 워크 스페이스 만들기(없다면)

mkdir -p ~/rp_ws/src
cd rp_ws/src

# 2. 패키지 복사, 클론하기

cp -r <복사할폴더경로> <복사할위치>
 
git clone <github주소>

# 예시

cp -r ~/Downloads/rp_homework ~/rp_ws/src/

git clone https://github.com/mjhri04/rp_homework.git

# (## 2.)부터 실행
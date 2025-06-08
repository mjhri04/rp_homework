## 실행 방법

## 1-1. rp_ws 파일을 가지고 있을 때

cd ~/rp_ws/

## 1-2. git clone 했을 때

cd rp_homework/

## 2. 캐시 삭제하기

rm -rf build/ install/ log/

## 3. 빌드하기

colcon build

## 4. 환경 설정

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

# 2. 패키지 복사 or 클론하기 (둘 중 하나만 실행)

cp -r <복사할폴더경로> <복사할위치>
 
git clone <github주소>

# 예시

cp -r ~/Downloads/rp_homework ~/rp_ws/src/

git clone https://github.com/mjhri04/rp_homework.git

# (1-2.)부터 실행

# 테러리스트가 탑승한 차량을 자동으로 추적하고, 차량과 충돌하여 폭발하는 자살 공격 로봇 시스템을 개발
 
## 기간 
2024.12.03 ~ 2024.12.09

## 🔈 Summary
Turtlebot3를 기반으로 yolo tracking & follow

## 주요기능 
- world view - 실기간 target과 AMR(turtlebot)의 위치를 yolo를 활용해 파악
- Nav2 - 목표 지점까지 이동
- yolo - target을 tracking 하고 follow, target 인식
- flask - 실시간으로 AMR, world view 확인, 주요 log 확인
- SQLite - log 저장

## 💻 Tech Stack
- ROS2
- SQLite
- flask

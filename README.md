# 🤖Smart Eco Clean-Bot
:ROS2 Humble 및 Gazebo Classic 환경에서 동작하는 자율주행 기반 쓰레기 탐지 로봇. TurtleBot3 플랫폼에 카메라를 결합하여, 정해진 경로를 순찰하며 쓰
레기로 추정되는 물체를 실시간으로 인식하고 그 위치를 지도상에 기록한다. 

## 팀 구성
- 팀명: Grit
- 팀원: 이시현(2391042)
- 프로젝트 설명: 
본 프로젝트는 로봇이 정해진 구역을 스스로 순찰하며, 카메라를 통해 쓰레기로 추정되는 객체를 
실시간으로 인식하고, 해당 위치를 지도 데이터상에 마킹하여 관리자가 직관적으로 확인할 수 있
는 시스템을 구축하는 것을 목표로 한다. 쓰레기를 발견하면 일정 시간동안 정지해 쓰레기를 명
확히 인식하고 회피하며 순찰을 재개하는 지능형 행동까지 구현하였다.  
핵심 목표는 다음과 같다, 
   - LiDAR 기반 SLAM으로 자체 제작한 미로 환경의 지도를 생성하고, Nav2를 통해 다중 waypoint를 순찰하는 자율주행 구현 
   - YOLOv8 객체 인식을 통한 실시간 쓰레기(bottle, cup, bowl 등) 탐지 
   - 탐지된 위치를 map 좌표계로 변환해 RViz2 Marker로 시각화하고 SQLite 데이터베이스를 통한 탐지 이력(시간/종류/좌표/신뢰도) 기록 
   - 탐지 시 일정 시간 대기 후, 해당 지점을 회피하며 순찰을 재개하는 행동 제어

## AI 사용 여부 및 사용 내용
- Claude terminal 사용
   - Bowl 인식 문제, 중복된 물체 중 하나만 인식하는 현상이 발생했을 때, 해결법을 찾기 위해서 사용.
   - load error 및 camera blackout 문제가 발생했을 때 발생한 에러 코드를 삽입해 에러의 오류를 찾고 해결 방안을 구함

## 참고한 자료
- **YOLOv8-ROS2(객체 인식 및 토픽 발행)**: `https://github.com/mgonzs13/yolo_ros.git`
- **PythonRobotics (좌표 변환/매핑 참고)**: `https://github.com/atsushisakai/pythonrobotics`
- **3DGEMS**: `https://data.nvision2.eecs.yorku.ca/3DGEMS/`
- **Semantic map**: `https://www.dbpia.co.kr/journal/articleDetail?nodeId=NODE01955746`
- **NAV2**: `https://docs.nav2.org/setup_guides/sensors/mapping_localization.html`
- **turtlebot3**: `https://emanual.robotis.com/docs/en/platform/turtlebot3/navigation/#run-navigation-nodes`
- 
## YouTube 링크(demo video)
- **Link**: `[https://youtu.be/stE4gCkjEjo](https://youtu.be/tZcsIw1GkIw)`

## Github 링크
- **Link**: `https://github.com/Greatharmony/eco_cleanbot.git`


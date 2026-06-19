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
 
## 주요 기능
- **자율 순찰**: Nav2 `NavigateToPose` 액션으로 10개의 사전 정의된 waypoint를 순환
- **실시간 쓰레기 인식**: YOLOv8n(`yolov8n.pt`)으로 카메라 영상에서 `bottle`, `cup`, `can`, `wine glass`, `bowl`, `book` 클래스 탐지
- **시맨틱 매핑**: 탐지 위치를 `map` 좌표계 기준 RViz2 Marker(SPHERE)로 종류별 색상 구분하여 표시
- **데이터 로깅**: 모든 탐지 결과를 `~/turtlebot3_ws/trash_log.db`(SQLite)에 [시간, 종류, 좌표, 신뢰도]로 누적 저장
- **정지 및 회피**: 쓰레기 확정 탐지 시 정지 → 5초 대기 → 해당 좌표를 회피하며 순찰 재개

## 대표적인 시스템 구성
 
| 노드 | 역할 | 관련 토픽/액션 |
|---|---|---|
| `patrol_node` | waypoint 순환, `/trash_alert` 수신 시 목표 취소 후 5초 대기 및 회피 경로 보정 | `/navigate_to_pose` (action client), `/trash_alert` (구독) |
| `trash_detector` | YOLOv8 추론, TF 기반 위치 조회, 마커/로그/알림 발행 | `/camera/image_raw` (구독), `/trash_markers`, `/trash_alert`, `/cmd_vel` (발행) |
| Nav2 스택 | `map_server`, `amcl`, `planner_server`, `controller_server`, `bt_navigator`, `waypoint_follower` 등 표준 Nav2 노드 | `/scan`, `/odom`, `/tf`, `/map`, `/cmd_vel` |
 
전체 노드/토픽 연결 구조는 [`rosgraph_full.png`](./rosgraph_full.png)에서 확인할 수 있습니다.
 
## 요구 사항
 
- Ubuntu 22.04
- ROS2 Humble
- Gazebo Classic 11
- TurtleBot3 패키지
- Python: `ultralytics`(YOLOv8), `numpy==1.24.4`, `opencv`(시스템 `python3-opencv` 권장)
## 설치
 
```bash
cd ~/turtlebot3_ws/src
git clone <this-repo-url> eco_cleanbot
 
pip install ultralytics --break-system-packages
pip install "numpy==1.24.4" --break-system-packages --no-cache-dir --user
sudo apt install python3-opencv
 
cp my_maze.launch.py ~/turtlebot3_ws/src/turtlebot3_simulations/turtlebot3_gazebo/launch/
 
cd ~/turtlebot3_ws
colcon build --packages-select eco_cleanbot --symlink-install
source install/setup.bash
```
## 실행 방법
 
**터미널 1 — Gazebo 월드 실행:**
```bash
source /opt/ros/humble/setup.bash
source ~/turtlebot3_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger_cam
ros2 launch turtlebot3_gazebo my_maze.launch.py
```
 
**터미널 2 — Nav2 + 저장된 맵 실행:**
```bash
export TURTLEBOT3_MODEL=burger_cam
ros2 launch turtlebot3_navigation2 navigation2.launch.py \
  use_sim_time:=True \
  map:=$HOME/turtlebot3_ws/src/eco_cleanbot/my_maze_map.yaml
```
> RViz2에서 **2D Pose Estimate**로 로봇의 실제 위치를 먼저 보정
 
**터미널 3 — 순찰 노드:**
```bash
python3 ~/turtlebot3_ws/src/eco_cleanbot/eco_cleanbot/scripts/patrol_node.py
```
 
**터미널 4 — 쓰레기 탐지 노드:**
```bash
python3 ~/turtlebot3_ws/src/eco_cleanbot/eco_cleanbot/scripts/trash_detector.py
```
> 실행 후, RViz2에서 trash_dectector Marker 켜놓은 상태에서 수행해아됩니다.

**터미널 5 — 좌표 trace:**
```bash
ros2 topic echo /cmd_vel
```
 
## 탐지 기록 확인
 
```bash
python3 -c "
import sqlite3
conn = sqlite3.connect('$HOME/turtlebot3_ws/trash_log.db')
for row in conn.execute('SELECT * FROM detections;'):
    print(row)
"
```
 
## 주요 설정값 (`trash_detector.py`)
 
```python
TRASH_CLASSES = {'bottle', 'cup', 'can', 'wine glass', 'bowl', 'book'}
CONFIDENCE_THRESHOLD = 0.20
DETECTION_FRAMES_REQUIRED = 2
MIN_DISTANCE_BETWEEN_MARKERS = 0.5  # meters
MISS_FRAMES_ALLOWED = 5
ALERT_COOLDOWN_SEC = 5.0
```
 
`is_new_location(x, y, trash_type)`은 좌표뿐 아니라 종류까지 함께 비교하므로, 같은 위치에 서로 다른 종류의 쓰레기(예: 그릇과 병)가 있어도 각각 독립적으로 기록됩니다.

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


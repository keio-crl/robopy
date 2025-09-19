# ロボットAPI

このページでは、Robopyのロボット制御に関連するAPIを説明します。

## :material-robot: RakudaRobot

::: robopy.robots.rakuda.rakuda_robot.RakudaRobot
    options:
      members:
        - __init__
        - connect
        - disconnect
        - teleoperation
        - record
        - record_parallel

::: robopy.config.robot_config.rakuda_config.RakudaConfig


::: robopy.robots.rakuda.rakuda_pair_sys.RakudaPairSys
    options:
      members:
        - __init__
        - connect
        - disconnect
        - teleoperation
        - get_obs


::: robopy.robots.rakuda.rakuda_leader.RakudaLeader
    options:
      show_root_heading: true
      show_source: false
      members:
        - __init__
        - connect
        - disconnect
        - get_obs

::: robopy.robots.rakuda.rakuda_follower.RakudaFollower
    options:
      show_root_heading: true
      show_source: false
      members:
        - __init__
        - connect
        - disconnect
        - set_action
        - get_obs

::: robopy.robots.koch.koch_robot.KochRobot
    options:
      show_root_heading: true
      show_source: false
      members:
        - __init__
        - connect
        - disconnect
        - teleoperation

## 使用例

### 基本的なRakudaRobotの使用

```python
from robopy import RakudaConfig, RakudaRobot

# 設定の作成
config = RakudaConfig(
    leader_port="/dev/ttyUSB0",
    follower_port="/dev/ttyUSB1"
)

# ロボットの作成と接続
robot = RakudaRobot(config)
robot.connect()

try:
    # テレオペレーション
    robot.teleoperation(duration=10)
    
    # データ記録
    obs = robot.record_parallel(max_frame=1000, fps=30)
    
finally:
    robot.disconnect()
```

### 個別アームの制御

```python
from robopy.robots.rakuda import RakudaLeader, RakudaFollower

# 個別アームの作成
leader = RakudaLeader("/dev/ttyUSB0")
follower = RakudaFollower("/dev/ttyUSB1")

leader.connect()
follower.connect()

try:
    # Leaderから観測データを取得
    leader_obs = leader.get_obs()
    
    # Followerに動作指令を送信
    follower.set_action(leader_obs["action"])
    
finally:
    leader.disconnect()
    follower.disconnect()
```
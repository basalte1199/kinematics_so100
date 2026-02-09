# kinematics_so100

SO-ARM100ロボットアームの運動学パッケージです。URDFモデル、逆運動学計算、RVizビジュアライゼーションを提供します。

## 機能

- **URDFモデル**: SO-ARM100の完全な3Dモデル（メッシュ付き）
- **逆運動学**: 目標位置(x, y, z)とyaw角から関節角度を計算
- **RVizビジュアライゼーション**: ロボットモデルの表示とリアルタイム更新
- **joint_states対応**: 外部トピックからロボット姿勢を受信

## パッケージ構成

```
kinematics_so100/
├── kinematics_so100/          # Pythonノード
│   ├── inverse_kinematics_example.py  # 逆運動学計算
│   └── joint_state_publisher.py       # サンプルjoint_state publisher
├── launch/                    # Launchファイル
│   ├── display.launch.py      # RVizビジュアライゼーション
│   └── rsp.launch.py          # Robot State Publisher
├── rviz/                      # RViz設定
│   └── display.rviz
└── so100_description/         # URDFモデル
    ├── so100.urdf
    └── assets/                # 3Dメッシュファイル
```

## Installation

### 依存パッケージ

```bash
sudo apt install ros-humble-robot-state-publisher
sudo apt install ros-humble-joint-state-publisher
sudo apt install ros-humble-joint-state-publisher-gui
```

### ビルド

```bash
git clone https://github.com/basalte1199/kinematics_so100.git
git coone https://github.com/basalte1199/feetech_scs_ros2_driver.git
cd ~/feetech_ros2
colcon build --symlink-install
source install/setup.bash
```

## Usage

### 1. Pythonで目標角度司令
```bash
ros2 run kinematics_so100 joint_command_publish example
```

Python内の配列を書き換えると６秒おきに目標角度を/joint_commandに出力します。

同時に実機を起動すると、司令にしたがって実機が動きます。:

```bash
ros2 run feetech_sts_example calibration_servo_config
ros2 run feetech_sts_example read_write_position
```

### 2. RVizでロボットモデルを表示

#### GUIスライダーで操作

```bash
ros2 launch kinematics_so100 display.launch.py gui:=true
```

GUIウィンドウが表示され、スライダーで各関節を操作できます。

#### 外部トピックから制御

```bash
ros2 launch kinematics_so100 display.launch.py gui:=false
```

別のターミナルで`/joint_states`トピックをpublishすると、ロボットモデルが動きます：

```bash
ros2 topic pub /joint_states sensor_msgs/JointState "{
  header: {stamp: now, frame_id: 'base'},
  name: ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper'],
  position: [0.0, 0.5, -1.0, 0.5, 0.0, 0.5]
}" -r 10
```

別のターミナルで以下を実行して実機を起動すると、rvizと実機の角度が同期します。

```bash
ros2 run feetech_sts_example read_write_position
```

### 3. 逆運動学の計算

URDFからロボットの寸法を自動読み込みして、目標位置から関節角度を計算します。

```bash
ros2 run kinematics_so100 inverse_kinematics_example
```

**出力例:**
```
Link lengths extracted from URDF:
  l1 (base-shoulder): 0.0165 m
  l2 (shoulder-elbow): 0.1025 m
  l3 (elbow-wrist1): 0.1126 m
  l4 (wrist1-wrist2): 0.1349 m

--- Computing Inverse Kinematics ---
Target position: X=0.150m, Y=0.000m, Z=0.200m
Target yaw: 0.000 rad

--- Joint Angles (Solution Found) ---
θ1 (Shoulder Pan):    0.00° ( 0.0000 rad)
θ2 (Shoulder Lift):  45.00° ( 0.7854 rad)
θ3 (Elbow Flex):    -90.00° (-1.5708 rad)
θ4 (Wrist Flex):     45.00° ( 0.7854 rad)
θ5 (Wrist Roll):      0.00° ( 0.0000 rad)

✓ Inverse kinematics computed successfully
```

### 4. Pythonコードでの使用

```python
from kinematics_so100.inverse_kinematics_example import SO100InverseKinematics

# 初期化
ik_solver = SO100InverseKinematics('/path/to/so100.urdf')

# 逆運動学計算
target_x = 0.15  # 15cm前方
target_y = 0.0   # 中央
target_z = 0.20  # 20cm上方
target_yaw = 0.0 # ヨー角

angles = ik_solver.inverse_kinematics(target_x, target_y, target_z, target_yaw)

if angles:
    theta1, theta2, theta3, theta4, theta5 = angles
    print(f"Joint angles: {angles}")
```

## ロボット仕様

### 関節構成

| 関節名 | 説明 | 可動範囲 |
|--------|------|----------|
| shoulder_pan | ベース回転 | -2.0 ~ 2.0 rad |
| shoulder_lift | 肩ピッチ | 0.0 ~ 3.5 rad |
| elbow_flex | 肘ピッチ | -3.14 ~ 0.0 rad |
| wrist_flex | 手首ピッチ | -2.5 ~ 1.2 rad |
| wrist_roll | 手首ロール | -3.14 ~ 3.14 rad |
| gripper | グリッパー | -0.2 ~ 2.0 rad |

### リンク長

- l1 (Base-Shoulder): 16.5 mm
- l2 (Shoulder-Elbow): 102.5 mm
- l3 (Elbow-Wrist): 112.6 mm
- l4 (Wrist-End): 134.9 mm

総リーチ: 約366 mm

## トピック

### Subscribed

- `/joint_states` (sensor_msgs/JointState): ロボットの関節状態

### Published

- `/robot_description` (std_msgs/String): ロボットのURDF記述
- `/tf` (tf2_msgs/TFMessage): 座標変換情報

## トラブルシューティング

### メッシュが表示されない

```bash
# クリーンビルド
cd ~/feetech_ros2
rm -rf build/kinematics_so100 install/kinematics_so100
colcon build --packages-select kinematics_so100
source install/setup.bash
```

### joint_statesが反映されない

- `gui:=false`オプションを使用していることを確認
- `/joint_states`トピックがpublishされているか確認：

```bash
ros2 topic hz /joint_states
ros2 topic echo /joint_states
```

## 参考

- SO-ARM100公式ドキュメント
- ROS 2 Humble Documentation

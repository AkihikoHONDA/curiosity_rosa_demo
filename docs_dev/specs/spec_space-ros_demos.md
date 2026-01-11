
# 外部パッケージ仕様記述書：space-ros/demos

## 1. パッケージ概要

`space-ros/demos`は、宇宙ロボット向けミドルウェア「Space ROS」を利用したロボット制御のサンプル実装群を格納したリポジトリです 。主にDockerを利用したコンテナ環境上で動作し、Gazeboシミュレータを用いた検証環境を提供します 。

### 1.1 全体構成

リポジトリ内には以下の主要なデモパッケージが含まれています ：

* **canadarm2**: 国際宇宙ステーションのロボットアーム（SSRMS）の制御デモ 。
* **curiosity_rover**: マーズ・サイエンス・レボラトリ（Curiosity）の走行・マスト・アーム制御デモ 。
* **nav2_demo**: Nav2を用いたローバーの自律移動デモ 。
* **racs2_demos_on_spaceros**: JAXAが提供するROS2-cFS連携ブリッジ（RACS2）を用いたデモ 。
* **ros_trick**: NASAのシミュレーション環境「Trick」とROS2を連携させるデモ 。
* **space_ros_memory_allocation_demo**: C++のpmrアロケータを用いたメモリ管理のデモ 。



---

## 2. Curiosity Rover デモの仕様詳細

本プロジェクトで直接利用する `curiosity_rover` パッケージ群の詳細仕様を以下に記述します。

### 2.1 構成パッケージ

1. **curiosity_description**: ローバーの形状定義（URDF/Xacro）およびモデルデータ（Mesh） 。
2. **curiosity_gazebo**: Gazeboシミュレーション環境（Worldファイル）およびROS-Gazeboプラグイン設定 。
3. **curiosity_rover_demo**: ローバーの各部位を制御するためのノード群と、上位レベルのサービスインターフェース 。

### 2.2 ロボット構造（URDF定義）

ローバーは以下の可動部位およびセンサを備えています ：

* **Chassis**: ローバー本体 。
* **Wheels**: 6輪構成。左右それぞれ前・中・後の車輪を持つ 。ステアリング機構を含む 。
* **Arm**: 4自由度のアームとツール先端 。
* **Sensor Mast**: 旋回（yaw）および俯仰（pitch）が可能なマスト 。
* **Sensors**: LiDAR（`lidar_link`）およびカメラ（`camera_link`）がマスト先端に搭載されている 。


### 2.3 ROS2 インターフェース

`curiosity_rover_demo` の各ノードが提供するインターフェースの詳細は以下の通りです。ほとんどの操作用サービスが `std_srvs/srv/Empty` でラップされている。

#### 2.3.1 走行制御（High-Level Service）

`run_demo` ノードによって提供されます。これらのサービスを呼ぶと、内部で `Twist` メッセージが生成され、`/cmd_vel` へパブリッシュされます。

| サービス名 | 型 | 説明 |
| --- | --- | --- |
| `/move_forward` | `std_srvs/srv/Empty` | 前進（内部で `linear.x = 2.0` を継続発行） |
| `/turn_left` | `std_srvs/srv/Empty` | 左旋回（前進しつつ左へ曲がる） |
| `/turn_right` | `std_srvs/srv/Empty` | 右旋回（前進しつつ右へ曲がる） |
| `/move_stop` | `std_srvs/srv/Empty` | 停止（発行中の速度コマンドを停止・ゼロクリア） |

#### 2.3.2 マスト制御（Service / Topic）

`move_mast` ノードによって提供されます。マストの展開・収納・旋回を制御します。

| インターフェース名 | カテゴリ | 型 | 説明 |
| --- | --- | --- | --- |
| `/mast_open` | Service | `std_srvs/srv/Empty` | マストを直立（撮影可能状態）にする |
| `/mast_close` | Service | `std_srvs/srv/Empty` | マストを水平に折りたたむ（移動推奨状態） |
| `/mast_rotate` | Service | `std_srvs/srv/Empty` | マストを3段階（2秒刻み）で段階的に旋回させる |
| `/mast_joint_trajectory_controller/joint_trajectory` | Topic | `trajectory_msgs/msg/JointTrajectory` | 3関節（p, 02, cameras）の直接角度制御用 |

#### 2.3.3 アーム制御（Service / Topic）

`move_arm` ノードによって提供されます。

| インターフェース名 | カテゴリ | 型 | 説明 |
| --- | --- | --- | --- |
| `/open_arm` | Service | `std_srvs/srv/Empty` | アームを展開する |
| `/close_arm` | Service | `std_srvs/srv/Empty` | アームを収納する |
| `/arm_joint_trajectory_controller/joint_trajectory` | Topic | `trajectory_msgs/msg/JointTrajectory` | 5関節（arm_01〜04, tools）の直接制御用 |

#### 2.3.4 低レベル制御・センサ（Topic）

シミュレータへの直接指令、およびフィードバック用トピックです。

| トピック名 | 型 | 説明 |
| --- | --- | --- |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | ローバー全体の速度指令。`move_wheel` ノードが購読 |
| `/wheel_velocity_controller/commands` | `std_msgs/msg/Float64MultiArray` | 6輪それぞれの回転速度（`move_wheel` が送信） |
| `/steer_position_controller/joint_trajectory` | `trajectory_msgs/msg/JointTrajectory` | 4輪のステアリング角度制御（`move_wheel` が送信） |

#### 2.3.5 カメラ・デプスセンサ（Topic）

Gazeboシミュレータからブリッジを介して配信される観測データです。カメラモデルはマスト上に設置されているため、マストと一緒に回転する。

| **トピック名**                                                                                                           | **型**                                                                                                                                                              | **内容・仕様**                                                                                      |
| ------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------ | ---------------------------------------------------------------------------------------------- |
| **`/image_raw`**                                                                                                    | `sensor_msgs/msg/Image`                                                                                                                                            | **メインカメラ映像**<br>・解像度: 800x800 px / 10Hz<br>・視野角: 約80度 (1.396 rad)<br>・設置: マスト先端（`camera_link`） |
| **`/image_raw/compressed`**<br>**`/image_raw/compressedDepth`**<br>**`/image_raw/theora`**<br>**`/image_raw/zstd`** | **`sensor_msgs/msg/CompressedImage`**<br>**`sensor_msgs/msg/CompressedImage`**<br>**`theora_image_transport/msg/Packet`**<br>**`sensor_msgs/msg/CompressedImage`** | 上記からの派生トピック                                                                                    |
| **`/scan`**                                                                                                         | `sensor_msgs/msg/LaserScan`                                                                                                                                        | **Lidar (2Dデプス)**<br>・範囲: 1.0m 〜 20.0m<br>・サンプル数: 640<br>・設置: マスト先端（`lidar_link`）              |

---

## 4. 実行環境およびデプロイメント仕様

Curiosityデモは、Space ROSの実行環境をカプセル化したDockerコンテナとして構築されます。

### 4.1 Dockerイメージ構成

ビルドプロセスにより生成される主要なイメージは以下の2点です。

|**イメージ名**|**用途**|**ベースイメージ**|
|---|---|---|
|`osrf/space-ros:curiosity_demo`|制御ロジック、ROS 2サービス提供用|`osrf/space-ros:jazzy-2025.10.0`|
|`osrf/space-ros:curiosity_gui`|Gazeboシミュレータ、RViz2、モデル定義用|`ros:jazzy-ros-core-noble`|

### 4.2 コンテナ内ディレクトリ構成

コンテナ内部では、以下の構造でROS 2ワークスペースが展開されています。

- **ホームディレクトリ**: `/home/spaceros-user`
- **ワークスペース**: `/home/spaceros-user/curiosity_ws`
    - **ソースディレクトリ**: `~/curiosity_ws/src`
        - `curiosity_description/`: ローバーのモデル定義（GUIコンテナに配置）
        - `curiosity_gazebo/`: シミュレーション環境設定（GUIコンテナに配置）
        - `curiosity_rover_demo/`: 制御・サービス提供ノード（Demoコンテナに配置）
    - **ビルド成果物**: `~/curiosity_ws/install`

### 4.3 デモ実行コマンド

リポジトリの `curiosity_rover` ディレクトリ内で提供されているスクリプト、およびコンテナ内でのROS 2起動コマンドは以下の通りです。

#### ホスト側での操作

- **ビルドコマンド**:

	```bash
	./build.sh
	```

    （内部的に `docker compose build` を実行します）

- **起動コマンド**:

  ```bash
    ./run.sh
    ```
    
    （内部的に `docker compose up` を実行し、シミュレータと制御ノードをバックグラウンドで起動します）
    

#### コンテナ内での主要起動コマンド（自動実行設定済み）

`docker-compose.yml` により、起動時に以下のコマンドがそれぞれのコンテナで実行されます。

- **シミュレータの起動 (GUIコンテナ)**:
    
    ```bash
    ros2 launch curiosity_gazebo curiosity_gazebo.launch.py
    ```
    
- **制御ノード・サービスの起動 (Demoコンテナ)**:
    
  ```bash
    ros2 launch curiosity_rover_demo mars_rover.launch.py
    ```

---

## 5. 留意点

- ネットワーク設定:
	docker-compose.yml で network_mode: host が指定されているため、ホストマシン側で動作するLLMエージェントからコンテナ内のROS 2トピック/サービスへ直接アクセスすることが可能です。
    
- 環境変数:
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp が指定されているため、追加するノードもこれに合わせる必要があります。
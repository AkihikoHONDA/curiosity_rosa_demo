# T14: Launch統合（demo.launch.py）：追加ノード群を順序起動（sim→adapter→viz→agent）、パラメータ配線

## 背景
ここまでで Simulator / Adapter / Agent / Visualizer が個別に存在しても、デモとしては「1コマンドで起動できる」ことが重要です。  
設計では `demo.launch.py` により、追加ノード群を一括起動し、config_dir などの共通パラメータを配線し、RVizも必要なら同時起動します（design.md「9. 起動」「3.2 launch」「6.1 topics」）。

このタスクは “動くものを束ねてデモにする” ための工程です。以後の Docker導線（T15）とスモーク（T16）が楽になります。

## 目的 / 完了条件 (DoD)
- [ ] `launch/demo.launch.py` を実装し、以下を起動できる:
  - [ ] Simulator Node（T05〜T07）
  - [ ] Adapter Node（T08）
  - [ ] Visualizer Node（T13）
  - [ ] Agent Node / Console runner（T11）
  - （任意）RViz2（config/rviz.yaml を指定して起動）
- [ ] 起動順序・依存を考慮した構成にする:
  - [ ] 最低限、Simulator/Adapter/Visualizer が先に立ち上がり、Agent が後から来ても壊れない
  - [ ] “厳密な起動順制御” が難しい場合は、各ノードが service/topic待ちを行いリトライする（ログで分かる）
- [ ] 共通パラメータ配線:
  - [ ] 全ノードに `config_dir`（任意）を渡せる
  - [ ] topics.yaml に基づくトピック名は各ノードが config_loader から読む（launchで個別上書きしない）
- [ ] 起動後の最小スモーク手順が成立する:
  - [ ] `ros2 launch curiosity_rosa_demo demo.launch.py`（パッケージ名は実装に合わせる）で起動できる
  - [ ] `ros2 service list` に capture_and_score と Adapterのサービス群が見える
  - [ ] `/trace/events` が publish される（Agent起動時 or 手動入力時）
- [ ] テスト併走（実用重視）:
  - [ ] launchテスト（pytest + launch_testing）が組めるなら追加
  - [ ] 難しければ、READMEに “手動スモーク手順” をDoDとして明記し、実行手順が再現できることを固定する
- [ ] 作成・修正ファイル:
  - `curiosity_rosa_demo/launch/demo.launch.py`（実装）
  - `curiosity_rosa_demo/launch/rviz.launch.py`（任意：分割したい場合）
  - `README.md`（起動コマンド、config_dir上書き例、スモーク手順）

## 実装詳細
- 使用するクラス/関数（例）
  - `from launch_ros.actions import Node`
  - `from launch.actions import DeclareLaunchArgument`
  - `from launch.substitutions import LaunchConfiguration`
  - `config_dir = LaunchConfiguration("config_dir")`
  - `Node(package="curiosity_rosa_demo", executable="simulator_node", parameters=[{"config_dir": config_dir}])`
  - 各ノードの executable 名は setup.py の entry_points に合わせて定義する
- RViz起動（任意）
  - `Node(package="rviz2", executable="rviz2", arguments=["-d", rviz_yaml_path])`
- 参照すべき設計
  - design.md Section 9（起動と実行導線）
  - design.md Section 3.2（launch配置）
  - design.md Section 2.3（underlay/overlay: overlay側のlaunchで追加ノード起動）

## 前提条件
- 完了しておくべきタスク:
  - T02（config_dir解決）
  - T07（Simulator service）
  - T08（Adapter services）
  - T11（Agent runner）
  - T13（Visualizer）

# T15: Docker/起動導線整備（compose overrideでbind mount注入、必要なら派生Dockerfile、run補助スクリプト）

## 背景
本デモの前提は「Space ROS demos の公式Dockerイメージ上で動作させ、実装リポジトリはホストに置いて bind mount でコンテナに取り込む」ことです（design.md「2.3 コンテナとワークスペースの境界」「2.1 実行環境」）。  
つまり、ユーザが実際にデモを回す際に必要なのは “ビルド・起動の儀式” ではなく、最短の再現導線です。ここが弱いと、どれだけ中身が良くてもデモとして失敗します。

このタスクでは、overlay_ws のビルドと launch 実行までを、Docker運用として再現可能に整備します。

## 目的 / 完了条件 (DoD)
- [ ] 公式Space ROS demosイメージを前提に、ホスト側リポジトリを bind mount して overlay_ws を使える起動方法を用意する:
  - [ ] `docker compose`（推奨）または `docker run` のどちらかで、確実に再現できる手順をREADMEに記載
  - [ ] 少なくとも `overlay_ws/` がコンテナ内で見える（書き込み可能）
- [ ] overlay_ws のビルド導線を固定する:
  - [ ] 例: `./scripts/build_overlay.sh`（コンテナ内で実行）で `colcon build` が通る
  - [ ] 例: `./scripts/run_demo.sh` で `source` → `ros2 launch ...` が実行される
- [ ] 依存（Pythonライブラリやaptパッケージ）が不足する場合の扱いを決める:
  - 方針A: 公式イメージに対し、派生Dockerfileを1枚用意し `opencv` 等を追加する
  - 方針B: 起動後に `apt install` / `pip install` する（再現性が落ちるため、可能ならAを推奨）
  - ※本タスクで方針を決め、READMEに明記する
- [ ] 設定の上書き導線（config_dir）を Docker運用でも使える:
  - [ ] `config_dir=/work/curiosity_rosa_demo/overlay_ws/src/.../config` のようにパラメータを通せる
  - [ ] もしくは「bind mountにより share/config を編集」する運用で統一する
- [ ] 作成・修正ファイル:
  - `docker/`（任意：派生Dockerfile置き場）
  - `compose.yml` または `docker-compose.yml`（推奨：override含む）
  - `scripts/build_overlay.sh`（新規）
  - `scripts/run_demo.sh`（新規）
  - `README.md`（環境セットアップ、起動、トラブルシュート）

## 実装詳細
- 具体作業（例：推奨の形）
  1) `compose.yml` を用意し、公式イメージを `image:` で参照  
     - `volumes:` でリポジトリを `/work/curiosity_rosa_demo` に mount
     - 必要なら `network_mode: host`（ROS2通信の都合）や X11/Wayland（RViz）設定を加える
  2) `scripts/build_overlay.sh` で:
     - `cd /work/curiosity_rosa_demo/overlay_ws`
     - `source /opt/ros/<distro>/setup.bash`（Space ROSの構成に合わせる）
     - `colcon build --symlink-install`
  3) `scripts/run_demo.sh` で:
     - `source /work/curiosity_rosa_demo/overlay_ws/install/setup.bash`
     - `ros2 launch curiosity_rosa_demo demo.launch.py`（T14）
- 参照すべき設計
  - design.md Section 2.1 / 2.3（公式Docker＋bind mount）
  - design.md Section 9（起動導線）
  - requirements.md（成果物インベントリ：README/スクリプト類）

## 前提条件
- 完了しておくべきタスク:
  - T01（リポジトリ骨格）
  - T14（launch統合があるとrunスクリプトが確定する）

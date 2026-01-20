# Curiosity - ROSA デモ

## パッケージ説明
本リポジトリは Space ROS Curiosity デモの overlay パッケージです。
明るさスコアとキャプチャ用のシミュレータノード、アダプタ、可視化、
LLM エージェント用のコンソールを追加します。

## デモの問題設定
目的は十分に明るい画像を撮影することです。ローバーは暗い場所から
開始するため、最初の撮影は暗くなる想定です。エージェントは
ツールのフィードバック（明るさスコア）を使って、マストの回転や移動を
組み合わせ、明るい画像が得られるまで改善します。

## 環境条件
- OS: Windows 上の WSL2 Ubuntu（想定）
- コンテナランタイム: Docker
- OpenAI API Key（`OPENAI_API_KEY` を設定）
- ベースイメージのビルド: `osrf/space-ros:curiosity_demo` は配布されていないため、`https://github.com/space-ros/demos.git` をクローンし `demos/curiosity_rover` で `./build.sh` を実行してビルドします

## インストール方法
1) 本リポジトリをクローンします。
2) Curiosity デモのベースイメージをビルドします（このリポジトリの外で実行）:

```bash
git clone https://github.com/space-ros/demos.git
cd demos/curiosity_rover
./build.sh
```

3) Docker Compose 用の `.env` を用意します:

```bash
cp .env.example .env
```

対話的に生成する場合:

```bash
chmod +x overlay_ws/scripts/gen_env.sh
./overlay_ws/scripts/gen_env.sh
```

## Quick Start（Docker Compose）
1) overlay ワークスペースをビルドします:

```bash
docker compose run --rm curiosity_demo /workspace/overlay_ws/scripts/build_overlay.sh
```

2) デモノードを起動します（simulator/adapter/visualizer、RViz は任意）:

```bash
docker compose run --rm curiosity_demo /workspace/overlay_ws/scripts/run_demo.sh
```

```bash
docker compose run --rm curiosity_demo /workspace/overlay_ws/scripts/run_demo.sh use_rviz:=true
```

3) 別ターミナルでエージェントを起動します:

```bash
docker compose run --rm curiosity_demo /workspace/overlay_ws/scripts/run_agent.sh
```

スクリプトに実行権限がない場合:

```bash
docker compose run --rm curiosity_demo bash -lc "chmod +x /workspace/overlay_ws/scripts/*.sh"
```

## 最小スモークテスト（T16）
デモノード起動後に実行します。

I/F 確認:
```bash
ros2 service list | grep -E "capture_and_score|/adapter/"
ros2 topic list | grep -E "/capture/image_raw|/trace/events"
```

観測:
```bash
ros2 service call /capture_and_score curiosity_rosa_demo/srv/CaptureAndScore "{}"
```

マスト旋回:
```bash
ros2 service call /adapter/mast_rotate std_srvs/srv/Trigger "{}"
```

```bash
ros2 service call /adapter/move_forward std_srvs/srv/Trigger "{}"
```

排他確認:
```bash
ros2 service call /adapter/move_forward std_srvs/srv/Trigger "{}"
```

Trace 確認:
```bash
ros2 topic echo /trace/events --once
```

## 成果物の保存（trace/image）
成果物は `overlay_ws/artifacts` に保存します。

Trace を `overlay_ws/artifacts/trace.jsonl` に保存（Ctrl+C で停止）:
```bash
docker compose run --rm curiosity_demo /workspace/overlay_ws/scripts/save_trace.sh
```

キャプチャ画像を1枚保存（PPM）:
```bash
docker compose run --rm curiosity_demo /workspace/overlay_ws/scripts/save_capture.sh
```

## LLM Agent の規定コマンド
自然言語の指示が可能です。以下は定義済みコンソールコマンドです:
- `:help` ヘルプ表示
- `:cap` 1回撮影してスコア判定
- `:status` ローバー状態表示（`:status llm` で説明）
- `:nudge` 規定秒数前進
- `:mast_rotate` マスト旋回
- `:demo` デモ用プロンプトを実行
- `:quit` 終了

LLMに公開しているツール:
- `capture_and_score`
- `mast_rotate`
- `move_nudge`（規定秒数前進、デフォルト 20.0 秒。`config/thresholds.yaml` で変更）
- `get_status`

注記:
- 本デモの一次対応では、マストの開閉は無効化しています（mast_rotateは利用可能）。

## 手動起動（docker run）
単一のデモコンテナを起動します（追加ターミナルは `docker exec` を使用）:

```bash
docker run --rm -it --net=host \
  -u $(id -u):$(id -g) \
  -e OPENAI_API_KEY=YOUR_API_KEY \
  -v "$PWD/overlay_ws:/workspace/overlay_ws" \
  --name curiosity_demo \
  curiosity_demo_ext \
  bash
```

GUI アプリ（RViz）を同一コンテナで使う場合は X11/WSLg を通します
（環境に合わせてパスを調整してください）:

```bash
docker run --rm -it --net=host \
  -u $(id -u):$(id -g) \
  -e OPENAI_API_KEY=YOUR_API_KEY \
  -e DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -e XDG_RUNTIME_DIR=/tmp/xdg_runtime \
  -e RVIZ_CONFIG_DIR=/tmp/rviz2 \
  -e XAUTHORITY=$XAUTHORITY \
  -v "$XAUTHORITY:$XAUTHORITY" \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "$PWD/overlay_ws:/workspace/overlay_ws" \
  --name curiosity_demo \
  curiosity_demo_ext \
  bash
```

注記: ホスト側のファイルが root 所有になるのを避けるため、コンテナはホストユーザーで実行します。
拡張イメージは `ROS_LOG_DIR=/tmp/ros_log` をデフォルトで設定しています。

### 手動起動（コンテナ内）
本パッケージのデモノードを一括起動します:

```bash
source /opt/ros/*/setup.bash
source /workspace/overlay_ws/install/setup.bash
ros2 launch curiosity_rosa_demo demo.launch.py
```

エージェント（ROSA venv 必須、別ターミナル）:

```bash
source /opt/ros/*/setup.bash
source /workspace/overlay_ws/install/setup.bash
source /opt/rosa_venv/bin/activate
ros2 run curiosity_rosa_demo agent_node
```

RViz:

```bash
source /opt/ros/*/setup.bash
rviz2 -d /workspace/overlay_ws/install/curiosity_rosa_demo/share/curiosity_rosa_demo/config/rviz.rviz
```

注記: `/capture/image_raw` は Reliable QoS で配信されるため、RViz のデフォルト設定で表示できます。

## トラブルシュート
- service が見えない: デモコンテナ起動と `source /opt/ros/*/setup.bash`、`source /workspace/overlay_ws/install/setup.bash` を確認
- TF が取れない: Curiosity デモが起動していて `/tf` が流れているか確認
- 画像が出ない: `/capture_and_score` を呼んで撮影をトリガー（`debug:=true` 未設定時）
- RViz が表示されない: X11/WSLg の設定（`DISPLAY`, `/tmp/.X11-unix`）と RViz 環境変数（`XDG_RUNTIME_DIR`, `RVIZ_CONFIG_DIR`）を確認
- LLM キー未設定: `OPENAI_API_KEY` を設定し `/opt/rosa_venv/bin/activate` を実行

## テスト
テストには OpenCV と ROSA venv が必要なので、拡張イメージで pytest を実行します:

```bash
docker run --rm -it --net=host \
  -u $(id -u):$(id -g) \
  -v "$PWD/overlay_ws:/workspace/overlay_ws" \
  curiosity_demo_ext \
  bash -lc "source /opt/ros/*/setup.bash && source /opt/rosa_venv/bin/activate && cd /workspace/overlay_ws && pytest -q src/curiosity_rosa_demo/tests"
```

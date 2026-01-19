
# design.md — Space ROS Curiosity Demo × ROSA（LLM Agent）基本設計（v0.6）

## 更新履歴（v0.5 → v0.6）
本版は「Critical（I/F揺れ・設定スキーマ揺れ）」を解消するため、設計の“正”を固定した。

- Curiosity既存I/F（Service/Topic名と型）は外部仕様を正として明文化した。
- Adapter公開I/Fは **`/adapter/*` 名前空間**へ統一し、既存Serviceとの衝突を防いだ。
- Adapter公開Serviceは、失敗理由（`Need to close mast`）を返せるよう **`std_srvs/srv/Trigger`** をデフォルトにした。
- Simulator観測I/F `CaptureAndScore.srv` を「ok/error_reason + stamp」を含む形で確定した。
- `config/topics.yaml` を「curiosity実体」と「adapter公開」と「sim独自I/F」を分離して記述できるスキーマへ更新した。
- `config/prompts.yaml` を ROSA `RobotSystemPrompts` のカテゴリ（embodiment/critical/context/nuance）に寄せ、bootstrap/memory/templates を同居させた。
- ディレクトリ構成を、実装タスク群（agent/sim/adapter/tools/trace/viz）に一致させた（設計書が索引として機能するように）。

---

## 0. 前提・設計方針

### 0.1 前提
本システムは Space ROS demos の Curiosity Rover デモ（Docker Compose）上で動作する。

Curiosityデモは 2コンテナ構成で、GUIコンテナ（Gazebo/RViz）と Demoコンテナ（制御ノード群）が host network 上で動作する。  
追加ノード（Agent / Simulator / Adapter / Visualizer）は、依存一貫性と再現性のため **原則 Demoコンテナ内で起動**する。

Curiosityデモ既存I/F（走行・マスト）は以下が“正”である（Service、型は `std_srvs/srv/Empty`）。
- 走行：`/move_forward`, `/turn_left`, `/turn_right`, `/move_stop`
- マスト：`/mast_open`, `/mast_close`, `/mast_rotate`
- 画像入力：`/image_raw/compressed`（`sensor_msgs/msg/CompressedImage`）

### 0.2 入力（定型文）扱いの方針
エージェントは起動時に自動実行しない。  
ユーザーが stdin から入力を与えたタイミングで 1ターン（または短いループ）を開始する。

運用上「毎回ほぼ同じ文章」を打つことは許容するが、実装は固定文字列の自動投入ではなく、テンプレ呼び出し（ショートカット）で支援する。

本設計では、コンソールコマンドは `:` で始める（例：`:help`, `:demo`, `:cap`）。

### 0.3 ねらい（デモとして見せたい構造）
本デモは「明示的手順を与えない」ことを核にする。  
環境フィードバック（明るさスコア）と制約（マスト展開中は移動不可）から、行動系列（展開→観測→必要なら旋回→観測→必要なら収納→移動→展開→観測…）を自律合成できることを、Traceとして観測可能にする。

再現性はハード強制しない。  
プロンプト、ツール説明、概算コスト、ログ整形による「ソフト誘導」にとどめる。

---

## 1. アーキテクチャ概要

### 1.1 全体構成（ノードと責務）
- Agent Node（ROSAベース）
  - stdin入力を受け、ROSAにより推論し、tools（Python関数）を逐次実行する
  - ツール結果は ToolResult（`ok`, `error_reason`, `data`）で統一する
  - Trace（構造化イベントログ）を publish する

- Environment Simulator Node
  - `/tf` からローバの X 座標を推定し、明るさスコア（0.0〜1.0）を算出する
  - `/image_raw/compressed` を購読し、暗化加工＋スコアオーバレイ済み画像を `/capture/image_raw/compressed` に再送する
  - 観測を 1 回の操作にするための `CaptureAndScore` Service を提供する（`/capture_and_score`）

- Curiosity Adapter Node
  - Curiosityデモ既存Service（Empty）をラップし、エージェント向けの統一I/Fを提供する
  - 排他制御（マスト展開中は移動不可）を Adapter 内で担保し、違反時は `Need to close mast` を返す

- Visualizer Node
  - RViz Markerで、正解ゾーン（明るい領域）と Trace を可視化する

```mermaid
flowchart LR
  U["User"] --> CLI["Terminal stdin"]
  CLI --> AG["Agent Node (ROSA)"]
  AG --> API["LLM API"]
  AG -->| "call tools" | ADP["Curiosity Adapter Node"]
  AG -->| "capture tool" | SIM["Environment Simulator Node"]

  SIM -->| "subscribe" | IMG["/image_raw/compressed"]
  SIM -->| "publish" | CAP["/capture/image_raw/compressed"]
  SIM -->| "lookup" | TF["/tf"]

  ADP -->| "call existing" | CUR["Curiosity Demo Nodes"]

  AG -->| "publish" | TRACE["/trace/events"]
  VIZ["Visualizer Node"] -->| "subscribe" | TRACE
  VIZ -->| "publish" | RVIZ["RViz Markers"]
````

### 1.2 パターン

Ports and Adapters（Hexagonal）を採用する。

Domain（ToolResult, TraceEvent, 主要ルール）は ROSやCuriosity固有I/Fに依存させない。
Curiosity固有I/Fは Adapter に閉じ込め、差分は設定（topics.yaml）で吸収する。

---

## 2. 共存方針（Space ROS demos と ROSA の扱い）

### 2.1 Space ROS demos（Curiosity）との共存（underlay/overlay）

Space ROS demos（Curiosity）を underlay として扱い、原則改変しない。
本リポジトリは overlay として追加ノード群のみを提供する。

### 2.2 ROSA との共存（RobotSystemPrompts + tools）

ROSA は Pythonライブラリとして導入し、Agent Node 内で import して利用する。

設計上の正は以下である。

* `rosa.ROSA` をエージェントエントリポイントとして用いる
* `rosa.RobotSystemPrompts` によりプロンプトを構造化して注入する（実APIでは `about_your_environment` を利用）
* ツールは `langchain.tools.tool` で定義し、ROSA に tools リストとして渡す
* ツールは逐次実行（sequential）を前提とし、「実測優先（ROSグラフ確認）」を促す文言を system prompt に含める

### 2.3 host network を前提とした実行配置

demos の compose は host network が前提のため、ホスト側で rclpy ノードを動かす構成も理論上は可能である。
ただし本設計のデフォルトは「追加ノードも Demo コンテナ内」で統一する。

---

## 3. ディレクトリ構成（正）

設計書は実装の索引である必要があるため、実装タスク（agent/sim/adapter/tools/trace/viz）に一致させる。

```
repo_root/
  README.md
  .gitignore

  docker/                     # 任意（依存を焼き込む場合）
  overlay_ws/
    scripts/                  # build/run補助（任意）
      build_overlay.sh
      run_demo.sh
    src/
      curiosity_rosa_demo/
        package.xml
        setup.cfg
        setup.py
        resource/curiosity_rosa_demo

        launch/
          demo.launch.py

        config/
          topics.yaml
          thresholds.yaml
          tool_costs.yaml
          prompts.yaml
          rviz.yaml

        srv/
          CaptureAndScore.srv

        curiosity_rosa_demo/
          __init__.py

          agent/
            agent_node.py
            console.py
            rosa_agent_factory.py
            memory.py

          sim/
            simulator_node.py
            light_model.py
            image_pipeline.py

          adapter/
            adapter_node.py

          tools/
            tool_impl.py

          trace/
            trace_codec.py
            trace_buffer.py

          viz/
            visualizer_node.py
            marker_builder.py

          infra/
            config_loader.py
            ros_io.py

          domain/
            models.py
            serde.py

        tests/
          ...
```

---

## 4. データモデル

### 4.1 ドメインモデル（Python dataclass相当）

* RoverPose

  * `x: float`（/tfから抽出した base_frame の X 座標）
  * `frame_id: str`
  * `stamp: builtin_interfaces/Time`

* MastState

  * `is_open: bool`
  * `yaw_rad: float | None`（取得可能なら）

* LightScore

  * `score: float`（0.0〜1.0）
  * `is_good: bool`（score >= quality threshold）

* CaptureResult

  * `score: float`
  * `is_good: bool`
  * `image_topic: str`（通常 `/capture/image_raw/compressed`）
  * `stamp: builtin_interfaces/Time`

* ToolResult（統一）

  * `ok: bool`
  * `error_reason: str`（失敗時必須。排他違反時は必ず `Need to close mast`）
  * `data: dict`（成功/失敗問わず追加情報。JSON互換を守る）

* TraceEvent（構造化ログ）

  * 必須：`event_id: str`, `ts: float`, `kind: str`, `message: str`
  * 任意：`tool_name: str | None`, `ok: bool | None`, `error_reason: str | None`, `score: float | None`, `data: dict | None`
  * `kind` は `OBSERVE|HYPOTHESIZE|DECIDE|ACT|RESULT|ERROR` を正とする

Traceは「見せるためのログ」である。巨大データ（画像バイト列など）は入れない。

### 4.2 CaptureAndScore Service 定義（正）

`curiosity_rosa_demo/srv/CaptureAndScore.srv`

Requestは最小化し、観測の成否は Response の `ok/error_reason` で表現する。
stamp は返答時刻でよい（画像時刻と厳密一致させない）。

```srv
# Request
---
# Response
bool ok
float32 score
bool is_good
string error_reason
string image_topic
builtin_interfaces/Time stamp
string debug
```

失敗例は以下である。

* 画像未受信（入力画像がまだ来ていない）
* TF未解決（world→base_frame が取得できない）
* 内部例外（decode/encode 等）

---

## 5. コンポーネント／クラス設計

## 5.1 Agent Node（`agent/agent_node.py`）

### 責務

stdin入力を受けて、その都度 1 ターン（または短いループ）を実行する。
ROSA Agent を構築し、tools と memory と Trace を統合する。
TraceEvent を `/trace/events` に publish する。

### Console I/F（例）

* `:help` 使い方表示
* `:quit` 終了
* `:status` get_status tool を叩く
* `:cap` capture_and_score tool を叩く
* `:demo` デモ用テンプレ（地面テクスチャ調査）を入力に展開して実行する

### ROSAプロンプト注入

プロンプトは `config/prompts.yaml` を正とし、以下を注入する。

* RobotSystemPrompts（embodiment/critical/context/nuance）
* bootstrap（運用知識の追記。自動実行ではなく「system promptへの追記」）
* memory投入の有効/無効、投入件数

実APIの `RobotSystemPrompts` は `relevant_context` ではなく
`about_your_environment` を利用するため、Agent実装側でマッピングする。

## 5.2 Environment Simulator Node（`sim/simulator_node.py`）

### 責務

`/tf` から X を取得し、明るさスコアを算出する。
`/image_raw/compressed` を購読し、暗化＋スコア焼き込み画像を `/capture/image_raw/compressed` に publish する。
観測を一回化する Service `/capture_and_score` を提供する。

### 内部ロジック（最小）

LightModel は deterministic とし、まずは線形でよい。

* `score = clamp((x - x_min) / (x_good - x_min), 0.0, 1.0)`
* `x >= x_good` で `score = 1.0`

## 5.3 Curiosity Adapter Node（`adapter/adapter_node.py`）

### 責務

Curiosityデモ既存Service（Empty）を呼び出し、エージェント向けに **状態管理と失敗理由を返せるI/F** を提供する。

排他制御は Adapter が担保する。
マスト展開中に移動要求が来た場合、実移動Serviceを呼ばずに失敗を返し、理由文字列は必ず `Need to close mast` とする。

### Adapter公開I/F（正）

Adapterが公開するServiceは **`/adapter/*`** とし、型は **`std_srvs/srv/Trigger`** をデフォルトとする。
Triggerの `success/message` を以下に対応させる。

* `success=true` → ToolResult.ok=true（messageは空でもよい）
* `success=false` → ToolResult.ok=false（messageを error_reason として扱う）

公開例：

* `/adapter/mast_open`（Trigger）
* `/adapter/mast_close`（Trigger）
* `/adapter/mast_rotate`（Trigger）
* `/adapter/move_forward`（Trigger）
* `/adapter/turn_left`（Trigger）
* `/adapter/turn_right`（Trigger）
* `/adapter/move_stop`（Trigger）
* `/adapter/get_status`（Trigger。messageにJSON文字列を入れる）

### 内部状態（最小）

* `mast_is_open: bool`

  * mast_open成功 → true
  * mast_close成功 → false
  * mast_rotate成功 → true を維持（展開状態のまま）

`get_status` は最低限以下を返す。

* `mast_is_open`
* `move_allowed`（mast_is_open の否定）
* `last_error_reason`（保持するなら）
* 可能なら `rover_x`（Simulator/TFから取れる範囲で）

## 5.4 Visualizer Node（`viz/visualizer_node.py`）

### 責務

RViz Markerで以下を可視化する。

* 明るい正解ゾーン（bright zone）の表示
* `/trace/events` の直近 N 件を Text Marker で表示
* （任意）直近scoreの大表示

Visualizerは壊れないことを最優先とする。
Trace JSON が壊れていても落ちない。TFが取れなくても落ちない。

---

## 6. API／インターフェース定義（ROS）

### 6.1 トピック（デフォルト）

* 入力画像（Curiosityデモ）

  * subscribe: `/image_raw/compressed`（`sensor_msgs/msg/CompressedImage`）

* 出力画像（本プロジェクト）

  * publish: `/capture/image_raw/compressed`（`sensor_msgs/msg/CompressedImage`）

* Trace（本プロジェクト）

  * publish: `/trace/events`（`std_msgs/msg/String`、JSON）

* 可視化（本プロジェクト）

  * publish: `/visualization_marker_array`（`visualization_msgs/msg/MarkerArray`）

* TF（Curiosityデモ）

  * subscribe: `/tf`, `/tf_static`

### 6.2 本システムが提供する Service（デフォルト）

* `/capture_and_score`（`curiosity_rosa_demo/srv/CaptureAndScore`）

  * 観測を1回化する

* `/adapter/*`（`std_srvs/srv/Trigger`）

  * Curiosity既存Service（Empty）をラップし、成功/失敗理由を返す
  * 排他違反は `Need to close mast` を返す

### 6.3 Curiosityデモ既存I/F（Adapterが呼ぶ側：正）

Curiosity Rover デモが提供する主要I/Fは `std_srvs/srv/Empty` の Service として確定している。

* 走行：

  * `/move_forward`
  * `/turn_left`
  * `/turn_right`
  * `/move_stop`

* マスト：

  * `/mast_open`
  * `/mast_close`
  * `/mast_rotate`（段階旋回）

* 画像：

  * `/image_raw/compressed`

---

## 7. 設定ファイル設計（YAML）

設定はコードから参照する“契約”である。
したがって、キーは設計で確定し、タスクやREADMEの例も同じキーに揃える。

### 7.1 `config/thresholds.yaml`（例）

```yaml
light_model:
  x_min: 0.0
  x_good: 5.0         # ここ以降 score=1.0

quality:
  score_threshold: 0.8

trace:
  buffer_size: 30     # 直近N件（Visualizer/Memoryに利用）

viz:
  bright_zone_x_min: 5.0   # デフォルトは x_good と同値にする運用
  bright_zone_x_max: 10.0  # 表示用（シミュレーション上の無限をRVizで表せないため）
```

### 7.2 `config/topics.yaml`（例：curiosity実体と公開I/Fを分離）

```yaml
images:
  input_compressed: "/image_raw/compressed"
  output_capture_compressed: "/capture/image_raw/compressed"

trace:
  events: "/trace/events"

tf:
  base_frame: "base_footprint"
  world_frame: "odom"

services:
  capture_and_score:
    kind: "service"
    name: "/capture_and_score"
    type: "curiosity_rosa_demo/srv/CaptureAndScore"

adapter:
  # Adapterが公開する（toolsが叩く）I/F
  move_forward: { kind: "service", name: "/adapter/move_forward", type: "std_srvs/srv/Trigger" }
  turn_left:    { kind: "service", name: "/adapter/turn_left",    type: "std_srvs/srv/Trigger" }
  turn_right:   { kind: "service", name: "/adapter/turn_right",   type: "std_srvs/srv/Trigger" }
  move_stop:    { kind: "service", name: "/adapter/move_stop",    type: "std_srvs/srv/Trigger" }

  mast_open:    { kind: "service", name: "/adapter/mast_open",    type: "std_srvs/srv/Trigger" }
  mast_close:   { kind: "service", name: "/adapter/mast_close",   type: "std_srvs/srv/Trigger" }
  mast_rotate:  { kind: "service", name: "/adapter/mast_rotate",  type: "std_srvs/srv/Trigger" }

  get_status:   { kind: "service", name: "/adapter/get_status",   type: "std_srvs/srv/Trigger" }

curiosity:
  # Curiosityデモ既存Service（Adapterが呼ぶ実体）
  move_forward: { kind: "service", name: "/move_forward", type: "std_srvs/srv/Empty" }
  turn_left:    { kind: "service", name: "/turn_left",    type: "std_srvs/srv/Empty" }
  turn_right:   { kind: "service", name: "/turn_right",   type: "std_srvs/srv/Empty" }
  move_stop:    { kind: "service", name: "/move_stop",    type: "std_srvs/srv/Empty" }

  mast_open:    { kind: "service", name: "/mast_open",    type: "std_srvs/srv/Empty" }
  mast_close:   { kind: "service", name: "/mast_close",   type: "std_srvs/srv/Empty" }
  mast_rotate:  { kind: "service", name: "/mast_rotate",  type: "std_srvs/srv/Empty" }

viz:
  marker_array: "/visualization_marker_array"
```

### 7.3 `config/tool_costs.yaml`（例）

```yaml
tools:
  capture_and_score: 1
  mast_rotate: 2
  mast_open: 2
  mast_close: 2
  move_forward: 5
  turn_left: 2
  turn_right: 2
  move_stop: 1
  get_status: 1
```

### 7.4 `config/prompts.yaml`（正：RobotSystemPrompts + bootstrap + templates）

prompts.yaml は「テンプレ集」だけでなく、ROSAへ注入する構造化プロンプトを含む。

```yaml
robot_system_prompts:
  embodiment_and_persona: |
    You are an exploration rover operating in a simulated Mars environment.

  critical_instructions: |
    - Execute tools sequentially, one at a time.
    - Prefer real measurements from tools over assumptions.
    - If a move action fails with "Need to close mast", you must close the mast before moving.

  relevant_context: |
    - The camera image may be dark depending on rover position.
    - Bright area starts at X >= 5.0m (configurable).

  nuance_and_assumptions: |
    - Start with low-cost observation attempts (capture, rotate) before high-cost movement.

bootstrap:
  enabled: true
  text: |
    You have access only to the provided tools.
    Use capture_and_score first to understand the situation.

memory:
  enabled: true
  max_events: 10

templates:
  demo_ground_texture:
    label: "Analyze ground texture"
    text: "地面のテクスチャを調査して（Analyze the ground texture）"
  rover_status:
    label: "Rover status"
    text: "現在のローバーの状態を教えて"
```

### 7.5 `config/rviz.yaml`

RVizのプリセット（Fixed Frame、MarkerArray表示など）を格納する。
最小構成でよいが、「起動したら見える」状態を作ることを正とする。

---

## 8. 例外・失敗時の扱い（重要仕様）

### 8.1 排他制御（マスト展開中は移動不可）

Adapterは移動要求を受けたら `mast_is_open` を確認する。

`mast_is_open == true` の場合：

* 実移動Service（Curiosity既存）を呼ばない
* Triggerで `success=false` を返し、`message` は必ず `Need to close mast`
* ToolResult.error_reason にも必ず同文字列を入れる
* TraceEvent にも `error_reason` を載せる

### 8.2 Capture失敗

画像未受信、TF未解決などは `CaptureAndScore` の Response で `ok=false` とし、`error_reason` を埋める。

---

## 9. 起動・実行設計（Docker＋WSL2）

demos側の標準手順（`./build.sh`, `./run.sh`）を尊重する。
本プロジェクトは compose override により bind mount で注入し、Demoコンテナ内で overlay をビルド・起動する。

本プロジェクトの `demo.launch.py` は追加ノード（Simulator/Adapter/Visualizer/Agent）のみを起動する。
Curiosity本体（Gazebo/既存制御ノード）は compose 起動時に既に動いている前提である。

---

## 10. テスト計画（最小）

* ユニットテスト：LightModel、画像加工関数、Trace codec/buffer、Config loader
* 小粒度統合：CaptureAndScore、Adapter排他（Need to close mast）、/trace/events publish/subscribe
* 手動スモーク：起動→`:cap`→Trace表示→排他イベント確認→明るい領域で成功

---

## 11. 実装フェーズで確定すべき事項（残件）

* TFの base_frame/world_frame は環境ごとに異なるため、`config/topics.yaml` で調整する
* RViz Fixed Frame（map/odom等）の正
* `get_status` に含める情報の最小セット（デモで説明したい内容に合わせる）

以上

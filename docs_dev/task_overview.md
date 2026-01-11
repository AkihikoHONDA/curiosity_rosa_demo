# tasks_overview.md — Space ROS Curiosity Demo × ROSA（Session C / Step 1）【更新版】

本更新は「設計（design）と要件（requirements）を正として、task詳細のI/F揺れを解消」することを目的とする。
特に以下を正として統一する。

- Curiosity既存I/F（空リクエストの上位Service群）：
  `/move_forward`, `/turn_left`, `/turn_right`, `/move_stop`,
  `/mast_open`, `/mast_close`, `/mast_rotate`（いずれも std_srvs/srv/Empty）
  画像入力：`/image_raw/compressed`

- 本プロジェクトの公開I/F：
  加工済み画像：`/capture/image_raw/compressed`
  Adapter公開Service：`/adapter/*`（std_srvs/srv/Trigger）
  観測Service：`/capture_and_score`（curiosity_rosa_demo/srv/CaptureAndScore）

- prompts.yaml スキーマ：
  robot_system_prompts（4カテゴリ）＋ bootstrap/memory/templates を正とする。

---

| TaskID | 概要 | 対応するdesign.mdの章 | 依存タスク | 複雑度(S/M/L) |
|---|---|---|---|---|
| T01 | リポジトリ雛形（overlay_ws/ROS2 Python package骨格、基本設定ファイル群、最低限の実行スクリプト枠）を作成 | 3 | - | M |
| T02 | 設定ローダ（YAML読込、デフォルト値、パス解決、最低限のバリデーション）を実装 | 7, 2.3 | T01 | M |
| T03 | ドメインモデル（ToolResult/TraceEvent等）＋JSONシリアライズ規約を実装 | 4 | T01 | S |
| T04 | ROS I/Oユーティリティ（Service呼び出し：Empty/Trigger、Publisher/Subscriber、TF参照の最小ヘルパ）を実装 | 5.2, 5.3, 6 | T01, T03 | M |
| T05 | Simulator: 明るさモデル（tf→X推定→score算出、閾値判定、内部状態保持）を実装 | 5.2, 4.1, 7.1 | T02, T03, T04 | M |
| T06 | Simulator: 画像処理パイプライン（/image_raw/compressed購読、暗化＋スコア文字入れ、/capture/image_raw/compressedへ再Publish）を実装 | 5.2, 6.1 | T05, T04, T02 | M |
| T07 | CaptureAndScore.srv定義＋Simulator側サービス実装（観測不能はok=false/error_reasonで返す） | 4.2, 6.2 | T06, T03 | M |
| T08 | Adapter Node: Curiosity既存Service（Empty）ラップ＋排他制御（Need to close mast統一）、公開I/Fは /adapter/*（Trigger） | 5.3, 6.3, 8.1 | T02, T03, T04 | M |
| T09 | Tool実装（ROSA向けtools群）：capture_and_score / mast_* / move_* / move_stop / get_status（ToolResult統一、cost表示の土台） | 5.1, 5.3, 7.3 | T07, T08, T03, T02, T04 | M |
| T10 | ROSA統合（RosaAgentFactory、RobotSystemPrompts注入、tools登録、最小メモリ実装の足場） | 2.2, 5.1 | T09, T03, T02 | M |
| T12 | Trace基盤：/trace/events 形式の確定（codec/buffer、堅牢decode） | 4.1, 6.1 | T03 | S |
| T11 | Console/Agent Node: stdin入力（テンプレ/ショートカット含む）→ROSA実行→Trace publish の1ターンループを実装 | 5.1, 0.2, 6.1 | T10, T09, T12, T03, T02 | M |
| T13 | Visualizer Node: Trace購読→RViz MarkerArray表示（正解ゾーン、直近ログ表示、レイアウト設定） | 5.4, 7.4 | T12, T02, T04 | M |
| T14 | Launch統合（demo.launch.py）：追加ノード群を順序起動（sim→adapter→viz→agent）、パラメータ配線 | 9.4, 6 | T07, T08, T11, T13, T02 | M |
| T15 | Docker/起動導線整備：compose overrideでbind mount注入、必要なら派生Dockerfile、run補助スクリプト | 2.1, 2.3, 9.1-9.3 | T01, T14 | M |
| T16 | 動作確認・テスト併走：最小スモーク（Service存在、排他エラー文字列、capture出力、RViz表示）＋成果物保存（画像/ログ） | 10, requirements DoD | T14, T15 | M |

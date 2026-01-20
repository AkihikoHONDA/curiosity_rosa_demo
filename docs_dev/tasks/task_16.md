# T16: 動作確認・テスト併走（最小スモーク一式）＋デモ成果物保存（画像/ログ）【更新版】

## 背景
このプロジェクトの成功は、部品が存在することではなく、デモとして再現可能に動くことである。
最後に「最小スモーク一式」を体系化し、欠陥が出たとき即切り分けできる状態にする。

本更新では I/F の正を以下に統一する。
- 加工済み画像トピック：`/capture/image_raw`
- Adapter公開サービス：`/adapter/*`（std_srvs/srv/Trigger）
- 観測サービス：`/capture_and_score`（curiosity_rosa_demo/srv/CaptureAndScore）
- 排他エラー文字列：`Need to close mast`

## Variant A（一次対応）
マスト要素を一時無効化するため、マスト操作と排他制御に関する項目は **スキップ** する。

## 目的 / 完了条件 (DoD)
- [ ] “最小スモーク一式” を定義し、READMEに手順として固定すること。

  1) 起動: Docker/compose（T15）＋ launch（T14）で全ノードが起動する
  2) I/F確認: 必須service/topicが存在する
     - `/capture_and_score` service
     - `/adapter/move_forward`, `/adapter/turn_left`, `/adapter/turn_right`, `/adapter/move_stop`
     - `/adapter/mast_rotate`
     - `/adapter/get_status`
     - `/capture/image_raw` topic
     - `/trace/events` topic

  3) 観測: `capture_and_score` を実行し、ok/score/is_good/image_topic が返る
  4) 行動（向き変更）: `mast_rotate` を実行し、観測を繰り返せる
  5) 行動（移動）: `move_forward` 等を実行し、scoreが改善することがある（明るい領域へ到達）

- [ ] テスト併走（最低ライン）。
  - [ ] `pytest -q` が通る（ユニットテスト群が緑）
  - [ ] launch_testing が難しければ、手動手順をDoDで固定する

- [ ] デモ成果物の保存導線を用意すること。
  - [ ] Traceログ: `/trace/events` をファイルに保存する導線（例: `ros2 topic echo /trace/events > overlay_ws/artifacts/trace.jsonl`）
  - [ ] 画像: `/capture/image_raw` を一定回数保存するスクリプト、またはREADMEに確実な保存方法を記載する
  - [ ] 成功した一連のログと画像の置き場所を `overlay_ws/artifacts/` に固定する

- [ ] 失敗時の切り分けガイド（最小）をREADMEに追記すること。
  - serviceが見えない
  - TFが取れない
  - 画像が出ない
  - RVizが表示されない
  - LLMキー未設定でAgentが動かない

## 実装詳細
- スモークの具体コマンド例（READMEに載せる前提）
  I/F存在確認:
  - `ros2 service list | grep -E "capture_and_score|/adapter/"`
  - `ros2 topic list | grep -E "/capture/image_raw|/trace/events"`

  観測:
  - `ros2 service call /capture_and_score curiosity_rosa_demo/srv/CaptureAndScore "{}"`

  マスト操作（Trigger）:
  - `ros2 service call /adapter/mast_rotate std_srvs/srv/Trigger "{}"`

  Trace確認:
  - `ros2 topic echo /trace/events --once`

- 成果物保存（例）
  - trace: `mkdir -p overlay_ws/artifacts && ros2 topic echo /trace/events > overlay_ws/artifacts/trace.jsonl`
  - capture: CompressedImage をファイル保存する小スクリプトを推奨（JSONやechoでは扱いにくいため）

## 前提条件
- 完了しておくべきタスク:
  - T14
  - T15
  - 主要ノード実装（T07/T08/T11/T13）

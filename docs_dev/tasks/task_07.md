# T07: CaptureAndScore.srv定義＋Simulator側サービス実装（観測不能はok=false/error_reasonで返す）【更新版】

## 背景
エージェントが「観測」を行うための中核I/Fとして、Simulatorが `/capture_and_score` をサービス提供し、呼び出しに対して「加工済み画像トピック名」と「score/is_good」を返す。
観測不能（画像未受信、TF未解決など）は疑似値で誤魔化さず、`ok=false` と `error_reason` で明示する。

## 目的 / 完了条件 (DoD)
- [ ] `srv/CaptureAndScore.srv` を設計の正に沿って確定し、ビルドに載せる（`rosidl` 生成が通る）こと。

  Requestは空で良い。

  Responseは少なくとも以下を含めること。
  - `bool ok`
  - `float32 score`
  - `bool is_good`
  - `string error_reason`
  - `string image_topic`
  - `builtin_interfaces/Time stamp`
  - `string debug`（任意だが、調査効率のため付与する）

- [ ] Simulator Node が当該serviceを提供し、呼び出し時点の「直近観測」を返すこと。
  - `score/is_good` は `last_score`（T05）から取得
  - `image_topic` は加工済み画像のトピック（`topics.images.output_capture_raw`）を返す
  - `stamp` は返答時刻で良い（画像時刻と厳密一致させない）

- [ ] 観測不能時の挙動を仕様化し、実装とテストで固定すること。
  方針は以下を正とする。
  - 画像未受信、TF未解決、内部例外などの場合は `ok=false` を返す
  - `error_reason` は空にしない（例: "image not received yet", "tf lookup failed", "exception: ..."）
  - `score/is_good` はデフォルト値で返しても良いが、エージェントが誤解しないよう `ok=false` を必ず優先する

- [ ] テスト併走。
  - [ ] `colcon build` が通り、srv生成が成立する
  - [ ] 可能なら rclpy 統合テスト: service call してレスポンスの型とフィールドが期待通り
  - [ ] 少なくとも手動スモーク手順をDoDに含める（`ros2 service call ...` をREADMEに記載）

- [ ] 作成・修正ファイル。
  - `curiosity_rosa_demo/srv/CaptureAndScore.srv`
  - `curiosity_rosa_demo/sim/simulator_node.py`（service追加）
  - `curiosity_rosa_demo/tests/test_capture_and_score_srv.py`（可能なら）
  - `package.xml / setup.py / CMakeLists.txt`（srv生成に必要な追記）
  - `README.md`（手動呼び出し例追記）

## 実装詳細
- 実装メモ
  ament_pythonで `.srv` を扱う場合はビルド構成が崩れやすい。
  ここは「確実にビルドが通る構成」を優先し、最小の CMakeLists 導入などで吸収する。

## 前提条件
- 完了しておくべきタスク:
  - T06
  - T03

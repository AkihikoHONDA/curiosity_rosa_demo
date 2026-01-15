# T13: Visualizer Node: Trace購読→RViz MarkerArray表示（正解ゾーン、直近ログ表示、レイアウト設定）【更新版】

## 背景
本デモは「エージェントが状況をどう理解し、どんな行動を選んだか」を視覚的に見せる必要がある。
Visualizer Node は、(1) 明るい正解ゾーン、(2) 直近Traceログ、(3) 可能なら最新score を RViz に表示する。
本タスクは「壊れない可視化」を最優先に、MarkerArray の最小セットを作る。
正解ゾーンの設定キーは thresholds.yaml の `viz.bright_zone_x_min/x_max` を正とする（表示用）。
スコアモデル（x_good）と一致させる運用も可能だが、表示のため別キーとして保持してよい。

## 目的 / 完了条件 (DoD)
- [ ] Visualizer Node を実装し、以下の入力を購読できること。
  - [ ] `/trace/events`（std_msgs/String; JSON）を購読し、T12の `decode_event()` で復号する
  - （任意）TFを参照し、roverの現在位置を取得できる（T04のTFヘルパを使用）

- [ ] RViz表示（MarkerArray）を実装し、少なくとも以下が表示されること。
  1) 正解ゾーン（bright zone）の可視化
     - thresholds.yaml の `viz.bright_zone_x_min/x_max` を反映する

  2) 直近ログ表示
     - 直近 N 件（configで指定）を Text Marker で表示する
     - 表示内容は `TraceBuffer.as_lines()` を使用して統一する

  3) 最新score表示（できれば）
     - 直近の `capture_and_score` 結果の `score/is_good` を大きめに表示する

- [ ] rviz.yaml を導入し、RViz起動時に MarkerArray が見えるプリセットが使えること。
  - Fixed Frame が world_frame（例: map）に設定される
  - MarkerArray の表示がONになっている

- [ ] Robustness（必須）。
  - /trace/events のJSONが壊れていてもノードが落ちない（T12のフォールバックを活用）
  - TFが取れない場合でもノードが落ちない（正解ゾーンとログだけ出す）

- [ ] テスト併走。
  - [ ] ユニット: marker生成ロジックを切り出し、入力（bright zone / lines）から MarkerArray が生成されることを検証
  - [ ] 手動スモーク: `ros2 topic pub /trace/events ...` でイベントを流すと RViz のテキストが更新される

- [ ] 作成・修正ファイル。
  - `curiosity_rosa_demo/viz/visualizer_node.py`（新規）
  - `curiosity_rosa_demo/viz/marker_builder.py`（新規）
  - `curiosity_rosa_demo/tests/test_marker_builder.py`（新規）
  - `config/rviz.yaml`（最小のRViz設定）
  - `package.xml`（依存が必要なら追記）

## 前提条件
- 完了しておくべきタスク:
  - T02
  - T04
  - T12

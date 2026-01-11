# T03: ドメインモデル（ToolResult/TraceEvent等）＋JSONシリアライズ規約を実装【更新版】

## 背景
本デモでは、各ノード（Simulator / Adapter / Agent / Visualizer）がやり取りする「状態」や「観測結果」を、Python側で一貫したデータモデルとして扱う。
特に Tool 呼び出し結果は `ToolResult` に統一し、排他違反時の失敗理由は定型文 `Need to close mast` で一致させる。

また Trace は「見せるためのログ」であり、JSONに落ちること・壊れても復旧できることが重要である。
巨大データ（画像バイト列など）はTraceへ入れない。

## 目的 / 完了条件 (DoD)
- [ ] 以下の dataclass（または同等）を実装すること。

  - `RoverPose(x, frame_id, stamp)`

  - `MastState(is_open, yaw_rad=None)`

  - `LightScore(score, is_good)`

  - `CaptureResult(score, is_good, image_topic, stamp)`

  - `ToolResult(ok, error_reason, data)`

  - `TraceEvent(event_id, ts, kind, message, tool_name=None, ok=None, error_reason=None, score=None, data=None)`

- [ ] JSONシリアライズ（dict化）規約を実装すること。

  - `to_dict()` で JSON互換な `dict` を得られる（`Time` は `{sec, nanosec}` 形式等へ変換）

  - `from_dict()` で意味が保存される形で復元できること（完全一致は必須ではない）

- [ ] `ToolResult` 規約を固定すること。

  - `ok=False` の場合は `error_reason` が必須（空文字不可）

  - 排他違反時の定型文は `Need to close mast` に統一（文字列一致）

- [ ] `TraceEvent.kind` は列挙（`OBSERVE|HYPOTHESIZE|DECIDE|ACT|RESULT|ERROR`）を許可し、未知値の扱いを方針として固定すること。

  - 方針は「例外で落とす」か「UNKNOWNに落とす」のどちらかに統一する。

- [ ] テスト併走。

  - [ ] `pytest -q` が通る

  - [ ] 正常系: `CaptureResult`, `ToolResult`, `TraceEvent` を `to_dict()`→`from_dict()` して主要フィールドが保存される

  - [ ] 異常系: `ToolResult(ok=False, error_reason="")` が例外になる

  - [ ] TraceEvent.kind 不正値が方針どおりに扱われる

## 実装詳細
- ファイル案
  - `curiosity_rosa_demo/domain/models.py`

  - `curiosity_rosa_demo/domain/serde.py`

  - `curiosity_rosa_demo/tests/test_domain_models.py`

- 実装上の注意
  `ToolResult.data` と `TraceEvent.data` は拡張用の入れ物である。
  ここに ROS msg をそのまま入れず、JSON互換（dict/str/float/int/bool/list）を守る。

## 前提条件
- 完了しておくべきタスク: T01

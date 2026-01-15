# T09: Tool実装（ROSA向けtools群）：capture_and_score / mast_* / move_* / move_stop / get_status【更新版】

## 背景
エージェント（ROSA）は「自然言語→Tool呼び出し→結果観測」を繰り返して意思決定する。
本デモの Tool は、Adapter が公開する `/adapter/*`（Trigger）と、Simulator の `/capture_and_score`（独自srv）を叩く薄いラッパとして実装し、結果は `ToolResult` に統一する。
Tool層は「失敗理由の統一」と「コスト提示（ソフト誘導）」を壊さないことを最優先とする。

## 目的 / 完了条件 (DoD)
- [ ] `curiosity_rosa_demo/tools/` を作成し、以下の Tool 群を実装すること（名称は設計・promptsのツール一覧に合わせる）。
  - 観測: `capture_and_score()`
  - マスト: `mast_open()`, `mast_close()`, `mast_rotate()`
  - 移動: `move_forward()`, `turn_left()`, `turn_right()`, `move_stop()`
  - 状態: `get_status()`

- [ ] 各Toolは service 呼び出しを行い、結果を `ToolResult` で返すこと。
  - 成功: `ok=True`
  - 失敗: `ok=False` かつ `error_reason` を必ず設定
  - 排他拒否は `Need to close mast` をそのまま透過し、Tool側で文言を改変しない

- [ ] `tool_costs.yaml` を参照し、ToolResult.data に `cost`（int）を付与できること。
- [ ] `capture_and_score` は Simulator の service（T07）を呼び、少なくとも以下を `data` に含めること。
  - `score`, `is_good`, `image_topic`, `stamp`
  - `ok=false` の場合は `error_reason` を優先して扱えるようにする

- [ ] `get_status` は Adapter の状態照会を呼び、ToolResult.data を dict として返すことを推奨する。
  Trigger.message が JSON の場合は decode して dict にする（decode失敗時は raw 文字列を data に残す）。

- [ ] テスト併走。
  - [ ] rclpy統合テスト: ダミーAdapter（Trigger）とダミーCaptureAndScoreを立て、Tool呼び出しが `ToolResult` を返す
  - [ ] コスト参照テスト: tool_costs.yaml を読み、期待する cost が付く

- [ ] 作成・修正ファイル。
  - `curiosity_rosa_demo/tools/tool_impl.py`（または分割）
  - `curiosity_rosa_demo/tests/test_tools.py`
  - `config/tool_costs.yaml`（キー整形が必要なら）

## 実装詳細
- 実装メモ
  Adapterのserviceは Trigger であるため、T04の `call_trigger_service()` を使用することを正とする。
  Curiosity既存（Empty）をToolから直接呼ぶ構成にはしない。

## 前提条件
- 完了しておくべきタスク:
  - T02
  - T03
  - T04
  - T07
  - T08

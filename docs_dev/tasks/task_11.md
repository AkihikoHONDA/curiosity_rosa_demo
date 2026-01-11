# T11: Console/Agent Node: stdin入力（テンプレ/ショートカット含む）→ROSA実行→Trace publish の1ターンループ【更新版】

## 背景
ROSA統合（T10）だけでは、実際に「人間が状況を与える→エージェントが考えてツールを叩く→結果が見える」まで到達しない。
本デモでは stdin からの入力で1ターンずつ回し、Trace（/trace/events）を可視化（T13）へ流す導線が必要である。

prompts.yaml の正は以下である。
- templates による入力支援（ショートカット）
- bootstrap.enabled/text は system prompt への追記（自動実行ではない）
- memory.enabled/max_events は短期履歴投入の制御

Trace形式は T12 の codec を正とし、publish側は codec を利用して統一する。

## 目的 / 完了条件 (DoD)
- [ ] Agent Node（または console runner）を実装し、1ターンのループが成立すること。

  1) stdin でユーザ入力を受ける

  2) ROSA に入力を渡して実行する

  3) tool実行結果を受け、要約を stdout に表示する

  4) TraceEvent を `/trace/events` に publish する（T12のcodecでJSON化）

- [ ] 入力支援（テンプレ/ショートカット）を実装すること（最小でよい）。

  - 例: `:help`（使い方表示）

  - 例: `:quit`（終了）

  - 例: `:status`（get_statusを叩く）

  - 例: `:cap`（capture_and_scoreを叩く）

  - 例: `:demo`（prompts.yamlのテンプレを挿入して実行）

- [ ] 設定で bootstrap を付与できること。

  - prompts.yaml の `bootstrap.enabled/text` を読み、system prompt へ追記する。

- [ ] `/trace/events` への publish 形式は T12 に従い、単一箇所で生成できること。

  - `std_msgs/msg/String` に JSON を流す（encodeは trace_codec）

  - decode不能なものを出さない（publish前に最低限の型整合を取る）

- [ ] テスト併走。

  - [ ] ユニット: 「tool呼び出し→TraceEvent化→JSON化」が最低限成立する

  - [ ] 手動スモーク手順を README に記載する

    起動→`:cap`→結果表示→`/trace/events` が出る（`ros2 topic echo` で確認可能）

- [ ] 作成・修正ファイル。

  - `curiosity_rosa_demo/agent/agent_node.py`（新規）

  - `curiosity_rosa_demo/agent/console.py`（任意）

  - `curiosity_rosa_demo/tests/test_agent_trace_format.py`（新規）

  - `README.md`（操作手順追記）

## 実装詳細
- 実装方針の注意（rclpy とstdinの共存）
  方式Aとして、Nodeは別スレッドで spin し、メインスレッドで stdin を読む方式を推奨する。
  Ctrl+C で止まり、ハングしないことを優先する。

## 前提条件
- 完了しておくべきタスク:
  - T02

  - T03

  - T09

  - T10

  - T12

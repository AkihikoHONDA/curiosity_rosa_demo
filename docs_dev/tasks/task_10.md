# T10: ROSA統合（RosaAgentFactory、RobotSystemPrompts注入、tools登録、最小メモリ実装の足場）【更新版】

## 背景
本デモは「Space ROS（Curiosity）上のロボット操作」を、ROSA（LLM Agent）で自律計画・実行させることが目的である。
ここまでで Tool 群（T09）と、観測（T07）・排他制御（T08）・データモデル（T03）が揃っているため、このタスクでは「ROSAにツールを渡し、構造化プロンプトを注入して、実際に1ターン実行できる状態」を作る。

prompts.yaml の正は、RobotSystemPrompts の4カテゴリに対応する `robot_system_prompts` と、bootstrap/memory/templates の同居である。

## 目的 / 完了条件 (DoD)
- [ ] ROSA のカスタムエージェント生成（Factory）を実装し、以下を満たすこと。
  - [ ] prompts.yaml の `robot_system_prompts`（4カテゴリ）を読み込み、`rosa.RobotSystemPrompts` へ対応付けて注入できること。
  - [ ] `bootstrap.enabled/text` が有効な場合、system prompt へ追記できること（自動実行はしない）。
  - [ ] T09 の tools を Agent に登録できること。
  - [ ] Agent の実行（1回の推論→tool呼び出し→結果受領）を起動できること。

- [ ] 最小メモリ（短期履歴）の足場を用意すること。
  - [ ] `memory.enabled/max_events` を読み取り、直近N件のTrace（またはTool実行履歴）を保持できるコンテナを実装する。
  - [ ] メモリ投入を無効化できること（enabled=false）。

- [ ] LLMプロバイダ設定（APIキー・モデル名等）はコードに埋め込まないこと。
  - [ ] 環境変数（例：`OPENAI_API_KEY`）で受ける、またはROS paramで受ける。
  - [ ] 未設定時は「何が不足か」が一目でわかる例外メッセージにする。

- [ ] テスト併走（最低限）。
  - [ ] ユニット: tools 登録リストが期待通り構成される（tool名が揃う）
  - [ ] “LLMを呼ばない” スモークが可能なら、Fake/Stub LLM を差し替えられる設計にする
  - [ ] 手動スモーク手順を README に記載する（APIキー設定時の最短起動）

- [ ] 作成・修正ファイル。
  - `curiosity_rosa_demo/agent/rosa_agent_factory.py`（新規）
  - `curiosity_rosa_demo/agent/memory.py`（新規）
  - `curiosity_rosa_demo/tests/test_rosa_integration_smoke.py`（新規）
  - `config/prompts.yaml`（スキーマ整形が必要なら）

## 実装詳細
- 実装メモ
  ROSA側のAPI変動に備え、Factoryの外へ漏れる型を最小化する。
  失敗時（APIキー無し等）は握りつぶさず、設定原因であることが分かる文言を返す。

## 前提条件
- 完了しておくべきタスク:
  - T02
  - T03
  - T09

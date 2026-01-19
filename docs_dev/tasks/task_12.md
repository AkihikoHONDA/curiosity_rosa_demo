# T12: Trace基盤：/trace/events 形式の確定（イベント種別・tool名・score・ok/error_reason等）＋記録（直近N件）

## 背景
LLMエージェントのデモは「何を考え、何を試し、なぜ失敗し、どう復帰したか」が見えないと説得力が落ちます。  
そこで設計では Trace を “人間に見せるための観測窓” として用意し、Agent/Tool実行の各段階を `TraceEvent` として `/trace/events` に流し、Visualizer（T13）で表示します（design.md「4.1 TraceEvent」「6.1 trace topic」「5.4 Visualizer」）。

T11で最低限 publish はできますが、ここで「イベント種別」「メッセージ書式」「保持件数」「scoreの載せ方」を固定しないと、RViz表示の安定性と、デバッグ効率が崩れます。

## 目的 / 完了条件 (DoD)
- [ ] TraceEventフォーマット（JSON）を確定し、コード上で単一箇所から生成できる:
  - [ ] 必須フィールド: `event_id`, `ts`, `kind`, `message`
  - [ ] 任意フィールド: `tool_name`, `ok`, `error_reason`, `score`, `data`（tool resultの一部を入れてもよいが、巨大化は禁止）
  - [ ] `kind` は列挙（`OBSERVE|HYPOTHESIZE|DECIDE|ACT|RESULT|ERROR`）を正とし、未知値の扱いを固定（T03の方針に一致）
- [ ] “直近N件リングバッファ” を実装し、Agent Node から参照できる:
  - [ ] `TraceBuffer(maxlen=N)` を実装
  - [ ] `append(event)` / `latest(n)` / `as_lines(n)`（Visualizer向けの整形）を提供
  - [ ] N は config（thresholds.yaml か prompts.yaml）で変更可能
- [ ] publish側（T11）と購読側（T13）が同じフォーマットに依存できるよう、以下を提供する:
  - [ ] `trace_codec.py`：`encode_event(event) -> str` / `decode_event(json_str) -> TraceEvent`
  - [ ] decode失敗時のフォールバック（壊れたJSONを受けてもVisualizerが死なない）
- [ ] テスト併走:
  - [ ] `pytest -q` が通る
  - [ ] 正常系: event→json→event で主要フィールドが保存される
  - [ ] 異常系: 壊れたJSONを decode しても例外で落ちず、`kind=ERROR` 等のフォールバックイベントに変換される
- [ ] 作成・修正ファイル:
  - `curiosity_rosa_demo/trace/trace_buffer.py`（新規）
  - `curiosity_rosa_demo/trace/trace_codec.py`（新規）
  - `curiosity_rosa_demo/tests/test_trace_codec.py`（新規）
  - `curiosity_rosa_demo/tests/test_trace_buffer.py`（新規）
  - （必要なら）T11 `agent_node.py` の Trace生成箇所を `trace_codec` 利用に置換

## 実装詳細
- 使用するクラス/関数（例）
  - `class TraceBuffer:`
    - `__init__(self, maxlen: int)`
    - `append(self, ev: TraceEvent) -> None`
    - `latest(self, n: int) -> list[TraceEvent]`
    - `as_lines(self, n: int) -> list[str]`
      - 例: `"[ACT] move_nudge ok=true"`, `"[RESULT] score=0.12 BAD"` 等
  - `encode_event(ev: TraceEvent) -> str`
    - `json.dumps(ev.to_dict(), ensure_ascii=False)`
  - `decode_event(s: str) -> TraceEvent`
    - 失敗時は `TraceEvent(kind="ERROR", message="trace decode failed", ...)`
- 参照すべき設計
  - design.md Section 4.1（TraceEventの項目）
  - design.md Section 6.1（/trace/events）
  - design.md Section 5.4（Visualizerが表示したい情報）
- 実装上の注意
  - traceは “見せるためのログ” なので、巨大な画像バイト列などは入れない。
  - decodeの堅牢性が重要（Visualizerが落ちるとデモが死ぬ）。

## 前提条件
- 完了しておくべきタスク:
  - T03（TraceEventモデル / serde）
  - T11（publish側があると統合が楽だが、Trace基盤自体はT11なしでも作れる）

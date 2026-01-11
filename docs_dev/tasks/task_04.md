# T04: ROS I/Oユーティリティ（Service呼び出し、Publisher/Subscriber、TF参照の最小ヘルパ）【更新版】

## 背景
本デモは Curiosity demos の既存I/F（主に `std_srvs/srv/Empty`）を Adapter が呼び出し、Agent/Tools は Adapter が公開する `std_srvs/srv/Trigger` を呼ぶ構成を正とする。
よって、ROS I/O ユーティリティは Empty と Trigger の両方を「同じ流儀（timeout＋ToolResult）」で扱える必要がある。

TF参照、Publisher/Subscriber 生成の定型処理もここに集約し、後続タスクでのコピペ増殖を防ぐ。

## 目的 / 完了条件 (DoD)
- [ ] `curiosity_rosa_demo/infra/ros_io.py` を新規作成し、以下を提供すること。

- [ ] Service 呼び出し（Empty系）を timeout 付きで実行できること。

  - [ ] `wait_for_service(node, service_name, timeout_sec) -> bool`

  - [ ] `call_empty_service(node, service_name, timeout_sec) -> ToolResult`

    正常系は `ok=True`。

    異常系（timeout/unavailable/exception）は `ok=False` とし、`error_reason` を必ず埋める。

- [ ] Service 呼び出し（Trigger系）を timeout 付きで実行できること。

  - [ ] `call_trigger_service(node, service_name, timeout_sec) -> ToolResult`

    Trigger の `success/message` を以下へ対応させる。

    `success=true` → `ToolResult(ok=True)`（messageは data へ入れても良い）

    `success=false` → `ToolResult(ok=False, error_reason=message)`（messageが空ならフォールバック文言を入れる）

- [ ] Publisher/Subscriber 生成のヘルパを提供すること。

  - [ ] `create_compressed_image_pub(node, topic, qos=...)`

  - [ ] `create_compressed_image_sub(node, topic, callback, qos=...)`

  - [ ] `create_trace_pub(node, topic, qos=...)`（`std_msgs/msg/String`）

- [ ] TF参照ユーティリティを提供すること。

  - [ ] `create_tf(node) -> (tf2_ros.Buffer, tf2_ros.TransformListener)`

  - [ ] `lookup_x_in_world(tf_buffer, world_frame, base_frame, timeout_sec) -> ToolResult`

    `ok=True` の場合、`data={"x": float}` を返す。

    失敗時は `ok=False` とし、`error_reason` に例外要約を入れる。

- [ ] テスト併走。

  - [ ] `pytest -q` が通る

  - [ ] 正常系: テスト内で `std_srvs/srv/Empty` のダミーServiceを立て、`call_empty_service()` が `ok=True`

  - [ ] 正常系: テスト内で `std_srvs/srv/Trigger` のダミーServiceを立て、`call_trigger_service()` が `ok=True/False` を反映

  - [ ] 異常系: 存在しないservice名を指定し、timeoutで `ok=False` となり `error_reason` が空でない

  - [ ] TF正常系: `map -> base_link` の static transform を流し、`lookup_x_in_world()` が `ok=True` で `x` を返す

## 実装詳細
- 実装メモ
  Service呼び出しは「待つ＋呼ぶ＋timeout」を1箇所に閉じる。
  rclpy の `call_async()` と `spin_until_future_complete()`（または executor）で timeout を実装する。

  Trigger の message は、排他違反（Need to close mast）の伝搬に使用されるため、空にしない。

## 前提条件
- 完了しておくべきタスク: T01, T03

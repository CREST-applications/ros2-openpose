# DEMO

転倒検知アプリケーションの詳細な技術構成について説明します。

## ROS2 Components

この Mermaid フローチャートは、[ros2_graph](https://github.com/kiwicampus/ros2_graph) によって生成しています。

```mermaid
flowchart LR
    /out/compressed(["/out/compressed<br>sensor_msgs/msg/CompressedImage"]) --> /display["/display"] & /pose_requester["/pose_requester"]
    /pose(["/pose<br>std_msgs/msg/String"]) --> /display
    /image_raw/compressed(["/image_raw/compressed<br>sensor_msgs/msg/CompressedImage"]) --> /image_republisher["/image_republisher"]
    /image_republisher --> /out/compressed
    /pose_requester --> /pose
    /v4l2_camera["/v4l2_camera"] --> /image_raw/compressed

    /out/compressed:::topic
    /display:::main
    /pose_requester:::main
    /pose_requester:::node
    /pose:::topic
    /image_raw/compressed:::topic
    /image_republisher:::main
    /image_republisher:::node
    /v4l2_camera:::main
    /v4l2_camera:::node
    classDef node opacity:0.9,fill:#2A0,stroke:#391,stroke-width:4px,color:#fff
    classDef action opacity:0.9,fill:#66A,stroke:#225,stroke-width:2px,color:#fff
    classDef service opacity:0.9,fill:#3B8062,stroke:#3B6062,stroke-width:2px,color:#fff
    classDef topic opacity:0.9,fill:#852,stroke:#CCC,stroke-width:2px,color:#fff
    classDef main opacity:0.9,fill:#059,stroke:#09F,stroke-width:4px,color:#fff
    classDef bugged opacity:0.9,fill:#933,stroke:#800,stroke-width:2px,color:#fff
```

### Nodes

- **/v4l2_camera**:

  カメラ画像を生成するためのノード。ROS2 の [v4l2_camera](https://index.ros.org/p/v4l2_camera/) パッケージを利用しています。
  また、カメラ画像を JPEG 圧縮するための [image_transport](https://wiki.ros.org/image_transport) パッケージをプラグインとして利用しています。

  - topics:
    - publication: `/image_raw/compressed`

- **/image_republisher**:

  TB3 から Mini-PC への画像転送を効率化するためのプロキシ。TB3 からの画像トピックを受信し、Mini-PC 内で再配信する。
  これにより、TB3 <--> Mini-PC 間のユニキャストを 1 本にし、Raspberry Pi の負荷を軽減します。

  - topics:
    - subscription: `/image_raw/compressed`
    - publication: `/out/compressed`

- **/pose_requester**:

  MEC-RM に対して、OpenPose によるポーズ推定をリクエストするためのノードです。

  - topics:
    - subscription: `/out/compressed`
    - publication: `/pose`

- **/display**:
  
  画像とポーズのキーポイントをマージして表示するためのノードです。
  将来的に、マージプロセスを分離し、`rqt_image_view` などの既存のツールを利用するように変更する予定です。

  - topics:
    - subscription: `/out/compressed`
    - subscription: `/pose`

### Settings

[compose.yml](../compose.yml) から、ある程度の設定が可能となっています。

## MEC-RM



### Future Work

```mermaid
flowchart LR
    /out/compressed(["/out/compressed<br>sensor_msgs/msg/CompressedImage"]) -->  /pose_requester["/pose_requester"] & /renderer
    /pose(["/pose<br>std_msgs/msg/String"]) --> /renderer
    /image_raw/compressed(["/image_raw/compressed<br>sensor_msgs/msg/CompressedImage"]) --> /image_republisher["/image_republisher"]
    /image_republisher --> /out/compressed
    /pose_requester --> /pose
    /v4l2_camera["/v4l2_camera"] --> /image_raw/compressed
    /renderer --> /rendered_image(["/rendered_image<br>sensor_msgs/msg/CompressedImage"])
    /rendered_image --> /display

    /out/compressed:::topic
    /display:::main
    /pose_requester:::main
    /pose_requester:::node
    /pose:::topic
    /image_raw/compressed:::topic
    /image_republisher:::main
    /image_republisher:::node
    /v4l2_camera:::main
    /v4l2_camera:::node
    /renderer:::node
    /rendered_image:::topic
    classDef node opacity:0.9,fill:#2A0,stroke:#391,stroke-width:4px,color:#fff
    classDef action opacity:0.9,fill:#66A,stroke:#225,stroke-width:2px,color:#fff
    classDef service opacity:0.9,fill:#3B8062,stroke:#3B6062,stroke-width:2px,color:#fff
    classDef topic opacity:0.9,fill:#852,stroke:#CCC,stroke-width:2px,color:#fff
    classDef main opacity:0.9,fill:#059,stroke:#09F,stroke-width:4px,color:#fff
    classDef bugged opacity:0.9,fill:#933,stroke:#800,stroke-width:2px,color:#fff
```

- **/renderer**:

  画像とポーズのキーポイントをマージして、新しい画像を生成するノードです。
  このノードにより、任意のツールで画像を表示することが可能となります。

  - topics:
    - subscription: `/pose`
    - publication: `/rendered_image`

- **/display**:

  [`rqt_image_view`](https://wiki.ros.org/rqt_image_view) による画像とポーズの表示を行うようにします。

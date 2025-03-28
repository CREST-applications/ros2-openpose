# Getting Started

これは、介護施設デモのうち、転倒検知アプリケーションを実行するためのマニュアルです。

## Worker (Desktop)

1. MEC-RM に接続できるネットワークで、電源を入れる。以上。

## TurtleBot3

1. **TurtleBot3 の起動**

   バッテリーまたは電源ケーブルに接続し、OpenCR (下段の青い基盤) の電源スイッチを入れます。

2. **TurtleBot3 へログイン**

    TurtleBot3 と同じネットワークの任意の端末から SSH で接続します。

    - hostname: `tb3-sasaki.local`
    - password: `user`

   ```bash
   ssh user@tb3-sasaki.local
   ```

3. **カメラを起動**

   ```bash
   ros2 run v4l2_camera v4l2_camera_node
   ```

   `ctrl + c` で終了します。

## Requester (Mini-PC)

1. **起動し、ログイン**

   - username: `user`
   - password: `user`

2. **アプリケーションの起動 (Docker)**

   ```bash
   cd ~/workspace/ros2-openpose # ワークスペースに移動
   docker compose up
   ```

   `ctrl + c` で終了します。

## Worker (Desktop-PC)

1. 電源を入れて、ログインするだけです。

   - username: `user`
   - password: `user`

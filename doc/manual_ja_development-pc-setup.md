## 開発用ワークステーションのセットアップ

### Debian バイナリパッケージからのインストール

Debian バイナリパッケージからソフトウェアのインストールを行います．

#### ROS および HIRO / NEXTAGE OPEN ソフトウェアのインストール

ROS Indigo および HIRO，NEXTAGE OPEN のパッケージをインストールします．
ターミナルを開いて次のコマンドを実行してください．．

```
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu trusty main" > /etc/apt/sources.list.d/ros-latest.list'
$ sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net --recv-key0xB01FA116
$ sudo apt-get update && sudo apt-get install ros-indigo-rtmros-nextage ros-indigo-rtmros-hironx python-catkin-tools
```

インストールの最後に setup.bash を読み込み，ROS の環境を設定します．

```
$ source /opt/ros/indigo/setup.bash
```

これは新しくターミナルを立ち上げて ROS を使用する前に毎回必要になります．
下記のように .bashrc ファイルに設定を加えて
ターミナル起動時に setup.bash を自動で実行し ROS 環境になるようにしておくと便利です．

```
$ echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
```

- 注意: 上記コマンドの `>>` を `>` にしてしまうと元々あった .bashrc 内の設定が消えてしまうので気をつけてください．


#### HIRO / NEXTAGE OPEN ソフトウェアのみのインストール

ROS Indigo が既にインストールされていて HIRO / NEXTAGE OPEN のソフトウェアのみをインストールする必要がある場合は次の1行だけ実行してください．

```
$ sudo apt-get update && sudo apt-get install ros-indigo-rtmros-nextage ros-indigo-rtmros-hironx
```

### ネットワーク設定

#### ホスト名の設定（オプション）

デフォルトの HIRO / NEXTAGE OPEN のソフトウェアでは QNX コンピュータのホスト名を通信の際に使います．
ホスト名の設定をオペレーティングシステム上で行っておくと便利です．

- 次の行を `/etc/hosts` に追加
```
# For accessing QNX for NEXTAGE Open
192.168.128.10 nextage
```
  - IP アドレスはユーザの環境により上記のものとは異なる場合があります．
  - `ping` を打って nextage から返ってくることを確認してください．
  - シミュレータを利用するために上記の設定を変更する必要はありません．
  - `192.168.128.x` セグメントを使用するネットワークアプリケーションを使用している場合を除き，
この設定はネットワークの使用には有害ではありません．

- 既知の OpenRTM の問題  https://github.com/start-jsk/rtmros_hironx/issues/33 を回避するため
ネットワーク接続に `eth0` が使用されていることを確認してください．


### ワークスペースのセットアップ

HIRO / NEXTAGE OPEN のプログラムコードを作成・ビルドする必要がない場合は
本項目のインストール手順は不要です．

下記のセットアップ手順ではワークスペース名を `catkin_ws` (catkin workspace) としていますが
他の名前・フォルダ名でも大丈夫です．

```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws
$ wstool init src
$ wstool merge -t src https://raw.githubusercontent.com/tork-a/rtmros_nextage/indigo-devel/hironxo.rosinstall
$ wstool update -t src
$ source /opt/ros/indigo/setup.bash
$ rosdep update && rosdep install -r -y --from-paths src --ignore-src
$ catkin build
$ source devel/setup.bash
```

ターミナルを開くたびに `source ~/catkin_ws/devel/setup.bash`
を実行したくない場合は .bashrc に設定します．

```
$ echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```

- 注意: 上記コマンドの `>>` を `>` にしてしまうと元々あった .bashrc 内の設定が消えてしまうので気をつけてください．


<!-- EOF -->

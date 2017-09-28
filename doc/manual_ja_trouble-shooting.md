
# トラブルシューティング

## ソフトウェア情報の確認

### コントローラボックス（QNX）の hrpsys のバージョンの確認

Python インタフェースから次のコマンドを実行して
コントローラボックス（QNXコンピュータ）で動作している hrpsys のバージョンを確認してください．

```
In [1]: robot.hrpsys_version
Out[1]: '315.2.8
```

コントローラボックスにログオンできる場合にはリモート接続をして
次のコマンドを実行して確認することもできます．

- 注意: バージョン 315.2.0 以降にて可能

```
$ cd /opt/jsk/lib
$ ls -l RobotHardware.so  
$ strings RobotHardware.so | grep ^[0-9]*\\.[0-9]*\\.[0-9]*$
315.2.0
```


## RTM ステートのモニタリング

ロボットホスト上で動作しているRTコンポーネントのリストを取得するには `rtls` を利用します．

- 注意: 最後のスラッシュ `/` を忘れずに入力してください．

```
$ rtls %HOST_ROBOT%:15005/
```

> シミュレーションの場合の例
>
> ```
> $ rtls localhost:15005/
> ```
>
> Nextage Open の場合の例（ホスト名をユーザの環境に合わせて変更してください）
>
> ```
> $ rtls nextage:15005/
> ```

`rtls` の実行例（シミュレーション）

```
$ rtls localhost:15005/
robotuser-PC.host_cxt/               StateHolderServiceROSBridge.rtc
fk.rtc                              DataLoggerServiceROSBridge.rtc
longfloor(Robot)0.rtc               ImageSensorROSBridge_HandLeft.rtc
sh.rtc                              HiroNX(Robot)0.rtc
ImageSensorROSBridge_HeadRight.rtc  seq.rtc
HrpsysJointTrajectoryBridge0.rtc    log.rtc
HGcontroller0.rtc                   sc.rtc
ModelLoader                         el.rtc
CollisionDetector0.rtc              ImageSensorROSBridge_HandRight.rtc
ImageSensorROSBridge_HeadLeft.rtc   HrpsysSeqStateROSBridge0.rtc
SequencePlayerServiceROSBridge.rtc  ForwardKinematicsServiceROSBridge.rtc
ic.rtc                              rmfo.rtc
```

`rtls` で得られたリストの各RTコンポーネントの情報を得るには `rtcat` を利用します．

```
$ rtls %HOST_ROBOT%:15005/%CONPONENT_NAME%
```

`rtcat` の実行例（シミュレーション）

```
$ rtcat localhost:15005/fk.rtc
fk.rtc  Active
  Category       example
  Description    forward kinematics component
  Instance name  fk
  Type name      ForwardKinematics
  Vendor         AIST
  Version        315.14.0
  Parent         
  Type           Monolithic
 +Extra properties
+Execution Context 0
+DataInPort: q
+DataInPort: sensorRpy
+DataInPort: qRef
+DataInPort: basePosRef
+DataInPort: baseRpyRef
+CorbaPort: ForwardKinematicsService

$ rtcat localhost:15005/CollisionDetector0.rtc
CollisionDetector0.rtc  Active
  Category       example
  Description    collisoin detector component
  Instance name  CollisionDetector0
  Type name      CollisionDetector
  Vendor         AIST
  Version        315.14.0
  Parent         
  Type           Monolithic
 +Extra properties
+Execution Context 0
+DataInPort: qRef
+DataInPort: qCurrent
+DataInPort: servoStateIn
+DataOutPort: q
+DataOutPort: beepCommand
+CorbaPort: CollisionDetectorService

$ rtcat localhost:15005/ImageSensorROSBridge_HeadRight.rtc
ImageSensorROSBridge_HeadRight.rtc  Active
  Category       example
  Description    openrhp image - ros bridge
  Instance name  ImageSensorROSBridge_HeadRight
  Type name      ImageSensorROSBridge
  Vendor         Kei Okada
  Version        1.0
  Parent         
  Type           Monolithic
 +Extra properties
+Execution Context 0
+DataInPort: image
+DataInPort: timedImage
robotuser@robotuser-PC:~$
```


## ROS ステートのモニタリング

### ROS 環境の確認

#### ROS のバージョンの確認

```
$ rosversion -d
```

#### ROS に関する環境変数の確認

```
$ env | grep ROS
```

### rqt を用いたモニタリング

ROS の rqt は開発時におけるデータの可視化に大変役立つツールセットです．
ここでは HIRO / NEXTAGE OPEN において特に便利ないくつかのツールを紹介します．

- 参考: rqt/Plugins - ROS Wiki [http://wiki.ros.org/rqt/Plugins](http://wiki.ros.org/rqt/Plugins)

まず rqt を起動します．

```
$ rqt
```

次に "Plugins" をクリックして以下のプラグインを選択します．

- ROS Graph ( [rqt_graph](http://wiki.ros.org/rqt_graph) )
- Robot Monitor ( [rqt_robot_monitor](http://wiki.ros.org/rqt_robot_monitor) )
- Topic Introspection ( [rqt_topic](http://wiki.ros.org/rqt_topic) )

下図のようなウィンドウが表示されるはずです．

- 注意: この画像はロボットモニタの診断が無効なシミュレータでキャプチャされたため，失効ステータスが表示されています．

![rqt Window](http://wiki.ros.org/hironx_ros_bridge?action=AttachFile&do=get&target=snap_rqt_graph_monitor_topic_vertical.png)


## QNXでのトラブルシューティング

HIRO / NEXTAGE OPEN の QNX 内プロセスの状態を見るリモート監視ツールがないため，
その状態を見るためには QNX コンピュータ上のログファイルの内容を見る必要があります．

### ログの回収

QNX のログファイルは `/opt/jsk/var/log` にあります．

- Nameserver.log
  - OpenRTM または CORBA に関係したログの多くが記載
- Modelloader.log
  - OpenHRP3 に関するログ
- rtcd.log
  - hrpsys のRTコンポーネントに関連したログ

これらのログファイルは次のいずれかの方法で取得することができます．

##### 【推奨】 ログファイルの zip ボールを取得するスクリプトを実行する

- 注意: rtmros_hironx 1.1.25 以降で利用可能

```
# Simplest

$ rosrun hironx_ros_bridge qnx_fetch_log.sh nextage qnx_nxo_user
:
$ ls
opt_jsk_var_logs_20170602-020236.zip

# Fetch only files generated after certain date. "1" can be anything except "archive"

$ rosrun hironx_ros_bridge qnx_fetch_log.sh nextage qnx_nxo_user 1 2017-01-11
```

##### 【代替】 QNX にリモート接続する

QNX コンピュータに SSH 接続をして，
ディレクトリ `/opt/jsk/var/log` 下にあるログファイルにアクセスしてください．

### ログのチェック

#### ロボット胸部の4つ全てのランプが電源投入後も点滅し続けている場合

電源投入後正常起動した場合はロボット胸部の緑と白の LED ライトだけが点滅する状態となります．
この場合においてもまだロボット胸部の4つのランプが全て点滅し続けている場合は QNX のログを確認してください．

`/opt/jsk/var/log/rtcd.log` が次のようになってるかを確認してください．

```
Logger::Logger: streambuf address = 0x805fc70
hrpExecutionContext is registered
pdgains.file_name: /opt/jsk/etc/HIRONX/hrprtc/PDgains.sav
dof = 15
open_iob - shmif instance at 0x80b3f58
the number of gyros = 0
the number of accelerometers = 0
the number of force sensors = 0
period = 5[ms], priority = 49
```

`/opt/jsk/var/log/Nameserver.log` が次のようになってるかを確認してください．

```
Sat Jan 24 10:55:33 2015:

Starting omniNames for the first time.
Wrote initial log file.
Read log file successfully.
Root context is IOR:010000002b00000049444c3a6f6d672e6f72672f436f734e616d696e672f4e616d696e67436f6e74
6578744578743a312e300000010000000000000070000000010102000d0000003139322e3136382e312e313600009d3a0b00
00004e616d6553657276696365000300000000000000080000000100000000545441010000001c0000000100000001000100
0100000001000105090101000100000009010100035454410800000095fbc2540100001b
Checkpointing Phase 1: Prepare.
Checkpointing Phase 2: Commit.
Checkpointing completed.

Sat Jan 24 11:10:33 2015:

Checkpointing Phase 1: Prepare.
Checkpointing Phase 2: Commit.
Checkpointing completed.
```

`/opt/jsk/var/log/ModelLoader.log` が次のようになっているかを確認してください．

```
ready
loading /opt/jsk/etc/HIRONX/model/main.wrl
Humanoid node
Joint nodeWAIST
  Segment node WAIST_Link
  Joint nodeCHEST_JOINT0
    Segment node CHEST_JOINT0_Link
    Joint nodeHEAD_JOINT0
      Segment node HEAD_JOINT0_Link
      Joint nodeHEAD_JOINT1
        Segment node HEAD_JOINT1_Link
        VisionSensorCAMERA_HEAD_R
        VisionSensorCAMERA_HEAD_L
    Joint nodeRARM_JOINT0
      Segment node RARM_JOINT0_Link
      Joint nodeRARM_JOINT1
        Segment node RARM_JOINT1_Link
        Joint nodeRARM_JOINT2
          Segment node RARM_JOINT2_Link
          Joint nodeRARM_JOINT3
            Segment node RARM_JOINT3_Link
            Joint nodeRARM_JOINT4
              Segment node RARM_JOINT4_Link
              Joint nodeRARM_JOINT5
                Segment node RARM_JOINT5_Link
    Joint nodeLARM_JOINT0
      Segment node LARM_JOINT0_Link
      Joint nodeLARM_JOINT1
        Segment node LARM_JOINT1_Link
        Joint nodeLARM_JOINT2
          Segment node LARM_JOINT2_Link
          Joint nodeLARM_JOINT3
            Segment node LARM_JOINT3_Link
            Joint nodeLARM_JOINT4
              Segment node LARM_JOINT4_Link
              Joint nodeLARM_JOINT5
                Segment node LARM_JOINT5_Link
The model was successfully loaded !
```

すべてのログが上記のような表示でしたらログに関しては正常で，
他に何か複雑なことが起こっている可能性があります．
サポートにお問い合わせください．


<!-- EOF -->

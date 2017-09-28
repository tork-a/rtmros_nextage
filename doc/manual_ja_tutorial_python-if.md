
# チュートリアル: Python インタフェース

## RTM Python インタフェース

### インタラクティブモードでの操作

#### rtm_ros_bridge の起動

rtm_ros_bridge を起動します．シミュレーションの場合は不要です．

```
$ roslaunch hironx_ros_bridge hironx_ros_bridge_real.launch nameserver:=%HOSTNAME%    (HIRO)

$ roslaunch nextage_ros_bridge nextage_ros_bridge_real.launch nameserver:=%HOSTNAME%  (NEXTAGE OPEN)
```

#### iPython インタフェースの起動

HIRO での iPython インタフェースの起動
ホスト名は各ロボットの設定に応じて変更してください．

```
ipython -i `rospack find hironx_ros_bridge`/scripts/hironx.py     (Simulation)

ipython -i `rospack find hironx_ros_bridge`/scripts/hironx.py  -- --host hiro014    (Real robot example)
```

NEXTAGE OPEN での iPython インタフェースの起動

```
ipython -i `rospack find nextage_ros_bridge`/script/nextage.py     (Simulation)

ipython -i `rospack find nextage_ros_bridge`/script/nextage.py  -- --host nextage    (Real Robot Example)
```

他のオプションも必要であれば指定することができます．

```
$ ipython -i `rospack find hironx_ros_bridge`/scripts/hironx.py -- --modelfile /opt/jsk/etc/HIRONX/model/main.wrl --host hiro014 --port 15005

$ ipython -i `rospack find nextage_ros_bridge`/script/nextage.py -- --modelfile /opt/jsk/etc/HIRONX/model/main.wrl --host nextage101 --port 15005
```

リモート接続する場合は `--robot` 引数を使用してロボットのインスタンス名を指定する必要があります．
例えば次のように指定して実行します．

```
$ ipython -i `rospack find hironx_ros_bridge`/scripts/hironx.py -- --host nxo_simulation_host --robot "HiroNX(Robot)0"
```
iPython インタフェース初期化時に行っていることは次のようになっています．

- RTCManager と RobotHardware の検出
  - RTCManager: RT コンポーネントを起動する OpenRTM のクラス
  - RobotHardware: ロボットハードウェアとのインタフェースである hrpsys に定義されているRTコンポーネント
- hrpsys コントローラを動作させるのに必要な RT コンポーネントへの接続とアクティベーション
- ロガーの開始
- Joint Groups への割当て（HIRO/NEXTAGE OPEN 固有）

#### よく使う RTM Python インタフェースコマンド

ロボット実機操作において特有なコマンドを紹介します．

- 注意-1: コマンドを実行するとロボットが動きます．
- 注意-2: 緊急停止スイッチをいつでも押せる状態にしておいてください．

ロボット実機のキャリブレーションを行うコマンドです．

```
ipython>>> robot.checkEncoders()
```

- ロボット実機のキャリブレーションがなされていない場合にのみ実行されます．
  - ほとんどの場合においてロボット胸部の緑のロボットステートインジケータライトが点滅します．

ロボットを初期姿勢するコマンドです．

```
ipython>>> robot.goInitial()
```

作業終了姿勢に移行するコマンドです．終了姿勢に移行した後にサーボが切れます．

```
ipython>>> robot.goOffPose()
```

- システムの再起動・シャットダウン時の前にはこのコマンドを実行してください．

サーボを入れるコマンドです．

```
ipython>>> robot.servoOn()
```

- 次の動作を行ってサーボが切れた状態のときには手動でサーボを入れる必要があります．
  - `goOffPose` を実行したとき
  - リレースイッチを押したとき（この場合はまず `servoOff` を行う必要があります）
  - ロボットに異常動作を与えて緊急停止したとき
- 現状の物理的関節角度にて角度指示値を設定します．
  - `goActual()` を内部的に呼び出しています．

サーボを切るコマンドです．

```
ipython>>> robot.servoOff()
```

- 明示的にサーボを切るのではなくリレースイッチを押した場合にはこのコマンドを実行してください．

#### RTM Python インタフェースの利用

iPython の初期化時にロボットのクライアントインタフェースクラスである HIRONX/NextageClient が iPython ターミナル上で `robot` ととしてインスタンス化されます．

`robot` において何ができるのかを見てみます．

```
In : robot.
```

上記のように iPython ターミナル上で入力したのに tab キーを押すと選択可能なものが表示されます．

```
robot.Groups                            robot.getCurrentRPY                     robot.rh_svc
robot.HandClose                         robot.getCurrentRotation                robot.saveLog
robot.HandGroups                        robot.getJointAngles                    robot.sc
robot.HandOpen                          robot.getRTCInstanceList                robot.sc_svc
robot.InitialPose                       robot.getRTCList                        robot.sensors
robot.OffPose                           robot.getReferencePose                  robot.seq
robot.RtcList                           robot.getReferencePosition              robot.seq_svc
robot.abc                               robot.getReferenceRPY                   robot.servoOff
robot.activateComps                     robot.getReferenceRotation              robot.servoOn
robot.afs                               robot.getSensors                        robot.setHandEffort
robot.checkEncoders                     robot.goActual                          robot.setHandJointAngles
robot.clearLog                          robot.goInitial                         robot.setHandWidth
robot.co                                robot.goOffPose                         robot.setJointAngle
robot.co_svc                            robot.hand_width2angles                 robot.setJointAngles
robot.configurator_name                 robot.hgc                               robot.setJointAnglesOfGroup
robot.connectComps                      robot.ic                                robot.setSelfGroups
robot.connectLoggerPort                 robot.init                              robot.setTargetPose
robot.createComp                        robot.isCalibDone                       robot.setupLogger
robot.createComps                       robot.isServoOn                         robot.sh
robot.el                                robot.kf                                robot.sh_svc
robot.el_svc                            robot.lengthDigitalInput                robot.simulation_mode
robot.ep_svc                            robot.lengthDigitalOutput               robot.st
robot.findModelLoader                   robot.liftRobotUp                       robot.stOff
robot.fk                                robot.loadPattern                       robot.tf
robot.fk_svc                            robot.log                               robot.vs
robot.flat2Groups                       robot.log_svc                           robot.waitForModelLoader
robot.getActualState                    robot.moveHand                          robot.waitForRTCManagerAndRoboHardware
robot.getBodyInfo                       robot.ms                                robot.waitInterpolation
robot.getCurrentPose                    robot.readDigitalInput                  robot.waitInterpolationOfGroup
robot.getCurrentPosition                robot.rh                                robot.writeDigitalOutput
```

`Groups` のリストとそのメンバを見てみます．

```
In : robot.Groups
Out:
[['torso', ['CHEST_JOINT0']],
 ['head', ['HEAD_JOINT0', 'HEAD_JOINT1']],
 ['rarm',
  ['RARM_JOINT0',
   'RARM_JOINT1',
   'RARM_JOINT2',
   'RARM_JOINT3',
   'RARM_JOINT4',
   'RARM_JOINT5']],
 ['larm',
  ['LARM_JOINT0',
   'LARM_JOINT1',
   'LARM_JOINT2',
   'LARM_JOINT3',
   'LARM_JOINT4',
   'LARM_JOINT5']]]
```

スクリプトインタラプタとしての iPython の利点はそこから API の情報を得られることにあります．

例えば，ロボットの現在の姿勢を知りたいがそのコマンドを知らないような場合でも，
まず少し推測してからタブ補完を利用することで次のように情報を得ることができます．

```
In : robot.getCurrent
robot.getCurrentPose      robot.getCurrentPosition  robot.getCurrentRPY       robot.getCurrentRotation
```

`getCurren` に対して4つの選択肢があり，
この内 `getCurrentPose` が意図するもののコマンドらしいことが分ります．
さらにそのメソッドの引数を知る必要がある場合にはコマンドの最後に `?` を入力します．

```
In : robot.getCurrentPose?
Type:       instancemethod
Base Class: <type 'instancemethod'>
String Form:<bound method HIRONX.getCurrentPose of <__main__.HIRONX instance at 0x1f39758>>
Namespace:  Interactive
File:       /opt/ros/hydro/lib/python2.7/dist-packages/hrpsys_config.py
Definition: robot.getCurrentPose(self, lname)
Docstring:  <no docstring>
```

ここで `getCurrentPose` は `lname`（link name の略）を受け取ることが分ります．
よって次のように実行します．

```
In: robot.getCurrentPose('RARM_JOINT0')
Out:
[0.912826202314136,
 -0.4083482880688395,
 0.0,
 0.0,
 0.39443415756662026,
 0.8817224037285941,
 -0.25881904510252074,
 -0.145,
 0.1056883139872261,
 0.2362568060275051,
 0.9659258262890683,
 0.370296,
 0.0,
 0.0,
 0.0,
 1.0]
```

この `getCurrentPose` は指示したリンクの回転行列と位置の値を1次元のリストとして戻します．

位置だけを知りたい場合には他の次の方法で取得することもできます．

```
In: robot.getCurrent
robot.getCurrentPose      robot.getCurrentPosition  robot.getCurrentRPY       robot.getCurrentRotation

In : robot.getCurrentPosition('RARM_JOINT0')
Out: [0.0, -0.145, 0.370296]
```

hrpsys においては位置ベクトルは次のように対応した3つの要素 [x, y, z] で表されます．

- x: 前
- y: 左
- z: 上

---

次に腕を動かしてみます．
まず初期姿勢まで動かします．

```
In : robot.goInitial()
```

目標姿勢を設定するにはどうすれば良いかを調べます．

```
In : robot.setTargetPose?
Type:       instancemethod
Base Class: <type 'instancemethod'>
String Form:<bound method HIRONX.setTargetPose of <__main__.HIRONX instance at 0x333b758>>
Namespace:  Interactive
File:       /opt/ros/hydro/lib/python2.7/dist-packages/hrpsys/hrpsys_config.py
Definition: robot.setTargetPose(self, gname, pos, rpy, tm)
Docstring:  <no docstring>
```

`gname` は `joint group` の名前です．
`pos` と `rpy` はリスト形式です．

目標姿勢を指定する前に現在のロボットの姿勢を変数に格納します．

```
In : pos = robot.getCurrentPosition('RARM_JOINT5')
In : rpy = robot.getReferenceRPY('RARM_JOINT5')
In : tm = 3
```

ロボットの姿勢はおそらく下図のようになっていることと思います．

![HIRO Current Pose](http://wiki.ros.org/hironx_ros_bridge?action=AttachFile&do=get&target=hiro_before_move_rarm.png)

それでは目標位置を現在の姿勢から少し変えて指示して，そこへの動作を実行させてみましょう．

```
In : pos[2] = 0.1

In : robot.setTargetPose('rarm', pos, rpy, tm)
Out: True
```

次の図のように右腕の手先が指定した位置へと移動したことと思います．

- 注意: 下図は MoveIt! が実行されているときにキャプチャしたもので，MoveIt! 由来の黄緑色で表示されている開始姿勢の腕はここのチュートリアルでは関係しませんので無視してください．

![HIRO setTargetPose](http://wiki.ros.org/hironx_ros_bridge?action=AttachFile&do=get&target=hiro_after_move_rarm.png)

ロボットでの作業が終了したら終了姿勢にしてください．

```
In : robot.goOffPose()
```

![HIRO goOffPose](http://wiki.ros.org/hironx_ros_bridge?action=AttachFile&do=get&target=hiro_powerOff.png)

#### hrpsys-based API のソースとドキュメント

hrpsys-based API は次のリンク先にソースとドキュメントがあります．

- 多くのコマンドは hrpsys_config.HrpsysConfigurator のペアレントクラスに定義されています．
  - [http://fkanehiro.github.io/hrpsys-base/df/d98/classpython_1_1hrpsys__config_1_1HrpsysConfigurator.html](http://fkanehiro.github.io/hrpsys-base/df/d98/classpython_1_1hrpsys__config_1_1HrpsysConfigurator.html)
- HIRO: hironx_ros_bridge/scripts/
  - [https://github.com/start-jsk/rtmros_hironx/blob/hydro-devel/hironx_ros_bridge/scripts/hironx.py](https://github.com/start-jsk/rtmros_hironx/blob/hydro-devel/hironx_ros_bridge/scripts/hironx.py)
- NEXTAGE OPEN: nextage_ros_bridge/scripts/
  - [https://github.com/tork-a/rtmros_nextage/blob/hydro-devel/nextage_ros_bridge/src/nextage_ros_bridge/nextage_client.py](https://github.com/tork-a/rtmros_nextage/blob/hydro-devel/nextage_ros_bridge/src/nextage_ros_bridge/nextage_client.py)


### RTM Python インタフェースプログラミング

Python を用いた HIRO / NEXTAGE OPEN のプログラミングは1つの統合されたインタフェースによって行うことができます．
インタフェースの名称はロボットによりそれぞれ異なります．

- HIRONX : HIRO / HIRONX ユーザ
- NextageClient : NEXTAGE OPEN ユーザ

本項では "HIRONX" を使用しますが NEXTAGE OPEN ユーザは NextageClient インタフェースを用いて同様のことを行うことができます．

#### サンプルコード - Acceptance Test (RTM)

Acceptance Test (RTM) のコードを参考に RTM Python インタフェースプログタミング
の方法を見たあとにコードを実行します．

##### acceptancetest_rtm.py

まずこのサンプルは下図のような2段階の依存関係を持っています．

![AcceptanceTest_Hiro Dependency](https://docs.google.com/drawings/d/1wNVjZ7LLxJQMpVFkPJeNgMLA99tyJCkQukFo7FnYwXI/pub?w=779&h=282)

このサンプルコードの動作は `AcceptanceTest_Hiro` クラスに記述されていて
`HIRONX` クラスに接続する `AcceptancetestRTM` クラスを利用しています．

`AcceptancetestRTM` について見てみます．コードの全体は次のようになっています．

- [https://raw.githubusercontent.com/start-jsk/rtmros_hironx/a7a43e5baf4dcd48e34b94f9781defadfbca03d0/hironx_ros_bridge/src/hironx_ros_bridge/testutil/acceptancetest_rtm.py](https://raw.githubusercontent.com/start-jsk/rtmros_hironx/a7a43e5baf4dcd48e34b94f9781defadfbca03d0/hironx_ros_bridge/src/hironx_ros_bridge/testutil/acceptancetest_rtm.py)

```python
1 # -*- coding: utf-8 -*-
2
3 # Software License Agreement (BSD License)
4 #
5 # Copyright (c) 2014, TORK (Tokyo Opensource Robotics Kyokai Association)
6 # All rights reserved.
7 #
8 # Redistribution and use in source and binary forms, with or without
9 # modification, are permitted provided that the following conditions
10 # are met:
11 #
12 #  * Redistributions of source code must retain the above copyright
13 #    notice, this list of conditions and the following disclaimer.
14 #  * Redistributions in binary form must reproduce the above
15 #    copyright notice, this list of conditions and the following
16 #    disclaimer in the documentation and/or other materials provided
17 #    with the distribution.
18 #  * Neither the name of TORK. nor the
19 #    names of its contributors may be used to endorse or promote products
20 #    derived from this software without specific prior written permission.
21 #
22 # THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
23 # "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
24 # LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
25 # FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
26 # COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
27 # INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
28 # BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
29 # LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
30 # CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
31 # LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
32 # ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
33 # POSSIBILITY OF SUCH DAMAGE.
34
35 import time
36
37 from hironx_ros_bridge.constant import Constant
38 from hironx_ros_bridge.testutil.abst_acceptancetest import AbstAcceptanceTest
39
40
41 class AcceptanceTestRTM(AbstAcceptanceTest):
42
43     def __init__(self, robot_client):
44         '''
45         @type robot_client: hironx_ros_bridge.hironx_client.HIRONX
46         '''
47         self._robotclient = robot_client
48
49     def go_initpos(self):
50         self._robotclient.goInitial()
51
52     def set_joint_angles(self, joint_group, joint_angles, msg_tasktitle=None,
53                          task_duration=7.0, do_wait=True):
54         '''
55         @see: AbstAcceptanceTest.set_joint_angles
56         '''
57         print("== RTM; {} ==".format(msg_tasktitle))
58         self._robotclient.setJointAnglesOfGroup(
59                          joint_group, joint_angles, task_duration, do_wait)
60
61     def set_pose(self, joint_group, pose, rpy, msg_tasktitle,
62                       task_duration=7.0, do_wait=True, ref_frame_name=None):
63
64         print("== RTM; {} ==".format(msg_tasktitle))
65         self._robotclient.setTargetPose(joint_group, pose, rpy, task_duration,
66                                         ref_frame_name)
67         if do_wait:
68             self._robotclient.waitInterpolationOfGroup(joint_group)
69
70     def set_pose_relative(
71                         self, joint_group, dx=0, dy=0, dz=0, dr=0, dp=0, dw=0,
72                         msg_tasktitle=None, task_duration=7.0, do_wait=True):
73         if joint_group == Constant.GRNAME_LEFT_ARM:
74             eef = 'LARM_JOINT5'
75         elif joint_group == Constant.GRNAME_RIGHT_ARM:
76             eef = 'RARM_JOINT5'
77
78         print("== RTM; {} ==".format(msg_tasktitle))
79         self._robotclient.setTargetPoseRelative(
80                                     joint_group, eef, dx, dy, dz, dr, dp, dw,
81                                     task_duration, do_wait)
82
83     def _run_tests_hrpsys(self):
84         '''
85         @deprecated: This method remains as a reference. This used to function
86                      when being called directly from ipython commandline and
87                      now replaced by optimized codes.
88         '''
89         _TIME_SETTARGETP_L = 3
90         _TIME_SETTARGETP_R = 2
91         _TIME_BW_TESTS = 5
92
93         self.robot.goInitial()
94
95         # === TASK-1 ===
96         # L arm setTargetPose
97         _POS_L_INIT = self.robot.getCurrentPosition('LARM_JOINT5')
98         _POS_L_INIT[2] += 0.8
99         _RPY_L_INIT = self.robot.getCurrentRPY('LARM_JOINT5')
100         self.robot.setTargetPose('larm', _POS_L_INIT, _RPY_L_INIT, _TIME_SETTARGETP_L)
101         self.robot.waitInterpolationOfGroup('larm')
102
103         # R arm setTargetPose
104         _POS_R_INIT = self.robot.getCurrentPosition('RARM_JOINT5')
105         _POS_R_INIT[2] -= 0.07
106         _RPY_R_INIT = self.robot.getCurrentRPY('RARM_JOINT5')
107         self.robot.setTargetPose('rarm', _POS_R_INIT, _RPY_R_INIT, _TIME_SETTARGETP_R)
108         self.robot.waitInterpolationOfGroup('rarm')
109         time.sleep(_TIME_BW_TESTS)
110
111         # === TASK-2 ===
112         self.robot.goInitial()
113         # Both arm setTargetPose
114         _Z_SETTARGETP_L = 0.08
115         _Z_SETTARGETP_R = 0.08
116         self.robot.setTargetPoseRelative('larm', 'LARM_JOINT5',
117                                          dz=_Z_SETTARGETP_L,
118                                          tm=_TIME_SETTARGETP_L, wait=False)
119         self.robot.setTargetPoseRelative('rarm', 'RARM_JOINT5',
120                                          dz=_Z_SETTARGETP_R,
121                                          tm=_TIME_SETTARGETP_R, wait=False)
122
123         # === TASK-3 ===
124         # Head toward down
125         _TIME_HEAD = 5
126         self.robot.setTargetPoseRelative('head', 'HEAD_JOINT0', dp=0.1, tm=_TIME_HEAD)
127         self.robot.waitInterpolationOfGroup('head')
128         # Head toward up
129         self.robot.setTargetPoseRelative('head', 'HEAD_JOINT0', dp=-0.2, tm=_TIME_HEAD)
130         self.robot.waitInterpolationOfGroup('head')
131         # See left by position
132         self.robot.setJointAnglesOfGroup('head', [50, 10], 2, wait=True)
133         # See right by position
134         self.robot.setJointAnglesOfGroup('head', [-50, -10], 2, wait=True)
135         # Set back face to the starting pose w/o wait.
136         self.robot.setJointAnglesOfGroup( 'head', [0, 0], 2, wait=False)
137
138         # === TASK-4 ===
139         # 0.1mm increment is not working for some reason.
140         self.robot.goInitial()
141         # Move by iterating 0.1mm at cartesian space
142         _TIME_CARTESIAN = 0.1
143         _INCREMENT_MIN = 0.0001
144         for i in range(300):
145             self.robot.setTargetPoseRelative('larm', 'LARM_JOINT5',
146                                              dy=_INCREMENT_MIN,
147                                              tm=_TIME_CARTESIAN)
148             self.robot.setTargetPoseRelative('rarm', 'RARM_JOINT5',
149                                              dy=_INCREMENT_MIN,
150                                              tm=_TIME_CARTESIAN)
151             print('{}th move'.format(i))
152
153         self.robot.goInitial()
154         # === TASK-5 ===
155         # Turn torso
156         _TORSO_ANGLE = 120
157         _TIME_TORSO_R = 7
158         self.robot.setJointAnglesOfGroup('torso', [_TORSO_ANGLE], _TIME_TORSO_R, wait=True)
159         self.robot.waitInterpolationOfGroup('torso')
160         self.robot.setJointAnglesOfGroup('torso', [-_TORSO_ANGLE], 10, wait=True)
161  
162         self.robot.goInitial()
163
164         # === TASK-6.1 ===
165         # Overwrite previous command, for torso using setJointAnglesOfGroup
166         self.robot.setJointAnglesOfGroup('torso', [_TORSO_ANGLE], _TIME_TORSO_R,
167                                          wait=False)
168         time.sleep(1)
169         self.robot.setJointAnglesOfGroup('torso', [-_TORSO_ANGLE], 10, wait=True)
170
171         self.robot.goInitial(5)
172
173         # === TASK-6.2 ===
174         # Overwrite previous command, for arms using setTargetPose
175         _X_EEF_OVERWRITE = 0.05
176         _Z_EEF_OVERWRITE = 0.1
177         _TIME_EEF_OVERWRITE = 7
178         _POS_L_INIT[0] += _X_EEF_OVERWRITE
179         _POS_L_INIT[2] += _Z_EEF_OVERWRITE
180         self.robot.setTargetPose('larm', _POS_L_INIT, _RPY_L_INIT, _TIME_EEF_OVERWRITE)
181         self.robot.waitInterpolationOfGroup('larm')
182         # Trying to raise rarm to the same level of larm.
183         _POS_R_INIT[0] += _X_EEF_OVERWRITE
184         _POS_R_INIT[2] += _Z_EEF_OVERWRITE
185         self.robot.setTargetPose('rarm', _POS_R_INIT, _RPY_R_INIT, _TIME_EEF_OVERWRITE)
186         self.robot.waitInterpolationOfGroup('rarm')
187         time.sleep(3)
188         # Stop rarm
189         self.robot.clearOfGroup('rarm')  # Movement should stop here.
190
191         # === TASK-7.1 ===
192         # Cover wide workspace.
193         _TIME_COVER_WORKSPACE = 3
194         # Close to the max width the robot can spread arms with the hand kept
195         # at table level.
196         _POS_L_X_NEAR_Y_FAR = [0.32552812002303166, 0.47428609880442024, 1.0376656470275407]
197         _RPY_L_X_NEAR_Y_FAR = (-3.07491977663752, -1.5690249316560323, 3.074732073335767)
198         _POS_R_X_NEAR_Y_FAR = [0.32556456455769633, -0.47239119592815987, 1.0476131608682244]
199         _RPY_R_X_NEAR_Y_FAR = (3.072515432213872, -1.5690200270375372, -3.072326882451363)
200
201         # Close to the farthest distance the robot can reach, with the hand kept
202         # at table level.
203         _POS_L_X_FAR_Y_FAR = [0.47548142379781055, 0.17430276793604782, 1.0376878025614884]
204         _RPY_L_X_FAR_Y_FAR = (-3.075954857224205, -1.5690261926181046, 3.0757659493049574)
205         _POS_R_X_FAR_Y_FAR = [0.4755337947019357, -0.17242322190721648, 1.0476395479774052]
206         _RPY_R_X_FAR_Y_FAR = (3.0715850722714944, -1.5690204449882248, -3.071395243174742)
207         self.robot.setTargetPose('larm', _POS_L_X_NEAR_Y_FAR, _RPY_L_X_NEAR_Y_FAR, _TIME_COVER_WORKSPACE)
208         self.robot.setTargetPose('rarm', _POS_R_X_NEAR_Y_FAR, _RPY_R_X_NEAR_Y_FAR, _TIME_COVER_WORKSPACE)
209         self.robot.waitInterpolationOfGroup('larm')
210         self.robot.waitInterpolationOfGroup('rarm')
211         time.sleep(3)
212         self.robot.setTargetPose('larm', _POS_L_X_FAR_Y_FAR, _RPY_L_X_FAR_Y_FAR, _TIME_COVER_WORKSPACE)
213         self.robot.setTargetPose('rarm', _POS_R_X_FAR_Y_FAR, _RPY_R_X_FAR_Y_FAR, _TIME_COVER_WORKSPACE)
214         self.robot.waitInterpolationOfGroup('larm')
215         self.robot.waitInterpolationOfGroup('rarm')
216
217         self.robot.goInitial()
```

タスクの内容を少し見てみます．

```python
37 from hironx_ros_bridge.constant import Constant
38 from hironx_ros_bridge.testutil.abst_acceptancetest import AbstAcceptanceTest
```

RTM API の HIRONX もしくは NextageClient クラスがインポートされていません．しかし，次の部分を見ると...

```python
43 def __init__(self, robot_client):
44     '''
45     @type robot_client: hironx_ros_bridge.hironx_client.HIRONX
46     '''
47     self._robotclient = robot_client
```

コンストラクタが HIRONX を受け取り，メンバー `self._robotclient` に格納していることが分ります．
以後は `self._robotclient` が HIRONX クラスインスタンスを参照することとなります．

コードの残りの部分は基本的な操作を行っているだけです．

例えば次のような部分です．
簡潔に全ての関節が初期姿勢になるように記述されています．

```python
49 def go_initpos(self):
50     self._robotclient.goInitial()
```

> 後々，自分のコードを開発してゆくうちに，
各クラスとメソッドの API ドキュメントを見てオプションを知りたくなるかもしれません．
このチュートリアルでは詳しく説明しませんので，ぜひご自身で探索してみてください．
例えば `goInitial` メソッドには次のオプションがあります．
```python
def hironx_ros_bridge.hironx_client.HIRONX.goInitial(self, tm = 7, wait = True, init_pose_type = 0)
```
- [http://docs.ros.org/hydro/api/hironx_ros_bridge/html/classhironx__ros__bridge_1_1hironx__client_1_1HIRONX.html#a295b0b4950cb580273b224dc659c8a23](http://docs.ros.org/hydro/api/hironx_ros_bridge/html/classhironx__ros__bridge_1_1hironx__client_1_1HIRONX.html#a295b0b4950cb580273b224dc659c8a23)

更に例を挙げると次の部分です．

```python
52     def set_joint_angles(self, joint_group, joint_angles, msg_tasktitle=None,
53                          task_duration=7.0, do_wait=True):
54         '''
55         @see: AbstAcceptanceTest.set_joint_angles
56         '''
57         print("== RTM; {} ==".format(msg_tasktitle))
58         self._robotclient.setJointAnglesOfGroup(
59                          joint_group, joint_angles, task_duration, do_wait)
```

こちらも簡潔に HIRONX のメソッドを呼び出すだけで，
多くのことがすでに HIRONX 内で処理されているので，
このような簡単な方法でコードを書くことができます．

##### サンプルコードの実行

新しい iPython ターミナルを開きます．
少なくとも3つのターミナルが開いた状態になっています．
次のように iPython ターミナルを起動します．
シミュレーション環境の場合はそれに合わせて起動してください．

```
$ ipython -i `rospack find hironx_ros_bridge`/scripts/acceptancetest_hironx.py -- --host %HOSTNAME%
```

次のコマンドで RTM インタフェースを通じてタスクが実行されます．

- 注意: ロボットが動きます．

```
IN [1]: acceptance.run_tests_rtm()
```

次のように記述することでタスクの逐次実行もできます．

```
IN [1]: acceptance.run_tests_rtm(do_wait_input=True)
```

iPython インタフェースを終了するときは `Ctrl-d` にてエスケーブします．


#### サンプルコード - 円を描く

ロボットのエンドエフェクタで円を描くサンプルコードを下記に示します．

- 注意: 本コードを改変する場合はまずシミュレーションでその動作を確認してから実機で動作させてください．

[NEXTAGE OPEN] Robots hands drawing circles: [https://www.youtube.com/watch?v=OVae1xa5Rak](https://www.youtube.com/watch?v=OVae1xa5Rak)

変数 `robot` は何らかの方法でユーザの HIRONX/NextageClient クラスのインスタンスに置き換える必要があります．

```python
def circle_eef(radius=0.01, eef='larm', step_degree=5, ccw=True, duration=0.1):
    '''
    Moves the designated eef point-by-point so that the trajectory as a whole draws a circle.

    Currently this only works on the Y-Z plane of *ARM_JOINT5 joint.
    And it's the most intuitive when eef maintains a "goInitial" pose where circle gets drawn on robot's X-Y plane
    (see the wiki for the robot's coordinate if you're confused http://wiki.ros.org/rtmros_nextage/Tutorials/Programming#HiroNXO_3D_model_coordination).

    Points on the circular trajectory is based on a standard equation https://en.wikipedia.org/wiki/Circle#Equations

    @param radius: (Unit: meter) Radius of the circle to be drawn.
    @param step_degree: Angle in degree each iteration increments.
    @param ccw: counter clock-wise.
    @param duration: Time for each iteration to be completed.
    '''
    goal_deg = GOAL_DEGREE = 360
    start_deg = 0
    if eef == 'larm':
        joint_eef = 'LARM_JOINT5'
    elif eef == 'rarm':
        joint_eef = 'RARM_JOINT5'
    eef_pos = robot.getCurrentPosition(joint_eef)
    eef_rpy = robot.getCurrentRPY(joint_eef)
    print('eef_pos={}'.format(eef_pos))
    X0 = eef_pos[0]
    Y0 = eef_pos[1]
    ORIGIN_x = X0
    ORIGIN_y = Y0 - radius
    print('ORIGIN_x={} ORIGIN_y={}'.format(ORIGIN_x, ORIGIN_y))
    i = 0
    for theta in range(start_deg, goal_deg, step_degree):
        if not ccw:
            theta = -theta
        x = ORIGIN_x + radius*math.sin(math.radians(theta))  # x-axis in robot's eef space is y in x-y graph
        y = ORIGIN_y + radius*math.cos(math.radians(theta))
        eef_pos[0] = x
        eef_pos[1] = y
        print('#{}th theta={} x={} y={} X0={} Y0={}'.format(i, theta, x, y, X0, Y0))
        robot.setTargetPose(eef, eef_pos, eef_rpy, duration)
        robot.waitInterpolation()
        i += 1
```

#### 使用場面に応じたプログラミング

デフォルトではいくつかの HIRONX / NextageClient クラスのコマンドは
動作終了を待ってから次の動作を開始します．
またいくつかのコマンドはそのように待たないものもあります．
それは各メソッドが受け取る引数から明確になる実装に依存します．
`wait` 引数を持つメソッドは待機するかどうかを指定できます．
それ以外のものは特に API ドキュメントに記述の無いかぎり待機の可否を指定できません．

次は "待機" の特性を持つメソッドの例です．

```python
70     def set_pose_relative(
71                         self, joint_group, dx=0, dy=0, dz=0, dr=0, dp=0, dw=0,
72                         msg_tasktitle=None, task_duration=7.0, do_wait=True):
73         if joint_group == Constant.GRNAME_LEFT_ARM:
74             eef = 'LARM_JOINT5'
75         elif joint_group == Constant.GRNAME_RIGHT_ARM:
76             eef = 'RARM_JOINT5'
77
78         print("== RTM; {} ==".format(msg_tasktitle))
79         self._robotclient.setTargetPoseRelative(
80                                     joint_group, eef, dx, dy, dz, dr, dp, dw,
81                                     task_duration, do_wait)
```

一般にはデフォルトでは `wait=True` とした方が安全となるでしょう．

次の `setTargetPose` は待機・中断の信号は受け取りません．
この場合においては `HrpsysConfigurator` の `waitInterpolationOfGroup()` を呼び出して対処することができます．
`HrpSysConfigurator` は `HIRONX` の親クラスです．

```python
61     def set_pose(self, joint_group, pose, rpy, msg_tasktitle,
62                       task_duration=7.0, do_wait=True, ref_frame_name=None):
63
64         print("== RTM; {} ==".format(msg_tasktitle))
65         self._robotclient.setTargetPose(joint_group, pose, rpy, task_duration,
66                                         ref_frame_name)
67         if do_wait:
68             self._robotclient.waitInterpolationOfGroup(joint_group)
```


#### 連続した軌道座標指示による動作

次のようにすることで連続した軌道座標指示による動作を行うことができます．

```python
hcf.playPatternOfGroup('LARM',
                        [[0.010,0.0,-1.745,-0.265,0.164,0.06],
                         [0.010,-0.2,-2.045,-0.265,0.164,0.06],
                         [0.010,-0.4,-2.245,-0.265,0.164,0.06],
                         [0.010,-0.6,-2.445,-0.265,0.164,0.06],
                         [0.010,-0.8,-2.645,-0.265,0.164,0.06]],
                        [1,1,1,1,1])
```

`hcf` は `robot` など Python インタフェースを起動した状況に応じて変更してください．

hrpsys 315.6.0 以降では `setJointAnglesSequenceOfGroup` も利用することができます．

- [https://github.com/fkanehiro/hrpsys-base/blob/3eab14b836dea11386dbdb7d0ab90a0ed9521237/python/hrpsys_config.py#L1018](https://github.com/fkanehiro/hrpsys-base/blob/3eab14b836dea11386dbdb7d0ab90a0ed9521237/python/hrpsys_config.py#L1018)

#### 相対姿勢指示による動作

エンドエフェクタのフレームや関節を現在の姿勢からの相対姿勢を指示して動作させるのも
HIRONX インタフェースの `setTargetPoseRelative` を用いることで簡単に行うことができます．

下記の `[1]` では `torso` を 3[s] かけて 0.1[rad] 回転させます．
`[2]` では右腕のエンドエフェクタフレームを 3[s] かけて前方に 0.1[m] 移動させます．

```python
In [1]: robot.setTargetPoseRelative('torso', 'CHEST_JOINT0', dw=0.1, tm=3)

In [2]: robot.setTargetPoseRelative('rarm', 'RARM_JOINT5', dx=0.1, tm=3)
```

#### ユーザ Python コードの作成

ここまでの RTM Python インタフェースのチュートリアルでは `hironx.py` や `nextage.py` といったスクリプトを実行していました．

クライアントインタフェースクラスのメソッドを利用して
アプリケーションモジュールを作成する方法を考えてみます．
そのモジュールを `your_nxo_sample.py` と名付け，
それをシミュレーションなり実機なりで動作させるものとします．

1. まず1つの方法として `nextage.py` を複製して `NextageClient` クラスがインスタンス化される行を書き換えます．
他の `NextageClient` クラスがアクセスする部分も消したり書き換えたりします．
2. 上記の方法はあまりスマートで簡潔ではありません．
それは大部分のコードを複製するというのはソフトウェア開発においては悪い実践法であるからです．
例えば [hrpsys_tools/hrpsys_tools_config.py](https://github.com/start-jsk/rtmros_common/blob/master/hrpsys_tools/scripts/hrpsys_tools_config.py) は
この目的に役立つことを意図して作られています．
  - 参考: 作成時のディスカッション - [https://github.com/start-jsk/rtmros_common/issues/340](https://github.com/start-jsk/rtmros_common/issues/340)


### デジタルI/O の利用（NEXTAGE OPEN）

本項はデジタルI/O（DIO）が備わっている NEXTAGE OPEN でのみ有効です．

DIO 操作で利用できるメソッドの全体については利用可能なメソッドが集約されている API ドキュメント，
特にデフォルトで利用可能なメソッドが集められている `NextageClient` クラスを参照してください。

- APIドキュメント: [http://docs.ros.org/hydro/api/nextage_ros_bridge/html/annotated.html](http://docs.ros.org/hydro/api/nextage_ros_bridge/html/annotated.html)
- NextageClient: [http://docs.ros.org/hydro/api/nextage_ros_bridge/html/classnextage__ros__bridge_1_1nextage__client_1_1NextageClient.html](http://docs.ros.org/hydro/api/nextage_ros_bridge/html/classnextage__ros__bridge_1_1nextage__client_1_1NextageClient.html)

#### RTM クライアントからのインタラクティブな操作

まず RTM クライアントを実行し，ネームサーバー内部（つまり QNX コントローラボックス上）のロボットに接続する必要があります．

```
$ ipython -i `rospack find nextage_ros_bridge`/script/nextage.py -- --host %HOST%
```

そうすると `_hands` オブジェクトから全ての DIO メソッドにアクセスすることができます．
iPython ターミナルで表示させると次のようになります．

```python
In [6]: robot._hands.h
robot._hands.handlight_both     robot._hands.handtool_l_attach  robot._hands.handtool_r_eject
robot._hands.handlight_l        robot._hands.handtool_l_eject   
robot._hands.handlight_r        robot._hands.handtool_r_attach  

In [6]: robot._hands.g
robot._hands.gripper_l_close  robot._hands.gripper_r_close  
robot._hands.gripper_l_open   robot._hands.gripper_r_open   

In [6]: robot._hands.a
robot._hands.airhand_l_drawin   robot._hands.airhand_l_release  robot._hands.airhand_r_keep
robot._hands.airhand_l_keep     robot._hands.airhand_r_drawin   robot._hands.airhand_r_release
```

ハンドツールを接続・取り外しする場合は次のように行います．

- ハンドへのツールの接続
  - ツールチェンジャの先端にあるソレノイドバルブが閉じている必要があります．
    - イジェクトツールコマンドでバルブを閉じることができます．
    - [http://docs.ros.org/hydro/api/nextage_ros_bridge/html/classnextage__ros__bridge_1_1nextage__client_1_1NextageClient.html#a1702a5edb90cf8d6e67164760e2d6e91](http://docs.ros.org/hydro/api/nextage_ros_bridge/html/classnextage__ros__bridge_1_1nextage__client_1_1NextageClient.html#a1702a5edb90cf8d6e67164760e2d6e91)
- ハンドからツールの取り外し
  - ツールが落ちてきますので気をつけてください．
    - [http://docs.ros.org/hydro/api/nextage_ros_bridge/html/classnextage__ros__bridge_1_1nextage__client_1_1NextageClient.html#ac562d5f65e6e994692e17c499d0ff745](http://docs.ros.org/hydro/api/nextage_ros_bridge/html/classnextage__ros__bridge_1_1nextage__client_1_1NextageClient.html#ac562d5f65e6e994692e17c499d0ff745)

他のよく使われるメソッドについてはその名前自体から大体の機能は類推できるかと思います．

- グリッパの開閉: `gripper_l_open`, `gripper_r_open`, `gripper_l_close`, `gripper_r_close`
- ライトの点灯等: `handlight_both`, `handlight_l`, `handlight_r`

サンプルとしてハンドDIOのシステムテストツールも参考にしてください．

- [https://github.com/tork-a/rtmros_nextage/blob/7cf7d3ef1c1d24cd496dfc646c70e83565e8e854/nextage_ros_bridge/test/test_gripper.py](https://github.com/tork-a/rtmros_nextage/blob/7cf7d3ef1c1d24cd496dfc646c70e83565e8e854/nextage_ros_bridge/test/test_gripper.py)

#### 2014年8月よりも前に出荷された NEXTAGE OPEN での DIO の利用

2014年8月よりも前に出荷された NEXTAGE OPEN で DIO を利用する場合は
DIO のバージョンを指定する必要があります．

```
$ ipython -i `rospack find nextage_ros_bridge`/script/nextage.py -- --host %HOST% --dio_ver 0.4.2
```

`NextageClient` インスタンスに DIO を指定する場合には
`set_hand_version` メソッドにて引数 `0.4.2` を与えて指定してください．
このバージョン番号は固定で，変更する必要はありません．

- set_hand_version: [http://docs.ros.org/indigo/api/nextage_ros_bridge/html/classnextage__ros__bridge_1_1nextage__client_1_1NextageClient.html#a670568692e46cfc3f10633ad962b8616](http://docs.ros.org/indigo/api/nextage_ros_bridge/html/classnextage__ros__bridge_1_1nextage__client_1_1NextageClient.html#a670568692e46cfc3f10633ad962b8616)


## ROS Python インタフェース

### インタラクティブモードでの操作

#### rtm_ros_bridge の起動

rtm_ros_bridge を起動します．シミュレーションンの場合は不要です．

```
$ roslaunch hironx_ros_bridge hironx_ros_bridge_real.launch nameserver:=%HOSTNAME%    (HIRO)

$ roslaunch nextage_ros_bridge nextage_ros_bridge_real.launch nameserver:=%HOSTNAME%  (NEXTAGE OPEN)
```

ROSのノードが動作しているかを確認してみます．

```
$ rosnode list
/diagnostic_aggregator
/hrpsys_profile
/hrpsys_ros_diagnostics
/hrpsys_state_publisher
/rosout
```

#### iPython ターミナルの実行

インタラクティブにロボットを操作できるように iPython インタラクティブコンソールを実行します．

```
$ ipython -i `rospack find hironx_ros_bridge`/scripts/hironx.py -- --host hiro014     （HIRO)

$ ipython -i `rospack find nextage_ros_bridge`/script/nextage.py -- --host nextage101  (NEXTAGE OPEN)

:  (same initialization step as simulation)

[hrpsys.py]  initialized successfully
```

シミュレーションの場合は引数は不要です．

```
$ ipython -i `rospack find hironx_ros_bridge`/scripts/hironx.py

$ ipython -i `rospack find nextage_ros_bridge`/script/nextage.py
```

これらは RTM インタフェースの時と同じです．
つまり `hironx.py` や `nextage.py` を実行したときに
それらが RTM クライアントと ROS クライアントの両方を起動るということです．

iPython コンソールを起動したときに次のようなエラーが出ることがあります．
これは MoveIt! のサービスが起動していないという内容のエラーです．
ROS クライアントメソッドのいくつかは MoveIt! が無くても動作しますので
ここではこのエラーを無視しても大丈夫です．

```
:
[hrpsys.py]  initialized successfully
[INFO] [WallTime: 1410918126.749067] [206.795000] Joint names; Torso: ['CHEST_JOINT0']
        Head: ['HEAD_JOINT0', 'HEAD_JOINT1']
        L-Arm: ['LARM_JOINT0', 'LARM_JOINT1', 'LARM_JOINT2', 'LARM_JOINT3', 'LARM_JOINT4', 'LARM_JOINT5']
        R-Arm: ['RARM_JOINT0', 'RARM_JOINT1', 'RARM_JOINT2', 'RARM_JOINT3', 'RARM_JOINT4', 'RARM_JOINT5']
[ERROR] [1410918130.900627643, 210.289999999]: Robot semantic description not found. Did you forget to define or remap '/robot_description_semantic'?
[ INFO] [1410918130.900757547, 210.289999999]: Loading robot model 'HiroNX'...
[ INFO] [1410918130.900794422, 210.289999999]: No root joint specified. Assuming fixed joint
[FATAL] [1410918131.991522557, 211.249999999]: Group 'left_arm' was not found.
[ERROR] [WallTime: 1410918132.006310] [211.285000] Group 'left_arm' was not found.
Make sure you've launched MoveGroup (e.g. by launching moveit_planning_execution.launch)
```

#### ROS Python インタフェースコマンド

実行している iPython コンソールから ROS Python インタフェースのコマンドを見てみます．
次のように `ros.` と入力し TAB キーを押します．

```python
In [1]: ros.
ros.Joint                        ros.get_joint                    ros.go_init
ros.Link                         ros.get_joint_names              ros.go_offpose
ros.get_current_state            ros.get_link                     ros.has_group
ros.get_current_variable_values  ros.get_link_names               ros.set_joint_angles_deg
ros.get_default_owner_group      ros.get_planning_frame           ros.set_joint_angles_rad
ros.get_group                    ros.get_root_link                ros.set_pose
ros.get_group_names              ros.goInitial             
```

`ROS_Client` は `ros` というオブジェクトから利用することができます．

`ROS_Client` は MoveIt! から RobotCommander を継承するしていて，
その派生クラスから多くのメソッドが上記のように見えているのです．
加えて `goInitial`,`go_init`, `go_offpose`, `set_pose` などの `ROS_Client` 用にいくつかのメソッドが実装されています．
これらのメソッドはすべて RTM バージョン の Hironx/NEXTAGE OPEN Python インターフェイスと同等のものです．

次のコマンドでロボットの全関節が初期姿勢に移行します．

- 注意-1: コマンドを実行するとロボットが動きます．
- 注意-2: 緊急停止スイッチをいつでも押せる状態にしておいてください．

```python
In [1]: ros.go_init()
[INFO] [WallTime: 1410918153.591171] [226.790000] *** go_init begins ***
[INFO] [WallTime: 1410918165.419528] [233.825000] wait_for_result for the joint group rarm = True
[INFO] [WallTime: 1410918165.423372] [233.825000] [positions: [0.010471975511965976, 0.0, -1.7453292519943295, -0.26529004630313807, 0.16406094968746698, -0.05585053606381855]
velocities: []
accelerations: []
effort: []
time_from_start:
  secs: 7
  nsecs: 0]
```

iPython の機能を使うと RTM の場合と同様に各 ROS Python インタフェースコマンドのAPIドキュメントを見ることができます．

```python
In [2]: ros.go_init?
Type:       instancemethod
Base Class: <type 'instancemethod'>
String Form:<bound method ROS_Client.go_init of <hironx_ros_bridge.ros_client.ROS_Client object at 0x49b7210>>
Namespace:  Interactive
File:       /home/rosnoodle/cws_hiro_nxo/src/start-jsk/rtmros_hironx/hironx_ros_bridge/src/hironx_ros_bridge/ros_client.py
Definition: ros.go_init(self, task_duration=7.0)
Docstring:
Init positions are taken from HIRONX.
TODO: Need to add factory position too that's so convenient when
      working with the manufacturer.
@type task_duration: float
```

RTM Python インタフェースとの互換性により，
同等の機能を持つメソッドの名前が違っていても（`goInitial`）同じでもどちらでも利用することができます．

```python
IN [3]: ros.goInitial?
Type:       instancemethod
String Form:<bound method ROS_Client.goInitial of <hironx_ros_bridge.ros_client.ROS_Client object at 0x7f23458f0c50>>
File:       /home/n130s/link/ROS/indigo_trusty/cws_hironxo/src/start-jsk/rtmros_hironx/hironx_ros_bridge/src/hironx_ros_bridge/ros_client.py
Definition: ros.goInitial(self, init_pose_type=0, task_duration=7.0)
Docstring:
This method internally calls self.go_init.

This method exists solely because of compatibility purpose with
hironx_ros_bridge.hironx_client.HIRONX.goInitial, which
holds a method "goInitial".

@param init_pose_type:
       0: default init pose (specified as _InitialPose)
       1: factory init pose (specified as _InitialPose_Factory)
```


### ROS Python インタフェースプログラミング

#### サンプルコード - Acceptance Test (ROS)

"RTM Python インタフェースプログラミング" の
"サンプルコード - Acceptance Test (RTM)" におけるプログラムと同様のサンプルコードを見てみます．
このプログラムは RTM クライアントで行ったことと同じタスクを ROS クライアントで行っています．

##### acceptancetest_ros.py

ROS の Python サンプルコード acceptancetest_ros.py は次のようになっています．

- [https://raw.githubusercontent.com/start-jsk/rtmros_hironx/9d7c2b1d801450b09e814b12093e1cf1986b5565/hironx_ros_bridge/src/hironx_ros_bridge/testutil/acceptancetest_ros.py](https://raw.githubusercontent.com/start-jsk/rtmros_hironx/9d7c2b1d801450b09e814b12093e1cf1986b5565/hironx_ros_bridge/src/hironx_ros_bridge/testutil/acceptancetest_ros.py)

```python
1 # -*- coding: utf-8 -*-
2
3 # Software License Agreement (BSD License)
4 #
5 # Copyright (c) 2014, TORK (Tokyo Opensource Robotics Kyokai Association)
6 # All rights reserved.
7 #
8 # Redistribution and use in source and binary forms, with or without
9 # modification, are permitted provided that the following conditions
10 # are met:
11 #
12 #  * Redistributions of source code must retain the above copyright
13 #    notice, this list of conditions and the following disclaimer.
14 #  * Redistributions in binary form must reproduce the above
15 #    copyright notice, this list of conditions and the following
16 #    disclaimer in the documentation and/or other materials provided
17 #    with the distribution.
18 #  * Neither the name of TORK. nor the
19 #    names of its contributors may be used to endorse or promote products
20 #    derived from this software without specific prior written permission.
21 #
22 # THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
23 # "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
24 # LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
25 # FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
26 # COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
27 # INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
28 # BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
29 # LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
30 # CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
31 # LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
32 # ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
33 # POSSIBILITY OF SUCH DAMAGE.
34
35 import rospy
36
37 from hironx_ros_bridge.testutil.abst_acceptancetest import AbstAcceptanceTest
38
39
40 class AcceptanceTestROS(AbstAcceptanceTest):
41
42     def __init__(self, robot_client):
43         '''
44         @type robot_client: hironx_ros_bridge.ros_client.ROS_Client
45         '''
46         self._robotclient = robot_client
47
48     def go_initpos(self, default_task_duration=7.0):
49         self._robotclient.go_init(default_task_duration)
50
51     def set_joint_angles(self, joint_group, joint_angles, msg_tasktitle=None,
52                          task_duration=7.0, do_wait=True):
53         '''
54         @see: AbstAcceptanceTest.move_armbyarm_impl
55         '''
56         rospy.loginfo("'''{}'''".format(msg_tasktitle))
57         self._robotclient.set_joint_angles_deg(
58                          joint_group, joint_angles, task_duration, do_wait)
59
60     def set_pose(self, joint_group, posision, rpy, msg_tasktitle=None,
61                  task_duration=7.0, do_wait=True, ref_frame_name=None):
62         '''
63         @see: AbstAcceptanceTest.set_pose
64         '''
65         rospy.loginfo('ROS {}'.format(msg_tasktitle))
66         self._robotclient.set_pose(joint_group, posision, rpy, task_duration,
67                                    do_wait, ref_frame_name)
68
69     def set_pose_relative(
70                         self, joint_group, dx=0, dy=0, dz=0, dr=0, dp=0, dw=0,
71                         msg_tasktitle=None, task_duration=7.0, do_wait=True):
72         rospy.logerr('AcceptanceTestROS; set_pose_relative is not implemented yet')
73         pass
```

最初に ROS_Client クラスのインスタンス化が必要ですが，
それは上記のコードの外側，[acceptancetest_hironx.py 内]([https://github.com/start-jsk/rtmros_hironx/blob/be5fb8d5a5b8339d9507f3c287edb406ca48c9c9/hironx_ros_bridge/scripts/acceptancetest_hironx.py#L103](https://github.com/start-jsk/rtmros_hironx/blob/be5fb8d5a5b8339d9507f3c287edb406ca48c9c9/hironx_ros_bridge/scripts/acceptancetest_hironx.py#L103))で次のように行われています．

```python
self._robotclient = ROS_Client()
```

上記のコードに戻ります．
初期姿勢へ動くのは簡潔に1行で書かれています．

```python
48     def go_initpos(self, default_task_duration=7.0):
49         self._robotclient.go_init(default_task_duration)
```

各関節を角度指示を与えるのも1行で書かれています．

```python
51 def set_joint_angles(self, joint_group, joint_angles, msg_tasktitle=None, task_duration=7.0, do_wait=True):
:
57    self._robotclient.set_joint_angles_deg(joint_group, joint_angles, task_duration, do_wait)
```

直交座標系における姿勢指示も行うことができます．
（このコマンドでは MoveIt! サービスを利用しています）

```python
60 def set_pose(self, joint_group, posision, rpy, msg_tasktitle=None, task_duration=7.0, do_wait=True, ref_frame_name=None):
:
66     self._robotclient.set_pose(joint_group, posision, rpy, task_duration, do_wait, ref_frame_name)
```

##### サンプルコードの実行

iPython コンソールから ROS クライアントを利用した `acceptance.run_tests_ros()` を実行して
どのように動作しているかを確認してみます．

- 注意: ロボットが動きます．

```
$ ipython -i `rospack find hironx_ros_bridge`/scripts/acceptancetest_hironx.py -- --host %HOSTNAME%
:
IN [1]: acceptance.run_tests_ros()
```

<!-- EOF -->

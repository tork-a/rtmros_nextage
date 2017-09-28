
# チュートリアル: MoveIt! Python インタフェース

## MoveIt! Commander

MoveIt! の動作計画などの機能は GUI（RViz）からの操作を提供しているだけではありません．
そのプログラミングインタフェースである MoveIt! Commander も提供されていますので，
プログラミング言語から MoveIt! の機能を利用してロボットを動かすこともできます．

## MoveIt! Python インタフェース使用環境

MoveIt! の Python インタフェース MoveIt! Python Commander を使用するときの環境は次のようになっています．

- rtm_ros_hironx: バージョン 1.1.4 以降（ Hironx/Nextage Open ）
- 推奨プログラミングインタフェース: `ROS_Client`（ RobotCommander から派生 ）
- RobotCommander は MoveIt! の Python プログラミングインターフェス

MoveIt! の Python インタフェースである moveit_commander パッケージが
Ubuntu コンピュータ上にない場合はインストールする必要があります．

```
$ sudo apt-get install ros-%YOUR_ROS_DISTRO%-moveit-commander
```

## MoveIt! Python インタフェースからのロボット操作

### MoveIt! Python インタフェースの起動

#### rtm_ros_bridge の実行

rtm_ros_bridge を実行します．

```
$ roslaunch hironx_ros_bridge hironx_ros_bridge_real.launch nameserver:=%HOSTNAME%   (Hironx)

$ roslaunch nextage_ros_bridge nextage_ros_bridge_real.launch nameserver:=%HOSTNAME%   (NEXTAGE OPEN)
```

#### MoveIt! サーバの起動

`ROS_Client` クラスをベースにしたプログラムを実行するためには
その "クライアント" に対する "サーバ" として MoveIt! のノードが走っている必要があります．

MoveIt! を起動します．

```
$ roslaunch hironx_moveit_config moveit_planning_execution.launch   (HIRO)

$ roslaunch nextage_moveit_config moveit_planning_execution.launch  (NEXTAGE OPEN)
```


### Python での MoveIt! Commander を用いたロボット操作

Python から MoveIt! Commander を利用しているサンプルプログラムで，
HIRO / NEXTAGE OPEN ロボットをどのように動作させているのかを見てみます．

- [https://github.com/tork-a/rtmros_nextage/blob/indigo-devel/nextage_ros_bridge/script/nextage_moveit_sample.py](https://github.com/tork-a/rtmros_nextage/blob/indigo-devel/nextage_ros_bridge/script/nextage_moveit_sample.py)

サンプルコード全体を下に記載します．その後に各行について何をしているのかを見てみます．

```python
1 #!/usr/bin/env python
2 ##########################################
3 # @file     nextage_moveit_sample.py     #
4 # @brief    Nextage Move it demo program #
5 # @author   Ryu Yamamoto                 #
6 # @date     2015/05/26                   #
7 ##########################################
8 import moveit_commander
9 import rospy
10 import geometry_msgs.msg
11
12 def main():
13         rospy.init_node("moveit_command_sender")
14
15         robot = moveit_commander.RobotCommander()
16     
17         print "=" * 10, " Robot Groups:"
18         print robot.get_group_names()
19
20         print "=" * 10, " Printing robot state"
21         print robot.get_current_state()
22         print "=" * 10
23
24         rarm = moveit_commander.MoveGroupCommander("right_arm")
25         larm = moveit_commander.MoveGroupCommander("left_arm")
26
27         print "=" * 15, " Right arm ", "=" * 15
28         print "=" * 10, " Reference frame: %s" % rarm.get_planning_frame()
29         print "=" * 10, " Reference frame: %s" % rarm.get_end_effector_link()
30     
31         print "=" * 15, " Left ight arm ", "=" * 15
32         print "=" * 10, " Reference frame: %s" % larm.get_planning_frame()
33         print "=" * 10, " Reference frame: %s" % larm.get_end_effector_link()
34
35         #Right Arm Initial Pose
36         rarm_initial_pose = rarm.get_current_pose().pose
37         print "=" * 10, " Printing Right Hand initial pose: "
38         print rarm_initial_pose
39
40         #Light Arm Initial Pose
41         larm_initial_pose = larm.get_current_pose().pose    
42         print "=" * 10, " Printing Left Hand initial pose: "
43         print larm_initial_pose
44
45         target_pose_r = geometry_msgs.msg.Pose()
46         target_pose_r.position.x = 0.325471850974-0.01
47         target_pose_r.position.y = -0.182271241593-0.3
48         target_pose_r.position.z = 0.0676272396419+0.3
49         target_pose_r.orientation.x = -0.000556712307053
50         target_pose_r.orientation.y = -0.706576742941
51         target_pose_r.orientation.z = -0.00102461782513
52         target_pose_r.orientation.w = 0.707635461636
53         rarm.set_pose_target(target_pose_r)
54
55         print "=" * 10," plan1 ..."
56         rarm.go()
57         rospy.sleep(1)
58         
59         target_pose_l = [
60                 target_pose_r.position.x,
61                 -target_pose_r.position.y,
62                 target_pose_r.position.z,
63                 target_pose_r.orientation.x,
64                 target_pose_r.orientation.y,
65                 target_pose_r.orientation.z,
66                 target_pose_r.orientation.w
67         ]
68         larm.set_pose_target(target_pose_l)
69
70         print "=" * 10," plan2 ..."
71         larm.go()
72         rospy.sleep(1)
73         
74         #Clear pose
75         rarm.clear_pose_targets()
76
77         #Right Hand
78         target_pose_r.position.x = 0.221486843301
79         target_pose_r.position.y = -0.0746407547512
80         target_pose_r.position.z = 0.642545484602
81         target_pose_r.orientation.x = 0.0669013615474
82         target_pose_r.orientation.y = -0.993519060661
83         target_pose_r.orientation.z = 0.00834224628291
84         target_pose_r.orientation.w = 0.0915122442864
85         rarm.set_pose_target(target_pose_r)
86         
87         print "=" * 10, " plan3..."
88         rarm.go()
89         rospy.sleep(1)
90
91         print "=" * 10,"Initial pose ..."
92         rarm.set_pose_target(rarm_initial_pose)
93         larm.set_pose_target(larm_initial_pose)
94         rarm.go()
95         larm.go()
96         rospy.sleep(2)
97         
98 if __name__ == '__main__':
99     try:
100         main()
101     except rospy.ROSInterruptException:
102         pass
```

動作計画と実行を行う Python スクリプトの主要な部分は

1. エンドエフェクタのリンクの位置と姿勢をターゲットとして指定
2. ターゲットの姿勢まで動作させる

という手順になります．
また，そのための準備としてエンドエフェクタの姿勢などを取得しています．

Python スクリプトの各行を具体的に見ていきます．

MoveIt! の Python インタフェース は
`moveit_commander.MoveGroupCommander` で提供されます．

```python
import moveit_commander
import rospy
import geometry_msgs.msg
```

この Python スクリプトから `MoveGroupCommander` を使うために
`rospy.init_node()` を呼び出して ROS ノードを実行します．

```python
rospy.init_node("moveit_command_sender")
```

RobotCommander をインスタンス化します．

```python
robot = moveit_commander.RobotCommander()
```

ロボットの全ての Group の名前のリストを取得，表示します．

```python
print "=" * 10, " Robot Groups:"
print robot.get_group_names()
```

ロボット全体の状態を表示するとデバッグに役立ちます．

```python
print "=" * 10, " Printing robot state"
print robot.get_current_state()
```

現在のロボットの各腕の姿勢を取得します．

```python
rarm_current_pose = rarm.get_current_pose().pose
larm_current_pose = larm.get_current_pose().pose
```

初期姿勢オブジェクトに現在のロボットの各腕の姿勢を代入します．

```python
rarm_initial_pose = rarm.get_current_pose().pose
print "=" * 10, " Printing Right Hand initial pose: "
print rarm_initial_pose

larm_initial_pose = larm.get_current_pose().pose
print "=" * 10, " Printing Left Hand initial pose: "
print larm_initial_pose
```

動作計画を行いロボットを動作させます．

**Execution Plan 1**

```python
target_pose_r = geometry_msgs.msg.Pose()
target_pose_r.position.x = 0.325471850974-0.01
target_pose_r.position.y = -0.182271241593-0.3
target_pose_r.position.z = 0.0676272396419+0.3
target_pose_r.orientation.x = -0.000556712307053
target_pose_r.orientation.y = -0.706576742941
target_pose_r.orientation.z = -0.00102461782513
target_pose_r.orientation.w = 0.707635461636
rarm.set_pose_target(target_pose_r)

print "=" * 10," plan1 ..."
rarm.go()
rospy.sleep(1)
```

![MoveIt! Commander - Execute Plan 1](http://wiki.ros.org/rtmros_nextage/Tutorials/Programming_Hiro_NEXTAGE_OPEN_MOVEIT?action=AttachFile&do=get&target=Plan1.png)


**Execution Plan 2**

```python
target_pose_l = [
    target_pose_r.position.x,
    -target_pose_r.position.y,
    target_pose_r.position.z,
    target_pose_r.orientation.x,
    target_pose_r.orientation.y,
    target_pose_r.orientation.z,
    target_pose_r.orientation.w
]
larm.set_pose_target(target_pose_l)

print "=" * 10," plan2 ..."
larm.go()
rospy.sleep(1)
```

![MoveIt! Commander - Execute Plan 2](http://wiki.ros.org/rtmros_nextage/Tutorials/Programming_Hiro_NEXTAGE_OPEN_MOVEIT?action=AttachFile&do=get&target=Plan2.png)


**Execution Plan 3**

```python
rarm.clear_pose_targets()

#Right Hand
target_pose_r.position.x = 0.221486843301
target_pose_r.position.y = -0.0746407547512
target_pose_r.position.z = 0.642545484602
target_pose_r.orientation.x = 0.0669013615474
target_pose_r.orientation.y = -0.993519060661
target_pose_r.orientation.z = 0.00834224628291
target_pose_r.orientation.w = 0.0915122442864
rarm.set_pose_target(target_pose_r)

print "=" * 10, " plan3..."
rarm.go()
rospy.sleep(1)
```

![MoveIt! Commander - Execute Plan 3](http://wiki.ros.org/rtmros_nextage/Tutorials/Programming_Hiro_NEXTAGE_OPEN_MOVEIT?action=AttachFile&do=get&target=Plan3.png)


**Go Initial**

```python
print "=" * 10,"Initial pose ..."
rarm.set_pose_target(rarm_initial_pose)
larm.set_pose_target(larm_initial_pose)
rarm.go()
larm.go()
rospy.sleep(2)
```

![MoveIt! Commander - Go Initial](http://wiki.ros.org/rtmros_nextage/Tutorials/Programming_Hiro_NEXTAGE_OPEN_MOVEIT?action=AttachFile&do=get&target=Initial.png)

動画: "Nextage Move it! demo" [https://www.youtube.com/watch?v=heKEKg3I7cQ](https://www.youtube.com/watch?v=heKEKg3I7cQ)

また，MoveIt! を表示している RViz 上でもこれらの動作計画が表示されます．

実際にサンプルプログラムを実行して，
HIRO / NEXTAGE OPEN ロボットが動作計画どおりに動いているかを見てみます．

- 注意: ロボットが動きます．

ターミナルから次のコマンドを実行してサンプルプログラムを実行します．

```
$ rosrun nextage_ros_bridge nextage_moveit_sample.py   (NEXTAGE OPEN)
```

MoveIt! Commander では更に様々なメソッドがありますので
ROS Wiki の moveit_commander のページを見てください．

- ROS Wiki - moveit_commander
  - [http://wiki.ros.org/moveit_commander](http://wiki.ros.org/moveit_commander)


## MoveIt! Python インタフェースでの両腕の動作計画

MoveIt! Python インタフェースでの両腕の動作計画を利用したサンプルコードを下記に記します．

両腕での動作計画サンプルコード

```python
import moveit_commander
import rospy
from geometry_msgs.msg import Pose

def main():
        rospy.init_node("moveit_command_sender")

        botharms = moveit_commander.MoveGroupCommander("botharms")

        target_pose_r = Pose()
        target_pose_l = Pose()
        q = quaternion_from_euler(0, -math.pi/2,0)
        target_pose_r.position.x = 0.3
        target_pose_r.position.y = 0.1
        target_pose_r.position.z = 0.0
        target_pose_r.orientation.x = q[0]
        target_pose_r.orientation.y = q[1]
        target_pose_r.orientation.z = q[2]
        target_pose_r.orientation.w = q[3]
        target_pose_l.position.x = 0.3
        target_pose_l.position.y =-0.1
        target_pose_l.position.z = 0.3
        target_pose_l.orientation.x = q[0]
        target_pose_l.orientation.y = q[1]
        target_pose_l.orientation.z = q[2]
        target_pose_l.orientation.w = q[3]
        botharms.set_pose_target(target_pose_r, 'RARM_JOINT5_Link')
        botharms.set_pose_target(target_pose_l, 'LARM_JOINT5_Link')
        botharms.go()
        rospy.sleep(1)

        target_pose_r.position.x = 0.3
        target_pose_r.position.y =-0.2
        target_pose_r.position.z = 0.0
        target_pose_l.position.x = 0.3
        target_pose_l.position.y = 0.2
        target_pose_l.position.z = 0.0
        botharms.set_pose_target(target_pose_r, 'RARM_JOINT5_Link')
        botharms.set_pose_target(target_pose_l, 'LARM_JOINT5_Link')
        botharms.go()
```


## その他の ROS_Client クラス利用法

もう少し詳しく `ROS_Client` クラスについて見てみます．

ここでは iPython コンソールを用います．

```
$ ipython -i `rospack find hironx_ros_bridge`/scripts/hironx.py                      (simulation)

$ ipython -i `rospack find hironx_ros_bridge`/scripts/hironx.py -- --host %HOSTNAME% (for real robot)
```

クライアントクラスの `ros` がインスタンス化されていることを確認します．

```python
In [1]: ros                                                                                  
Out[1]: <hironx_ros_bridge.ros_client.ROS_Client at 0x7f0d34a00590>
```

初期姿勢にします．

```python
In [2]: ros.go_init()                                                                        
[INFO] [WallTime: 1453005565.730952] [1923.265000] *** go_init begins ***                    
[INFO] [WallTime: 1453005572.749935] [1930.345000] wait_for_result for the joint group rarm = True                                                                                       
[INFO] [WallTime: 1453005572.750268] [1930.345000] [positions: [0.010471975511965976, 0.0, -1.7453292519943295, -0.26529004630313807, 0.16406094968746698, -0.05585053606381855]         
velocities: []                                                                               
accelerations: []                                                                            
effort: []                                                                                   
time_from_start:                                                                             
  secs: 7                                                                                    
  nsecs: 0]   
```


### 関節角度指令コマンド

ロボットの腕関節角度（ラジアン）の目標値指示による動作をさせるコマンドです．

```python
In [3]: ros.set_joint_angles_rad('larm', [0.010, 0.0, -2.094, -0.265, 0.164, 0.06], duration=2, wait=True)                                                                                
[INFO] [WallTime: 1453005602.774010] [1960.440000] wait_for_result for the joint group larm = True                                                                                       

In [4]: ros.set_joint_angles_rad('larm', [0.010, 0.0, -1.745, -0.265, 0.164, 0.06], duration=2, wait=True)                                                                                
[INFO] [WallTime: 1453005606.789887] [1964.500000] wait_for_result for the joint group larm = True  
```

- `set_joint_angles_deg` により "度/degree" 指示も可能です．
- `set_joint_angles_*` メソッドは目標値の設定だけでなく，動作の実行も行います．


### エンドエフェクタ姿勢指令コマンド

ロボットのエンドエフェクタ（EEF）姿勢の目標値指示による動作をさせるコマンドです．

```python
IN [4]: ros.set_pose?
Type:       instancemethod
Definition: ros.set_pose(self, joint_group, position, rpy=None, task_duration=7.0, do_wait=True, ref_frame_name=None)
Docstring:
@deprecated: Use set_pose_target (from MoveGroupCommander) directly.
Accept pose defined by position and RPY in Cartesian format.

@type joint_group: str
@type position: [float]
@param position: x, y, z.
@type rpy: [float]
@param rpy: If None, keep the current orientation by using
            MoveGroupCommander.set_position_target. See:
             'http://moveit.ros.org/doxygen/' +
             'classmoveit__commander_1_1move__group_1_1' +
             'MoveGroupCommander.html#acfe2220fd85eeb0a971c51353e437753'
@param ref_frame_name: reference frame for target pose, i.e. "LARM_JOINT5_Link".

In [5]: ros.set_pose('larm', [0.3256221413929748, 0.18216922581330303, 0.16756590383473382], [-2.784279171494696, -1.5690964385875825, 2.899351051232168], task_duration=7, do_wait=True)                                                                                            
[INFO] [WallTime: 1453007509.751512] [3869.365000] setpose jntgr=larm mvgr=<moveit_commander.move_group.MoveGroupCommander object at 0x7f21aaa8a950> pose=[0.3256221413929748, 0.18216922581330303, 0.16756590383473383, -2.784279171494696, -1.5690964385875825, 2.899351051232168] posi=[0.3256221413929748, 0.18216922581330303, 0.16756590383473383] rpy=[-2.784279171494696, -1.5690964385875825, 2.899351051232168]                                                    

In [6]: ros.set_pose('larm', [0.3256221413929748, 0.18216922581330303, 0.06756590383473382], [-2.784279171494696, -1.5690964385875825, 2.899351051232168], task_duration=7, do_wait=True)                                                                                            
[INFO] [WallTime: 1453007518.445929] [3878.220000] setpose jntgr=larm mvgr=<moveit_commander.move_group.MoveGroupCommander object at 0x7f21aaa8a950> pose=[0.3256221413929748, 0.18216922581330303, 0.06756590383473382, -2.784279171494696, -1.5690964385875825, 2.899351051232168] posi=[0.3256221413929748, 0.18216922581330303, 0.06756590383473382] rpy=[-2.784279171494696, -1.5690964385875825, 2.899351051232168]   
```

### tf の利用

エンドエフェクタ（EEF）の任意のフレームに対する相対姿勢を取得するために
ROS の `tf` とそのリスナークライアントの `TransformListener` という強力なライブラリを使います．

次のコードは左手のリンクフレーム `/LARM_JOINT5_Link` の腰フレーム `/WAIST` に対する相対姿勢を取得する方法です．

```python
import tf

import rospy

listener = tf.TransformListener()
try:    
    (trans, rot) = listener.lookupTransform('/WAIST', '/LARM_JOINT5_Link', rospy.Time(0))
except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
```

取得された姿勢をロボットに送ります．

```python
In [7]: pos = list(trans)
In [8]: rpy = list(tf.transformations.euler_from_quaternion(rot))
In [9]: ros.set_pose('larm', pos, rpy, task_duration=7, do_wait=True)
[INFO] [WallTime: 1453006377.256805] [2735.650000] setpose jntgr=larm mvgr=<moveit_commander.move_group.MoveGroupCommander object at 0x7f0d1d31b950> pose=[0.3255715961317417, 0.18217283734133255, 0.06760844121713444, 2.8644073530085747, -1.569564170540247, -2.7497620461224517] posi=[0.3255715961317417, 0.18217283734133255, 0.06760844121713444] rpy=[2.8644073530085747, -1.569564170540247, -2.7497620461224517]
```

- 注意
  - `lookupTransform` 戻り値はタプル型なのに対して `set_pose` はリスト型を受け取る
    - 姿勢を `set_pose` メソッドに送る前に "タプル→リスト変換" を忘れないようにする．

`tf.transformation` にクォータニオンからオイラー角への変換を行うツールがあります．

NumPy ライブラリを利用することで位置や姿勢のベクトルを変更することができます．

- NumPy: [https://docs.scipy.org/doc/numpy-dev/user/quickstart.html](https://docs.scipy.org/doc/numpy-dev/user/quickstart.html)

```python
In [7]: ros.set_pose('larm', list(numpy.array(pos) + numpy.array([0,0,0.1])), rpy, task_duration=7, do_wait=True)                                                                        
[INFO] [WallTime: 1453006471.587639] [2830.075000] setpose jntgr=larm mvgr=<moveit_commander.move_group.MoveGroupCommander object at 0x7f0d1d31b950> pose=[0.32557159613174169, 0.18217283734133255, 0.16760844121713445, 2.8644073530085747, -1.569564170540247, -2.7497620461224517] posi=[0.32557159613174169, 0.18217283734133255, 0.16760844121713445] rpy=[2.8644073530085747, -1.569564170540247, -2.7497620461224517]
```

- メモ
  - 認識プロセスによって検出されたオブジェクト姿勢からエンドエフェクタの目標姿勢を算出するために `lookupTransform` を利用することもできます．
    - 例）ar_track_alvar などを用いて AR マーカオブジェクトのカメラからの相対位置を取得した場合
      - ar_track_alvar: [http://wiki.ros.org/ar_track_alvar](http://wiki.ros.org/ar_track_alvar)
  - 上記のコマンドでは HIRO / NEXTAGE OPEN のデフォルトのベースフレームである `/WAIST` フレームに対する相対姿勢を見ているので，必然的に Move Group の `get_current_pose` の結果と同じになります．
    - 使用目的に応じて参照フレームを変更してください．


<!-- EOF -->

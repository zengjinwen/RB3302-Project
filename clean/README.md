# RB3302 简洁导航版

本目录是独立重写的导航，不覆盖旧 mission。实机与 Gazebo 共用 navigation.py、planning.py、navigation.yaml，以及现有地图、EKF、AMCL插件与定位配置。无需为新导航编译 Python 包；既有 ROS 工作空间依赖仍需已安装。

## 一条命令运行仿真

```bash
bash ~/rb3302_fresh/sim_clean.sh X N X,O,Q,X
```

Gazebo 和 RViz一起启动。绿色线是规划路线，costmap为16cm静态膨胀。画面渲染采用兼容Ogre；雷达的服务器渲染仍为Ogre2。关闭主终端Ctrl+C退出。

Q中心放置40x40cm障碍并运行绕路任务：

```bash
SIM_OBSTACLE=Q bash ~/rb3302_fresh/sim_clean.sh X N X,P,G,X
```

只看仿真传感器：`SIM_MISSION=0 bash ~/rb3302_fresh/sim_clean.sh X N X,O,Q,X`。
无界面回归：`SIM_HEADLESS=1 SIM_RVIZ=0 bash ~/rb3302_fresh/sim_clean.sh X N X,O,Q,X`。

仿真ROS域默认为77且只在本机通信；真车域7。SIM_DOMAIN可为独立测试指定其他域。真实位姿只供测试记录，不输入导航。

## 实机入口（尚未上机验证）

机器人上：

```bash
bash ~/rb3302_fresh/run_clean.sh X N X,O,Q,X
```

电脑看真车RViz：

```bash
bash ~/rb3302_fresh/rviz_clean.sh
```

旧 run.sh 与 rviz.sh 保留，不能与新任务同时运行。新旧实机入口共用任务锁。

## 逻辑

1. 发布起点和朝向，等地图、AMCL、融合里程计及雷达。
2. A*在60cm字母格中心搜索四方向路线，细地图只检查墙和车身空间。压缩连续直线，保留转角和goal。
3. 沿分段中心线Pure Pursuit跟踪；根据前视点同时修正朝向与横向偏差。使用map→odom→base_link连续位姿，不叠加两份前进速度。
4. 前视方向偏差大于60度时进入独立转向。固定相对目标，累计融合odom转角，到角后发零速，停车0.3秒再跟踪；不执行短转标定，不用假设的固定秒数。剩余误差由路径跟踪处理，odom无进展的转向12秒报错停车。
5. 临近折点/goal速度随距离降低；在2.5cm接近范围切换下一段。goal另外停车等新AMCL，确认5cm以内才记到点并停3秒。
6. 激光先排除已知墙附近12cm的回波，再检查格中心±22cm内的有面积回波。至少5点、可见宽度20cm，持续同格0.6秒才确认。只保留最新确认的一格；如果挡住剩余路线，立即停车并重新A*。不是任意形状DWA绕障。
7. 前方20cm内、车身走廊内的未知回波提供近距离停车保护，不必等障碍稳定确认才避免撞上。
8. 反馈超过1秒失效则停车，AMCL三倍标准差横跨半格/一个正方向时也停车；激光持续请求AMCL更新，恢复可信定位后继续。数据失效和无路线不会假装成功。

## 参数及取舍

navigation.yaml集中8个参数：前进速度0.10m/s；转向控制量0.40；前视20cm；goal复核5cm；转角容差约3.4度；静态膨胀16cm；障碍可见宽度20cm；稳定确认0.6秒，另含起点/朝向/目标运行参数。

16cm是20x20cm车身对角半径14.1cm加约2cm余量，不复用紧急停车距离。地图离散分辨率5cm会影响显示和匹配精度。

仿真四轮中心距18cm、轮半径3.3cm、摩擦及指令响应为模型假设。硬件适配器复现旧驱动纯转向的左右轮指令映射，但不能证明真实固件将相同指令转换成相同轮速。40cm方形障碍是已说明的测试样例，不是老师已确认尺寸。仿真成功不等于实机已验证。

## 文件

- navigation.py：ROS接口、单一状态机、跟踪和稳定障碍确认。
- planning.py：静态膨胀、60cm图和A*。
- navigation.yaml：控制参数。
- robot.launch.py：真车一键启动，使用相同navigation.py。
- ../sim/clean.launch.py：仿真一键启动，使用相同navigation.py。
- 验证结果、独立真实轨迹和日志保存在 ../report_evidence/clean_navigation/。

## 更多仿真回归

`bash ~/rb3302_fresh/test_sim.sh` 会在独立ROS域81—86运行六组测试，最多同时两组，自动保存每组真实轨迹、AMCL误差和ROS消息连接证据。无须先启动sim_clean.sh。

测试可选环境变量：SIM_OBSTACLE_SIZE（方块边长m，默认0.4）；SIM_SPAWN_DX / SIM_SPAWN_DY（实际出生偏差m）；SIM_SPAWN_YAW（实际朝向偏差度，初始化仍用指定正方向）；SIM_LIDAR_NOISE（测距高斯标准差m，默认0.005）；SIM_GYRO_NOISE（角速度高斯标准差rad/s，默认0）。这些只改变仿真传感器/场景，不改变导航或AMCL算法。

任务完成、无障碍误报和无几何碰撞，与实际到点误差是否小于5cm分别评价；不会把任务完成冒充实机精度达标。

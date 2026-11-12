当前已验证的新导航入口：`bash ~/rb3302_fresh/sim_clean.sh X N X,O,Q,X`。详见 `../clean/README.md` 和 `../report_evidence/clean_navigation/RESULTS.md`。以下是旧仿真入口建立时的记录，不代表新版本验证状态。

# RB3302 Gazebo

状态：场景生成、ROS launch加载、地图栅格重建检查已通过。Gazebo尚未安装，尚未完成动态运行验证。

本地电脑只需安装一次：
```bash
sudo apt-get install -y ros-jazzy-ros-gz
```
启动Gazebo、RViz、EKF、AMCL和现有mission：
```bash
bash ~/rb3302_fresh/sim.sh X N X,P,G,X
```
只看传感器/地图，不启动任务：
```bash
SIM_MISSION=0 bash ~/rb3302_fresh/sim.sh X N X,P,G,X
```
在Q中心放40cm宽障碍：
```bash
SIM_OBSTACLE=Q bash ~/rb3302_fresh/sim.sh X N X,P,G,X
```
无界面模式：
```bash
SIM_HEADLESS=1 SIM_RVIZ=0 bash ~/rb3302_fresh/sim.sh X N X,P,G,X
```

复用当前已安装的rb3302_mission、A*、控制配置、地图、EKF、AMCL（含自定义运动模型插件），不复制第二套导航算法。sim.sh不会编译或回退运动代码；当前运动版本仍是带短转标定的版本，之前中断的恢复没有执行。

墙体从rb3302_refined_aligned.pgm逐像素生成，沿用地图分辨率、原点与方向；60cm字母格中心显示绿色标记。20x20cm车体，四轮，轮半径3.3cm、左右轮中心距18cm为模型假设，不是实测。雷达和IMU安装变换与现有URDF一致。每方向两轮由差速插件同时驱动。硬件适配器模拟原驱动纯转向的轮速映射；真实底盘指令响应和摩擦未标定，不能用仿真转弯时间作为实机标定值。

独立ROS域77、仅本机通信；真车域7不受影响。没有启动串口或硬件服务。真实位姿单独发布到/sim/ground_truth，只供比较，不输入定位或任务控制。

任务退出后保留Gazebo/RViz供检查，Ctrl+C关闭全部。仿真使用/clock，但现有任务部分计时使用系统单调时钟；仿真运行明显低于实时速度时，计时转弯结果不代表实机，需先解决性能或统一任务时基，不应拿错误时间结果调参数。

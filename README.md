# 四轮足机器人项目（更新中）

这是一个基于 **ROS 2 + Gazebo** 的四轮足混合机器人项目。

机械模型为作者自制，sw导出urdf并用Gazebo加载，目前已实现足端笛卡尔阻抗位置闭环，VMC虚拟模型控制，基本trot步态等等。

## 🛠️ 环境要求(推荐)
- **Ubuntu 22.04**
- **ROS 2 Humble**
- **Gazebo Classic 11**

### 安装依赖

忘记有些啥了（有空补上），看看运行时报错缺什么自己补。

## 📁 运行方式
1.**克隆仓库**
```bash
git clone https://github.com/Oakfox2022/USTB_Doggo.git
```
2.**进入工作空间**
```bash
cd USTB_Doggo/ros2_ws/
```
3.**构建**
```bash
colcon build
source install/setup.bash
```
4.**简单仿真实验**
```bash
ros2 launch ustb_doggo_urdf ustb_doggo.launch.py
```
5.**阻抗控制实验**
```bash
ros2 launch ustb_doggo_impedance_control ustb_doggo_impedance_control.launch.py
```
在弹出的终端窗口中输入键盘按键进行控制：（目前有效按键：w s p 自行探索）

6.**VMC控制实验**
```bash
ros2 launch ustb_doggo_virtual_model_control ustb_doggo_virtual_model_control.launch.py
```
在弹出的终端窗口中输入键盘按键进行控制：（目前有效按键：w a s d q e i j k l p 1 2 3 4 自行探索）
可实现机身姿态保持水平

7.**正运动学打印足端位置**
```bash
ros2 run ustb_doggo_impedance_control forward_kinematics
```

8.**slam-toolbox建图实验**
```bash
ros2 launch ustb_doggo_slam ustb_doggo_slam.launch.py
```
在弹出的终端窗口中输入键盘按键控制机器人移动完成建图


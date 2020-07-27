# Beidou_Navigation
Ros driver of Beidou positioning System  
  
## 准备工作  
安装好基站与移动端  
打开串口调试助手，device名称为/dev/ttyUSB0  
打开终端，打开串口权限，sudo chmod 666 /dev/ttyUSB0
打开设备即可接收到数据  
观察无人车使用的语句inspvaxa：  
#INSPVAXA,COM1,0,73.5,FINESTEERING,1695,309428.000,00000040,4e77,43562;INS_SOLUTION_GOOD,INS_PSRSP,51.11637873403,-114.03825114994,1063.6093,-16.9000,-0.0845,-0.0464,-0.0127,0.138023492,0.069459386,90.000923268,0.9428,0.6688,1.4746,0.0430,0.0518,0.0521,0.944295466,0.944567084,1.000131845,3,0*e877c178  
遥控无人车左拐右拐（画8）直至INS状态字变为INS_SOLUTION_GOOD，POS状态字变为INS_RTKFIXED

## 安装方法  
mkdir -p ~/catkin_workspace/src  
cd catkin_workspace/src  
git clone https://github.com/wangxizhe/Beidou_Navigation.git  
cd  
catkin_make  
  
## 运行方法  
source devel/setup.bash  
roslaunch novatel_driver beidou.launch  

## 话题介绍  
本驱动发布四个话题，分别为/imu/data、/gpa/fix、/gps/odom、/gps/vel  
### /imu/data（sensor_msgs::Imu）  
#### 此话题内存放的为惯导侧得的数据  
x为车的正右方、y为车前进方向、z为车的正上方。三个方向与惯导安装方式与内参校准方式有关，此处安装方式为Y轴朝后，X轴朝左，在内参标定时需对坐标系进行旋转，旋转至Y轴朝车前进方向，Z轴朝上。   
linear_acceleration为x、y、z三个方向的加速度  
angular_velocity为x、y、z三个方向的角速度  
orientation为偏航四元数，此偏航四元数由加速度与角速度侧得，由于没有磁力计数据，由此四元数测得的RPY只有roll与pitch是正确的  
### /gpa/fix（sensor_msgs::NavSatFix）  
#### 位置信息
latitude 纬度  
longitude  经度  
altitude 海拔高度  

#### position_covariance为误差矩阵  
position_covariance[0] 纬度误差  
position_covariance[4] 经度误差  
position_covariance[8] 海拔误差   
### /gps/odom（nav_msgs::Odometry）  
#### 姿态角信息  
pose.pose.orientation.x  Roll  
pose.pose.orientation.y  Pitch  
pose.pose.orientation.z  Yaw  
#### pose.covariance  误差矩阵  
pose.covariance[0] roll误差  
pose.covariance[1] pitch误差  
pose.covariance[5] yaw误差  
### /gps/vel（geometry_msgs::TwistWithCovarianceStamped）  
#### 速度信息  
twist.twist.linear.x   东方向的速度  
twist.twist.linear.y   北方向的速度  
twist.twist.linear.z   天方向的速度  

#### twist.covariance   误差矩阵  
twist.covariance[0]  东方向的速度误差  
twist.covariance[7]  北方向的速度误差  
twist.covariance[14] 天方向的速度误差  

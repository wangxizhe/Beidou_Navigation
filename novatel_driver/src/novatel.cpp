/****************************
 * 文件名：novatel.cpp
 * 创建人：东北大学-王希哲
 * 描 述：东北大学105实验室、北斗定位系统、ROS驱动
 * 日 期：2019-12-29
 * 版 本：1.0.0
 ***************************/
#include <ros/ros.h>
#include <serial/serial.h>
#include <string.h>
#include "std_msgs/String.h"
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>

#define Kp 2.0f
#define Ki 0.2f
#define  halfT 0.05f

using namespace std;
serial::Serial ser;
class Novatel_gps
{
public:
    Novatel_gps()
    {
       IMU_pub = nh.advertise<sensor_msgs::Imu>("/imu/data", 20);
       fix_pub = nh.advertise<sensor_msgs::NavSatFix>("/gps/fix", 20);
       vel_pub = nh.advertise<geometry_msgs::TwistWithCovarianceStamped>("/gps/vel", 20);
       odom_pub = nh.advertise<nav_msgs::Odometry>("/gps/odom", 20);

       str_latlon.clear();
       str_imu.clear();
    }

    void recieve();
    void SplitString(const string& s, vector<string>& v, const string& c);
    void Split_To_Float_int(const string &in, vector<float> &out_float, vector<int> &out_int, int index, int imu_index);
    void IMUupdate(double gx, double gy, double gz, double ax, double ay, double az);
    void mean_filter(double in, double out);

private:
  ros::NodeHandle nh;
  std::string str_latlon;
  std::string str_imu;
  float q0 = 1, q1 = 0, q2 = 0, q3 = 0;    // quaternion elements representing the estimated orientation
  float exInt = 0, eyInt = 0, ezInt = 0;    // scaled integral error
  ros::Publisher IMU_pub, fix_pub, vel_pub, odom_pub;
  double temp_x , temp_y , temp_z ;
  int count_a=0, count_g=0, LOCK=0;
  std::vector<double> vec_a;


};

void Novatel_gps::recieve()
{
    std_msgs::String result;
    std::string recv_str;
    ostringstream recv_str_ss;
    result.data = ser.read(ser.available());
    recv_str_ss <<result;
    recv_str=recv_str_ss.str();
    vector<string> vec_str_1;
    SplitString(recv_str, vec_str_1, ": ");
    vec_str_1[1].erase(vec_str_1[1].end() - 1);
//    if(vec_str_1[1].length()>0)
//         ROS_INFO_STREAM("Read: " << vec_str_1[1]);

    string::size_type pos1,pos2,pos3;
    pos1 = vec_str_1[1].find("%");
    pos2 = vec_str_1[1].find("#");
    pos3 = vec_str_1[1].find("*");
    if((pos1!=string::npos)&&(pos2!=string::npos))
    {
        vector<string> vec_str_2;
        SplitString(vec_str_1[1], vec_str_2, "\n");
        for (int i=0; i<vec_str_2.size(); i++)
        {
            if(*vec_str_2[i].begin()==37)
            {
                    str_imu.clear();
                    str_imu.append(vec_str_2[i]);
            }
            else if (*vec_str_2[i].begin()==35)
            {

                    str_latlon.clear();
                    str_latlon.append(vec_str_2[i]);
            }
            else
            {
                if(str_imu.length()>str_latlon.length())
                    str_imu.append(vec_str_1[1]);
                else
                    str_latlon.append(vec_str_1[1]);
            }
         }
    }
    else
    {
        if(pos1!=string::npos)         //%
        {
//                str_imu.clear();
//                if(pos3!=string::npos)        //*
//                {
//                    str_imu.append(vec_str_1[1]);
//                }
//                else
//                {
                    if(*vec_str_1[1].begin()==37)
                    {
                        str_imu.clear();
                        str_imu.append(vec_str_1[1]);
                    }
                    else
                    {
                        vector<string> vec_str_2;
                        SplitString(vec_str_1[1], vec_str_2, "\n");
                        str_latlon.append(vec_str_2[0]);
                        str_imu.clear();
                        str_imu.append(vec_str_2[1]);
                    }
//                }
        }
        else if(pos2!=string::npos)       //#
        {

            if(*vec_str_1[1].begin()==35)
            {
                str_latlon.clear();
                str_latlon.append(vec_str_1[1]);
            }
            else
            {
                vector<string> vec_str_2;
                SplitString(vec_str_1[1], vec_str_2, "\n");
                str_imu.append(vec_str_2[0]);
                str_latlon.clear();
                str_latlon.append(vec_str_2[1]);
            }


//            if(pos3!=string::npos)        //*
//            {
//                str_latlon.append(vec_str_1[1]);
//            }
//            else
//            {
//                if(*vec_str_1[1].begin()==35)
//                {
//                     str_latlon.append(vec_str_1[1]);
//                }
//                else
//                {
//                    vector<string> vec_str_2;
//                    SplitString(vec_str_1[1], vec_str_2, "\n");
//                    str_imu.append(vec_str_2[0]);
//                    str_latlon.append(vec_str_2[1]);
//                }
//            }
        }
        else
        {
            if(pos3!=string::npos)        //*
            {
                if(str_imu.length()>str_latlon.length())
                    str_imu.append(vec_str_1[1]);
                else
                    str_latlon.append(vec_str_1[1]);
            }
            else
            {
                if(str_imu.length()>str_latlon.length())
                    str_imu.append(vec_str_1[1]);
                else
                    str_latlon.append(vec_str_1[1]);
            }
        }
    }

    string::size_type pos4,pos5,pos6,pos7;
    pos4 = str_imu.find("*");
    pos6 = str_imu.find("%");
    pos5 = str_latlon.find("*");
    pos7 = str_latlon.find("#");
    if((*str_imu.begin()==37)&&(pos4!=string::npos))
    {
//        ROS_INFO_STREAM("Read1: " << str_imu);
        vector<int> vec_imu;
        vector<float> vec_latlon;
        Split_To_Float_int(str_imu, vec_latlon, vec_imu, 3, 1);
        str_imu.clear();
        double a_z=((vec_imu[0]*(0.2/65536)/125)/1000)*9.8065*125;//xright y front z up
        double a_y=-((vec_imu[1]*(0.2/65536)/125)/1000)*9.8065*125+0.1;
        double a_x=((vec_imu[2]*(0.2/65536)/125)/1000)*9.8065*125;
        double a_x_turn=-a_x;
        double a_y_turn=a_y;
        double a_z_turn=-a_z;

        double a_x_filter, a_y_filter, a_z_filter;
        mean_filter(a_x_turn, a_x_filter);
        mean_filter(a_y_turn, a_y_filter);
        mean_filter(a_z_turn, a_z_filter);

//        ROS_INFO("a_x_turn=%f, a_y_turn=%f, a_z_turn=%f", a_x, a_y, a_z);

        double g_z=((vec_imu[3]*(0.008/65536))/125)*50;
        double g_y=-((vec_imu[4]*(0.008/65536))/125)*50;
        double g_x=((vec_imu[5]*(0.008/65536))/125)*50;
        double g_z_2=g_z/180*M_PI;
        double g_y_2=g_y/180*M_PI;
        double g_x_2=g_x/180*M_PI;

        double g_x_turn, g_y_turn, g_z_turn;
        count_g++;
        if(count_g<200)
        {

            g_x_turn=-g_x_2;
            g_y_turn=g_y_2;
            g_z_turn=-g_z_2;
            temp_x += g_x_turn;
            temp_y += g_y_turn;
            temp_z += g_z_turn;
        }
        else
        {
            if(count_g==200)
            {
                temp_x /= 200;
                temp_y /= 200;
                temp_z /= 200;
            }
            g_x_turn=-g_x_2-temp_x;
            g_y_turn=g_y_2-temp_y;
            g_z_turn=-g_z_2-temp_z;
        }
//        ROS_INFO("g_x_turn=%f, g_y_turn=%f, g_z_turn=%f", a_x, a_y, a_z);
        IMUupdate(g_x_turn, g_y_turn, g_z_turn, a_x_filter, a_y_filter, a_z_filter);

        sensor_msgs::Imu imu_data;
        imu_data.header.stamp = ros::Time::now();
        imu_data.header.frame_id = "base_link";
        //四元数位姿,所有数据设为固定值，可以自己写代码获取ＩＭＵ的数据，，然后进行传递
        imu_data.orientation.x = q1;
        imu_data.orientation.y = q2;
        imu_data.orientation.z = q3;
        imu_data.orientation.w = q0;
        //线加速度
        imu_data.linear_acceleration.x = a_x_filter;
        imu_data.linear_acceleration.y = a_y_filter;
        imu_data.linear_acceleration.z = a_z_filter;
        //角速度
        imu_data.angular_velocity.x = g_x_turn;
        imu_data.angular_velocity.y = g_y_turn;
        imu_data.angular_velocity.z = g_z_turn;
        //发布IMU
        IMU_pub.publish(imu_data);
//        tf::Quaternion quat;
//        double roll, pitch, yaw;
//        tf::quaternionMsgToTF(imu_data.orientation, quat);
//        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
//        ROS_INFO("roll=%f, pitch=%f, yaw=%f", roll, pitch, yaw);
    }
    if((*str_latlon.begin()==35)&&(pos5!=string::npos))
    {
//        ROS_INFO_STREAM("Read2: " << str_latlon);
        vector<int> vec_imu;
        vector<float> vec_latlon;
        Split_To_Float_int(str_latlon, vec_latlon, vec_imu, 2, 0);
        str_latlon.clear();
        sensor_msgs::NavSatFix gps_data;
        gps_data.header.stamp = ros::Time::now();
        gps_data.header.frame_id = "base_link";
        gps_data.latitude=vec_latlon[0];
        gps_data.longitude=vec_latlon[1];
        gps_data.altitude=vec_latlon[2];
        gps_data.position_covariance[0]=vec_latlon[10]; //lat_error
        gps_data.position_covariance[4]=vec_latlon[11]; //lon_error
        gps_data.position_covariance[8]=vec_latlon[12]; //alt_+erroe
        fix_pub.publish(gps_data);

        geometry_msgs::TwistWithCovarianceStamped msg_vel;
        msg_vel.header.stamp=ros::Time::now();
        msg_vel.header.frame_id = "base_link";
        msg_vel.twist.twist.linear.x=vec_latlon[5];    //east
        msg_vel.twist.twist.linear.y=vec_latlon[4];    //north
        msg_vel.twist.twist.linear.z=vec_latlon[6];    //up
        msg_vel.twist.covariance[0] = vec_latlon[14]; // east
        msg_vel.twist.covariance[7] = vec_latlon[13]; // north
        msg_vel.twist.covariance[14] = vec_latlon[15]; // up
        vel_pub.publish(msg_vel);

        nav_msgs::Odometry msg_odom;
        msg_odom.header.stamp=ros::Time::now();
        msg_odom.header.frame_id = "base_link";
        msg_odom.pose.pose.orientation.x=vec_latlon[7];//roll
        msg_odom.pose.pose.orientation.y=vec_latlon[8];//pitch
        msg_odom.pose.pose.orientation.z=vec_latlon[9];//yaw

        msg_odom.pose.covariance[0]=vec_latlon[16];//roll
        msg_odom.pose.covariance[1]=vec_latlon[17];//pitch
        msg_odom.pose.covariance[5]=vec_latlon[18];//yaw
        odom_pub.publish(msg_odom);
    }
}
void Novatel_gps::SplitString(const string& s, vector<string>& v, const string& c)
{
    string::size_type pos1, pos2;
    pos2 = s.find(c);
    pos1 = 0;
   while(string::npos != pos2)
    {
       v.push_back(s.substr(pos1, pos2-pos1));
       pos1 = pos2 + c.size();
       pos2 = s.find(c, pos1);
   }
   if(pos1 != s.length())
        v.push_back(s.substr(pos1));
}
void Novatel_gps::Split_To_Float_int(const string &in, vector<float> &out_float, vector<int> &out_int, int index, int imu_index)
{
    vector<string> vec_str_3;
    SplitString(in, vec_str_3, ";");
    vector<string> vec_str_4;
    SplitString(vec_str_3[1], vec_str_4, "*");
    vector<string> vec_final;
    SplitString(vec_str_4[0], vec_final, ",");
    if(imu_index==1)
    {
        for (int i=index; i<vec_final.size(); i++)
        {
            istringstream iss(vec_final[i]);
            int num;
            iss >> num;
            out_int.push_back(num);
        }

    }
    else
    {
        for (int i=index; i<vec_final.size(); i++)
        {
            istringstream iss(vec_final[i]);
            float num;
            iss >> num;
            out_float.push_back(num);
        }
    }

}
//https://blog.csdn.net/CALL_LKC/article/details/74938681
void Novatel_gps::IMUupdate(double gx, double gy, double gz, double ax, double ay, double az)
{
  double norm;
  double vx, vy, vz;
  double ex, ey, ez;

  // normalise the measurements
  norm = sqrt(ax * ax + ay * ay + az * az);
  ax = ax / norm;
  ay = ay / norm;
  az = az / norm;

  // estimated direction of gravity
  vx = 2.0 * (q1 * q3 - q0 * q2);
  vy = 2.0 * (q0 * q1 + q2 * q3);
  vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

  // error is sum of cross product between reference direction of field and direction measured by sensor
  ex = (ay * vz - az * vy);
  ey = (az * vx - ax * vz);
  ez = (ax * vy - ay * vx);

  // integral error scaled integral gain
  exInt = exInt + ex * Ki;
  eyInt = eyInt + ey * Ki;
  ezInt = ezInt + ez * Ki;

  // adjusted gyroscope measurements
  gx = gx + Kp * ex + exInt;
  gy = gy + Kp * ey + eyInt;
  gz = gz + Kp * ez + ezInt;

  // integrate quaternion rate and normalise
  q0 = q0 + (-q1 * gx - q2 * gy - q3 * gz) * halfT;
  q1 = q1 + (q0 * gx + q2 * gz - q3 * gy) * halfT;
  q2 = q2 + (q0 * gy - q1 * gz + q3 * gx) * halfT;
  q3 = q3 + (q0 * gz + q1 * gy - q2 * gx) * halfT;

  // normalise quaternion
  norm = sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 = q0 / norm;
  q1 = q1 / norm;
  q2 = q2 / norm;
  q3 = q3 / norm;

}

void Novatel_gps::mean_filter(double in, double out)
{
    if(LOCK==0)
    {
        for (int i=0; i<10; i++)
        {
            vec_a.push_back(0);
        }
        LOCK=1;
    }
    vec_a[10]=vec_a[9];
    vec_a[9]=vec_a[8];
    vec_a[8]=vec_a[7];
    vec_a[7]=vec_a[6];
    vec_a[6]=vec_a[5];
    vec_a[5]=vec_a[4];
    vec_a[4]=vec_a[3];
    vec_a[3]=vec_a[2];
    vec_a[2]=vec_a[1];
    vec_a[1]=vec_a[0];
    vec_a[0]=in;

    static int cnst=0;
    if(cnst<vec_a.size())
    {
        cnst++;
        out=in;
    }
    else
    {
        std::vector<double> vec_a1;
        for (int i=0; i<vec_a.size(); i++)
        {
            vec_a1.push_back(vec_a[i]);
        }
        for(int i=0; i<vec_a1.size(); i++)
        {
            for (int j=0; j<vec_a1.size()-i; j++)
            {
                 if(vec_a1[j]>vec_a1[j+1])
                 {
                     float temp=vec_a1[j];
                     vec_a1[j]=vec_a1[j+1];
                     vec_a1[j+1]=temp;
                 }
            }
        }

        float sum=0;
        for(int i=1; i<vec_a1.size()-1; i++)
        {
            sum+=vec_a1[i];
        }
        out=sum/(vec_a1.size()-2);
    }
}

int main(int argc, char** argv)
{
      ros::init(argc, argv, "Novatel_gps");
      try
         {
             ser.setPort("/dev/ttyUSB0");
             ser.setBaudrate(115200);
             serial::Timeout to = serial::Timeout::simpleTimeout(50);
             ser.setTimeout(to);
             ser.open();
         }
      catch (serial::IOException& e)
      {
         ROS_ERROR_STREAM("Unable to open port ");
         return -1;
      }
      if(ser.isOpen())
      {
         ROS_INFO_STREAM("Serial Port initialized");
      }
      else
      {
         return -1;
      }
      Novatel_gps novatel_gps;
      while(ros::ok())
      {
         novatel_gps.recieve();

      }
      return 0;
}


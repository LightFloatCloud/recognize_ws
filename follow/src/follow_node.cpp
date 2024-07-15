#include "ros/ros.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/Twist.h"
#include <geometry_msgs/Point.h>


float obj_x,obj_y;
double track_on=0;
void position_callback(const geometry_msgs::Point::ConstPtr& msg)
{
    //ROS_INFO("Received message: x=%.4f, y=%.4f.",msg->x,msg->y);
    obj_x = msg->x;
    obj_y = msg->y;

    track_on = msg->z;
}


int main(int argc,char * argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"follow_node");
    ros::NodeHandle nh("~");

    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener listener(buffer);

    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel",1000);
    geometry_msgs::Twist twist;

double yuzhi_anquan;
nh.param<double>("yuzhi_anquan", yuzhi_anquan, 2);
geometry_msgs::Point obj_point;
ros::Subscriber sub = nh.subscribe("/obj_xyz", 1, position_callback);

int debug_on;
nh.param<int>("debug_on",debug_on , 0);

double xishu_linearx;
nh.param<double>("xishu_linearx",xishu_linearx , 0.5);

double xishu_angularz;
nh.param<double>("xishu_angularz",xishu_angularz , 2);

double yuzhi_x;
nh.param<double>("yuzhi_x",yuzhi_x , 3);

double yuzhi_z;
nh.param<double>("yuzhi_z",yuzhi_z , 3);

bool info_print;
nh.param<bool>("info_print", info_print, false);

    ros::Rate rate(10);

    while (ros::ok() )
    {
        try
        {
            //geometry_msgs::TransformStamped msgs_trans = buffer.lookupTransform("robot2/base_footprint","robot1/base_footprint",ros::Time(0));    
            twist.linear.x = xishu_linearx * (sqrt(pow(obj_x,2) + pow(obj_y,2))-yuzhi_anquan);
            twist.angular.z = xishu_angularz * atan2(obj_y,obj_x);

            

            if (twist.linear.x>yuzhi_x)
            {
               twist.linear.x=yuzhi_x;
            };
            if (twist.angular.z>yuzhi_z)
            {
               twist.angular.z=yuzhi_z;
            };
            if(abs(track_on) <0.001)
            {
                twist.angular.z = 0;
                twist.linear.x = 0;
            }

            if(info_print)
            {
                ROS_INFO("----------------------------");
                ROS_INFO("linear: %.4f angular: %.4f",twist.linear.x,twist.angular.z);
                if(track_on)
                {
                    ROS_INFO("Object Tracked: x: %f y: %f ",obj_x,obj_y);
                }
                else
                {
                    ROS_INFO("Tracked Failed!");
                }
            }

            if(debug_on)
            {
                pub.publish(twist);
            }
        }
        catch(const std::exception& e)
        {
            ROS_INFO("error %s",e.what());
        }

        rate.sleep();

        ros::spinOnce();
        

        
    }
    return 0;
}

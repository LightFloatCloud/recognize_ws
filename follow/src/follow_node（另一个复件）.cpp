#include "ros/ros.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/Twist.h"

int main(int argc,char * argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"flow_sub");
    ros::NodeHandle nh;

    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener listener(buffer);

    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/robot2/cmd_vel",1000);
    geometry_msgs::Twist twist;

    ros::Rate rate(10);

    while (ros::ok())
    {
        try
        {
            geometry_msgs::TransformStamped msgs_trans = buffer.lookupTransform("robot2/base_footprint","robot1/base_footprint",ros::Time(0));    
            twist.linear.x = 0.5 * (sqrt(pow(msgs_trans.transform.translation.x,2) + pow(msgs_trans.transform.translation.y,2))-2);
            twist.angular.z = 2 * atan2(msgs_trans.transform.translation.y,msgs_trans.transform.translation.x);

            ROS_INFO("[robot2]----------------------------");
            ROS_INFO("[robot2]--linear: %f angular: %f",twist.linear.x,twist.angular.z);
            ROS_INFO("[robot2]--tx: %f ty: %f tz %f",msgs_trans.transform.translation.x,msgs_trans.transform.translation.y,msgs_trans.transform.translation.z);

            if (twist.linear.x>3)
            {
               twist.linear.x=3;
            };
            if (twist.angular.z>3)
            {
               twist.angular.z=3;
            };

           pub.publish(twist);
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
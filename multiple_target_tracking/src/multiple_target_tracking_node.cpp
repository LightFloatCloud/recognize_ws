#include <omp.h>
#include <ros/ros.h>
#include <string>

#include <std_msgs/Header.h>
////////////
#include <geometry_msgs/Point.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <perception_msgs/Obstacle.h>
#include <perception_msgs/ObstacleArray.h>

#include <Eigen/Dense>
#include <cassert>
#include <limits.h>
#include <float.h>

#include "kalman.hpp"
#include "obj.hpp"



static std::vector<int> COLOR_B = {244, 233, 156, 103, 63, 33, 3, 0, 0};
static std::vector<int> COLOR_G = {67, 30, 99, 58, 81, 150, 169, 188, 150};
static std::vector<int> COLOR_R = {54, 99, 176, 183, 181, 243, 244, 212, 136};

class MTT
{
private:
    std::string sub_topic_;
    std::string pub_topic_;
    std::string pub_topic_obstacle_array_;
    std::string frame_id_;
    //////////////////////////////////////////
    std::string pub_topic_position_;

    bool show_objects_num_;
    bool show_time_;

    double time_interval_;
    double gate_threshold_;
    int blind_update_limit_;

    double sigma_ax_;
    double sigma_ay_;
    double sigma_ox_;
    double sigma_oy_;

    double min_scale_;
    double max_scale_;
    double min_height_;
    double max_height_;
    ///////////////
    double confidence_;
    double corrupt_time_;

    ros::Subscriber sub_;
    ros::Publisher pub_;
    ros::Publisher pub_obstacle_array_;
    ///////////////////////////////////
    ros::Publisher pub_position_;
    

    std::vector<Object> objs_observed_;
    std::vector<int> objs_label_; // 0: unoccupied, 1: for objs_tracked_, 2: for objs_temp_
    /////////////////////////////////
    //std::vector<double> objs_tracked_distance_;
    int current_tracked_id_ = -1;  // 当前追踪的物体id  
    int next_tracked_id_ = -1;    // 下一个要追踪的物体的ID，用于延迟切换  
    ros::Time next_switch_time_;  // 下一次切换追踪目标的时间（基于某种时间单位） 
    void updateClosestObject();



    std::vector<Object> objs_tracked_;
    std::vector<Object> objs_temp_;
    int number_;

    void observe(const visualization_msgs::MarkerArray& in);
    void update();
    void augment();
    void publishMarkers(visualization_msgs::MarkerArray& markers);
    void publishObstacles(perception_msgs::ObstacleArray& obstacles);
    void callback(const visualization_msgs::MarkerArray& in);

public:
    MTT(ros::NodeHandle& nh);
    ~MTT();
};

MTT::MTT(ros::NodeHandle& nh)
{
    nh.param<std::string>("sub_topic", sub_topic_, "/objects");
    nh.param<std::string>("pub_topic", pub_topic_, "/objects_tracked");
    nh.param<std::string>("pub_topic_obstacle_array", pub_topic_obstacle_array_, "/obstacle_array");
    nh.param<std::string>("frame_id", frame_id_, "pandar");

    nh.param<bool>("show_objects_num", show_objects_num_, false);
    nh.param<bool>("show_time", show_time_, false);

    nh.param<double>("time_interval", time_interval_, 0.1);
    nh.param<double>("gate_threshold", gate_threshold_, 1000);
    nh.param<int>("blind_update_limit", blind_update_limit_, 5);

    nh.param<double>("sigma_ax", sigma_ax_, 0.1);
    nh.param<double>("sigma_ay", sigma_ay_, 0.1);
    nh.param<double>("sigma_ox", sigma_ox_, 0.1);
    nh.param<double>("sigma_oy", sigma_oy_, 0.1);

    nh.param<double>("min_scale", min_scale_, 1.0);
    nh.param<double>("max_scale", max_scale_, 6.0);
    nh.param<double>("min_height", min_height_, 1.0);
    nh.param<double>("max_height", max_height_, 2.5);

    sub_ = nh.subscribe(sub_topic_, 1, &MTT::callback, this);
    pub_ = nh.advertise<visualization_msgs::MarkerArray>(pub_topic_, 1);
    pub_obstacle_array_ = nh.advertise<perception_msgs::ObstacleArray>(pub_topic_obstacle_array_, 1);

    /***************************/
    nh.param<double>("confidence", confidence_, 0.38);
    nh.param<double>("corrupt_time", corrupt_time_, 2);
    nh.param<std::string>("pub_topic_position", pub_topic_position_, "/obj_xyz");
    pub_position_ = nh.advertise<geometry_msgs::Point>(pub_topic_position_,10); 
    


    number_ = 0;

    ros::spin();
}

MTT::~MTT()
{
}

// 查找最近的物体并更新信息  
void MTT::updateClosestObject() 
{  
    const double Track_Thres = 15; // 超过这个距离不主动跟踪了

    double closest_distance_ = std::numeric_limits<double>::max();  
    int new_closest_id = -1;  // 当前最近的id
    for (int i = 0; i < objs_tracked_.size(); ++i) {  
        //if (objs_tracked_[i].number == current_tracked_id_) continue;
        double x = objs_tracked_[i].x0;
        double y = objs_tracked_[i].y0;
        double distance = x*x + y*y;
        if (distance < closest_distance_) {  
            closest_distance_ = distance;  
            //new_closest_id = i;  
            new_closest_id = objs_tracked_[i].number;  
        }  
    }  
    if(current_tracked_id_ == -1) {  // 未跟踪则跟踪最近的物体
        if(closest_distance_ < Track_Thres*Track_Thres)
            current_tracked_id_ = new_closest_id;
        return;
    }

    bool is_current_disappeared = std::find_if(objs_tracked_.begin(), objs_tracked_.end(),  
                                                   [&](const Object& obj) { return obj.number == current_tracked_id_; }) == objs_tracked_.end();  
    // 物体消失， 则跟踪next_tracked_id_或最近的物体
    if (is_current_disappeared) {  
        bool is_next_still_there = std::find_if(objs_tracked_.begin(), objs_tracked_.end(),  
                                                [&](const Object& obj) { return obj.number == next_tracked_id_; }) != objs_tracked_.end();  
        current_tracked_id_ = (is_next_still_there) ? next_tracked_id_ : new_closest_id ; // 物体消失，立即切换到最近的  
        next_tracked_id_ = -1;
        return;  
    }  
    else if(current_tracked_id_ == new_closest_id) { // 物体未消失，但它自己就是最近的物体
        next_tracked_id_ = -1;
        return;
    }

    // 处理切换追踪目标的逻辑，有比追踪物体更近的物体
    if (new_closest_id != current_tracked_id_ && new_closest_id != -1) {  
        // 如果找到了一个新的更近物体，并且不是当前追踪的物体  
        if(new_closest_id != next_tracked_id_) {
            next_tracked_id_ = new_closest_id;  
            next_switch_time_ = ros::Time::now() + ros::Duration(corrupt_time_); // 设置切换时间  
        }
    }  

    // 检查是否到了切换时间  
    if (next_tracked_id_ != -1 && ros::Time::now() >= next_switch_time_) {  
        // 如果两秒已过且新物体仍然存在，则切换追踪目标  
        bool is_next_still_there = std::find_if(objs_tracked_.begin(), objs_tracked_.end(),  
                                                [&](const Object& obj) { return obj.number == next_tracked_id_; }) != objs_tracked_.end();  
        if (is_next_still_there) {  
            current_tracked_id_ = next_tracked_id_;  
            next_tracked_id_ = -1; // 重置下一个追踪目标ID  
        }  
        else {
            std::cerr << "No next still there!" << std::endl;
        }
    }  
}  
void MTT::observe(const visualization_msgs::MarkerArray& in)
{
    objs_observed_.clear();
    objs_label_.clear();
    
    
    for(int i = 0; i < in.markers.size(); i++)
    {
        Object obj;
        obj.x0 = in.markers[i].pose.position.x;
        obj.y0 = in.markers[i].pose.position.y;
        obj.z0 = in.markers[i].pose.position.z;
        obj.l = in.markers[i].scale.x;
        obj.w = in.markers[i].scale.y;
        obj.h = in.markers[i].scale.z;
        // obj.l = 0.5;
        // obj.w = 0.5;
        // obj.h = 1.7;

        double xx = in.markers[i].pose.orientation.x;
        double yy = in.markers[i].pose.orientation.y;
        double zz = in.markers[i].pose.orientation.z;
        double ww = in.markers[i].pose.orientation.w;
        
        Eigen::Quaterniond q(ww, xx, yy, zz);
        Eigen::Vector3d q_eul = q.toRotationMatrix().eulerAngles(2, 1, 0);
        double phi = q_eul[0];

        obj.phi = phi;
        // obj.phi = 0;
        obj.has_orientation = false;

        obj.xref = in.markers[i].pose.position.x;
        obj.yref = in.markers[i].pose.position.y;

        double obj_scale = obj.l > obj.w ? obj.l : obj.w;
        double obj_height = obj.h;
        
        if((obj_scale >= min_scale_) && (obj_scale <= max_scale_) &&
           (obj_height >= min_height_) && (obj_height <= max_height_))
        {
            objs_observed_.push_back(obj);
        }

        std::string text = in.markers[i].text;
        if (!text.empty()) {
            size_t colon_pos = text.find(':');
            if (colon_pos != std::string::npos) {
                std::string label = text.substr(0, colon_pos);
                double confidence = std::stod(text.substr(colon_pos + 1));
                if (label == "Pedestrian" && confidence >= confidence_) {
                    objs_observed_.push_back(obj);
                }
                else if (label == "AvgI" && confidence >= confidence_) {
                    objs_observed_.push_back(obj);
                }
            }
        }


    }
    
    int num = objs_observed_.size(); objs_label_.resize(num);
    for(int i = 0; i < num; i++)
    {
        objs_label_[i] = 0;
    }
}

// track -> track  track -> lost
void MTT::update()
{
    #pragma omp for
    for(int j = 0; j < objs_tracked_.size(); j++) // 第j个 追踪后目标
    {
        bool flag = false; // 标记是否找到匹配  
        int idx = 0;  // 匹配的观察到物体的索引  
        double ddm = DBL_MAX;  // 最小距离的平方（初始化为最大可能值）

        // 遍历每个观察到的物体，寻找匹配 
        for(int k = 0; k < objs_observed_.size(); k++) // 第k个 追踪前目标
        {
            if(objs_label_[k] != 0) {continue;} // 如果已经标记过（不是未匹配的），则跳过
            
            double x = objs_observed_[k].xref;
            double y = objs_observed_[k].yref;
            double dd = objs_tracked_[j].tracker.compute_the_residual(x, y);
            if((dd < ddm) && (dd < gate_threshold_))
            {
                idx = k; ddm = dd; flag = true;  // 令第idx号obj成为与j号匹配的号码 objs_tracked_[j]=objs_observed_[idx]
            }
        }
        // 如果找到匹配
        if(flag)
        {
            objs_label_[idx] = 1; // 标记为已匹配 
            
            double zx = objs_observed_[idx].xref;
            double zy = objs_observed_[idx].yref;
            objs_tracked_[j].tracker.predict();
            objs_tracked_[j].tracker.update(zx, zy);
            objs_tracked_[j].tracker_blind_update = 0;

            objs_tracked_[j].xref = objs_tracked_[j].tracker.get_state()(0, 0);
            objs_tracked_[j].vx = objs_tracked_[j].tracker.get_state()(1, 0);
            objs_tracked_[j].yref = objs_tracked_[j].tracker.get_state()(2, 0);
            objs_tracked_[j].vy = objs_tracked_[j].tracker.get_state()(3, 0);

            objs_tracked_[j].x0 = objs_observed_[idx].x0;
            objs_tracked_[j].y0 = objs_observed_[idx].y0;
            objs_tracked_[j].z0 = objs_observed_[idx].z0;
            objs_tracked_[j].l = objs_observed_[idx].l;
            objs_tracked_[j].w = objs_observed_[idx].w;
            objs_tracked_[j].h = objs_observed_[idx].h;
            objs_tracked_[j].phi = objs_observed_[idx].phi;
            objs_tracked_[j].has_orientation = objs_observed_[idx].has_orientation;
            
        }
        else
        {
            objs_tracked_[j].tracker.predict();
            objs_tracked_[j].tracker_blind_update += 1; // 本轮无可匹配的值

            objs_tracked_[j].xref = objs_tracked_[j].tracker.get_state()(0, 0);
            objs_tracked_[j].vx = objs_tracked_[j].tracker.get_state()(1, 0);
            objs_tracked_[j].yref = objs_tracked_[j].tracker.get_state()(2, 0);
            objs_tracked_[j].vy = objs_tracked_[j].tracker.get_state()(3, 0);
            
            objs_tracked_[j].x0 = objs_tracked_[j].xref;
            objs_tracked_[j].y0 = objs_tracked_[j].yref;
        }
    }
    // 遍历所有物体，仅添加存活物体 
    std::vector<int> miss;
    for(int j = 0; j < objs_tracked_.size(); j++)
    {
        miss.push_back(objs_tracked_[j].tracker_blind_update);
    }
    std::vector<Object> objs_copy = objs_tracked_; objs_tracked_.clear(); 
    for(int j = 0; j < objs_copy.size(); j++)
    {
        if(miss[j] <= blind_update_limit_)
        {
            objs_tracked_.push_back(objs_copy[j]);
        }
    }
}

// temp -> track(连续两帧) temp -> lost(无连续两帧) observe -> temp  
void MTT::augment()
{
    #pragma omp for
    for(int j = 0; j< objs_temp_.size(); j++)
    {
        bool flag = false;
        int idx = 0;
        double ddm = DBL_MAX;

        for(int k = 0; k < objs_observed_.size(); k++) // 能匹配上 objs_temp_ 的 objs_label_=2
        {
            if(objs_label_[k] != 0) {continue;}
            
            double x = objs_observed_[k].xref;
            double y = objs_observed_[k].yref;
            double dd = objs_temp_[j].tracker.compute_the_residual(x, y);
            if((dd < ddm) && (dd < gate_threshold_))
            {
                idx = k; ddm = dd; flag = true;
            }
        }

        if(flag)
        {
            objs_label_[idx] = 2;
            
            double zx = objs_observed_[idx].xref;
            double zy = objs_observed_[idx].yref;
            double x = objs_temp_[j].tracker.get_state()(0, 0);
            double y = objs_temp_[j].tracker.get_state()(2, 0);
            double t = time_interval_;
            objs_temp_[j].tracker.init(t, zx, (zx - x) / t, zy, (zy - y) / t,
                                       sigma_ax_, sigma_ay_, sigma_ox_, sigma_oy_);
            // 编号生成  track 的 id
            number_ += 1;
            number_ = number_ % 10000;
            objs_temp_[j].number = number_;
            
            objs_temp_[j].x0 = objs_observed_[idx].x0;
            objs_temp_[j].y0 = objs_observed_[idx].y0;
            objs_temp_[j].z0 = objs_observed_[idx].z0;
            objs_temp_[j].l = objs_observed_[idx].l;
            objs_temp_[j].w = objs_observed_[idx].w;
            objs_temp_[j].h = objs_observed_[idx].h;
            objs_temp_[j].phi = objs_observed_[idx].phi;
            objs_temp_[j].has_orientation = objs_observed_[idx].has_orientation;

            assert((COLOR_B.size() == COLOR_G.size()) && (COLOR_B.size() == COLOR_R.size()));
            
            int num_c = COLOR_R.size();
            objs_temp_[j].color_r = COLOR_R[number_ % num_c];
            objs_temp_[j].color_g = COLOR_G[number_ % num_c];
            objs_temp_[j].color_b = COLOR_B[number_ % num_c];
            
            
            objs_tracked_.push_back(objs_temp_[j]);
        }
    }
    
    objs_temp_.clear();
    
    for(int j = 0; j< objs_observed_.size(); j++)
    {
        if(objs_label_[j] == 0)
        {
            objs_temp_.push_back(objs_observed_[j]);
        }
    }
    
    #pragma omp for
    for(int j = 0; j < objs_temp_.size(); j++)
    {
        double x = objs_temp_[j].xref;
        double y = objs_temp_[j].yref;
        objs_temp_[j].tracker.init(time_interval_, x, 0, y, 0,
                                   sigma_ax_, sigma_ay_, sigma_ox_, sigma_oy_);
    }
}

void MTT::publishMarkers(visualization_msgs::MarkerArray& markers)
{
    std_msgs::Header header;
    header.stamp = ros::Time::now();
    header.frame_id = frame_id_;

    /**************************************/
    auto it = std::find_if(objs_tracked_.begin(), objs_tracked_.end(),  
                            [&](const Object& obj) { return obj.number == current_tracked_id_; });
    int index = std::distance(objs_tracked_.begin(), it);                 
    /****************************************/
    
    for(int i = 0; i < objs_tracked_.size(); i++)
    {
        
        /****************************************/
        if(i!=index)
            continue;
        /****************************************/
        
        visualization_msgs::Marker mar;
        mar.header = header;

        // 设置该标记的命名空间和ID，ID应该是独一无二的
        // 具有相同命名空间和ID的标记将会覆盖
        mar.ns = "people";
        mar.id = objs_tracked_[i].number;

        // 设置标记类型
        mar.type = visualization_msgs::Marker::CUBE;
        
        // 设置标记行为，ADD为添加，DELETE为删除
        mar.action = visualization_msgs::Marker::ADD;

        // 设置标记位姿
        mar.pose.position.x = objs_tracked_[i].x0;
        mar.pose.position.y = objs_tracked_[i].y0;
        mar.pose.position.z = objs_tracked_[i].z0;
        mar.pose.orientation.w = cos(0.5 * objs_tracked_[i].phi);
        mar.pose.orientation.x = 0;
        mar.pose.orientation.y = 0;
        mar.pose.orientation.z = sin(0.5 * objs_tracked_[i].phi);

        // 设置标记尺寸
        mar.scale.x = objs_tracked_[i].l;
        mar.scale.y = objs_tracked_[i].w;
        mar.scale.z = objs_tracked_[i].h;

        // 设置标记颜色，应确保不透明度alpha非零
        mar.color.r = (float) objs_tracked_[i].color_r / 255;
        mar.color.g = (float) objs_tracked_[i].color_g / 255;
        mar.color.b = (float) objs_tracked_[i].color_b / 255;
        mar.color.a = 0.85;

        // 设置标记生存时间，单位为s
        mar.lifetime = ros::Duration(0.5);
        
        //////////////////////////
        mar.text = "ID:" + std::to_string(objs_tracked_[i].number);

        markers.markers.push_back(mar);
    }

    pub_.publish(markers);
}

/*
void MTT::publishObstacles(perception_msgs::ObstacleArray& obstacles)
{
    std_msgs::Header header;
    header.stamp = ros::Time::now();
    header.frame_id = frame_id_;
    
    for(int i = 0; i < objs_tracked_.size(); i++)
    {
        
        
        perception_msgs::Obstacle obs;
        obs.header = header;

        // 设置命名空间和ID，ID应该是独一无二的
        obs.ns = "obstacle";
        obs.id = objs_tracked_[i].number;

        // 设置位姿
        obs.pose.position.x = objs_tracked_[i].x0;
        obs.pose.position.y = objs_tracked_[i].y0;
        obs.pose.position.z = objs_tracked_[i].z0;
        obs.pose.orientation.x = 0;
        obs.pose.orientation.y = 0;
        obs.pose.orientation.z = sin(0.5 * objs_tracked_[i].phi);
        obs.pose.orientation.w = cos(0.5 * objs_tracked_[i].phi);

        // 设置尺寸
        obs.scale.x = objs_tracked_[i].l;
        obs.scale.y = objs_tracked_[i].w;
        obs.scale.z = objs_tracked_[i].h;

        // 设置速度
        obs.v_validity = true;
        obs.vx = objs_tracked_[i].vx;
        obs.vy = objs_tracked_[i].vy;
        obs.vz = 0;
        
        // 设置加速度
        obs.a_validity = false;
        obs.ax = 0;
        obs.ay = 0;
        obs.az = 0;

        obstacles.obstacles.push_back(obs);
    }

    pub_obstacle_array_.publish(obstacles);
}
*/

void MTT::callback(const visualization_msgs::MarkerArray& markers_in)
{
    ros::Time time_start = ros::Time::now();

    observe(markers_in);
    update();   // 从已跟踪的标记筐中继续对比，加入已跟踪类别中
    augment();  // 从上一帧的标记筐中对比，加入已跟踪类别中

    /********************/
    updateClosestObject();
    
    visualization_msgs::MarkerArray markers_out;
    publishMarkers(markers_out);

    // perception_msgs::ObstacleArray obstacles;
    // publishObstacles(obstacles);


    if(show_objects_num_ || show_time_)
    {
        std::cout << "" << std::endl;
        std::cout << "[multiple_target_tracking]" << std::endl;
    }
    
    if(show_objects_num_)
    {
        std::cout << "Size of observed objects: " << markers_in.markers.size() << std::endl;
        std::cout << "Size of tracked objects: " << markers_out.markers.size() << std::endl;
    }
    
    /*----------------------修改部分--------------------*/
    double min_dis = DBL_MAX;
    size_t index = 0; // 发布最近的marker的相对坐标
    for(size_t p = 0; p < markers_out.markers.size(); p++) // 实际上markers_out.size()为1
    {
        double x = markers_out.markers[p].pose.position.x;
        double y = markers_out.markers[p].pose.position.y;
        double distance =  x*x + y*y;
        if(distance < min_dis)
        {
            min_dis = distance;
            index = p;
        }
    }

    //
        geometry_msgs::Point point_msg;
        if(markers_out.markers.size()>0)
        {
            point_msg.x = objs_tracked_[index].x0+0.5;  // 传感器安装偏差
            point_msg.y = objs_tracked_[index].y0;
            //point_msg.z = objs_tracked_[index].z0+0.3;

            std::cout << "The index_"<< index <<" position: " << point_msg.x <<","<< point_msg.y << std::endl;
        }
        //point_msg.z = (double)markers_out.markers.size();
        pub_position_.publish(point_msg);
    

/*
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);
    std::string  target_name = "livox_frame";
    std::string  source_name = "base_footprint";
    geometry_msgs::PointStamped point;
    point.header.frame_id = target_name;
    point.header.stamp = ros::Time();
    point.point.x=0;
    point.point.y=0;
    point.point.z=0;
    //geometry_msgs::TransformStamped transformStamped = tf_buffer.lookupTransform(target_name, source_name, ros::Time(0));
    //geometry_msgs::Vector3 translation = transformStamped.transform.translation;
    //ROS_INFO("x=%f,y=%f,z=%f",translation.x, translation.y, translation.z);
    geometry_msgs::PointStamped point_base;
    point_base = tf_buffer.transform(point, source_name);
    //std::out << "x="<<translation
    if(markers_out.markers.size()>0)
    {
        geometry_msgs::Point point_msg;
        
        point_msg.x = objs_tracked_[index].x0 + point_base.point.x;
        point_msg.y = objs_tracked_[index].y0 + point_base.point.y;
        point_msg.z = objs_tracked_[index].z0 + point_base.point.z;
        std::cout << "The index_"<< index <<" position: " << point_msg.x <<","<< point_msg.y << std::endl;
        pub_position_.publish(point_msg);
    }
    */
    
    
    
    ros::Time time_end = ros::Time::now();

    if(show_time_)
    {
        std::cout << "Time cost per frame: " << time_end - time_start << "s" << std::endl;
    }
    
    
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "multiple_target_tracking");
    ros::NodeHandle nh("~");

    omp_set_num_threads(4);

    MTT mtt(nh);
    return 0;
}

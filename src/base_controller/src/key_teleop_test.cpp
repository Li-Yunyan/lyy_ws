#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>

class SubscribeAndPublish
{
public:
    SubscribeAndPublish()
    {
        //Topic you want to publish
        pub_ = n_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
      
        //Topic you want to subscribe
        //注意这里，和平时使用回调函数不一样了。
        sub_ = n_.subscribe("/turtle1/cmd_vel", 10, &SubscribeAndPublish::callback, this);
    }
    void callback(const geometry_msgs::Twist& input)
    {
        ROS_INFO("%.2f, %.2f\r\n", input.linear.x,input.angular.z);
        pub_.publish(input);
    }

    private:
      ros::NodeHandle n_;
      ros::Publisher  pub_;
      ros::Subscriber sub_;
};

int main(int argc, char** argv){
    ros::init(argc, argv, "key_teleop_test");
    SubscribeAndPublish SAPObject;
    
    ros::spin();
    return 0;
}

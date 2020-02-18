#include <ros/ros.h>
#include <sensor_msgs/BatteryState.h>
#include <std_msgs/String.h>

std::string old_message = "";

void batteryCallback(const sensor_msgs::BatteryState::ConstPtr& msg)
{
    int middle = 50;
    int low = 20;
    int empty = 10;
    std::string message = "";
    std::string location = "";
    int percentage;

    if (msg->location == "hero2")
    {
        percentage = msg->percentage;
        location = msg->location
        ROS_INFO(percentage)

        // check battery status
        if (percentage < empty)
        {
            message = "The battery on " + location + " is at " std::to_string(percentage) + "percent. Charge me now!";
        }
        else if (percentage < low)
        {
            message = "The battery on " + location + " is at " std::to_string(percentage) + "percent. Charge me as soon as possible";
        }
        else if (percentage < middle)
        {
            message = "The battery on " + location + " is at " std::to_string(percentage) + "percent. Keep an eye on the battery";
        }

        // publish voice message
        if (old_message != message && message != "")
        {
            std_msgs::String speech_msg;
            speech_msg.data = message;
            speech_pub.publish(speech_msg);
            old_message = message;
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Battery_manager_laptop");
    ros::NodeHandle gn;

    ros::Publisher speech_pub = gn.advertise<std_msgs::String>("text_to_speech/input", 10);
    ros::Subscriber battery_sub = gn.subscribe("battery", 1, batteryCallback);

    ros::Rate loop_rate(1.0);

    while(gn.ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}


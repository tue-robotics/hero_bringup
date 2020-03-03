#include <ros/ros.h>
#include <sensor_msgs/BatteryState.h>
#include <std_msgs/String.h>
#include <string>
#include <sstream>


std::string old_message = "";
ros::Publisher speech_pub;

void batteryCallback(const sensor_msgs::BatteryState::ConstPtr& msg)
{
    int percentage;
    int middle;
    int low;
    int empty;
    ros::param::get("/battery_middle", middle);
    ros::param::get("/battery_low", low);
    ros::param::get("/battery_empty", empty);
    std::string message = "";
    std::string location = "";
    std::ostringstream string_message;

    if (msg->location == "hero2")
    {
        percentage = (msg->percentage)*100;
        location = msg->location;

        // check battery status
        if (percentage < empty)
        {
            string_message << "The battery on" << location <<  " is at"  << percentage << " percent. Charge me now!";
        }
        else if (percentage < low)
        {
            string_message << "The battery on" << location <<  " is at"  << percentage << " percent. Don't forget to charge me";
        }
        else if (percentage < middle)
        {
            string_message << "The battery on" << location <<  " is at"  << percentage << " percent. Keep an I on the batteries";
        }

        message = string_message.str();

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

    ros::Subscriber battery_sub = gn.subscribe("battery", 1, batteryCallback);
    speech_pub = gn.advertise<std_msgs::String>("text_to_speech/input", 10);

    ros::Rate loop_rate(1.0);

    while(gn.ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}


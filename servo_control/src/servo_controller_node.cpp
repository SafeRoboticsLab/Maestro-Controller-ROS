#include <ros/ros.h>
#include <ros/console.h>

#include <signal.h>
#include <servo_controller.h>

void signalHandler(int sig) {
    ROS_INFO("Controller shutdown complete. Calling ros shutdown.");
    ros::shutdown();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "servo_controller_node", ros::init_options::NoSigintHandler+ros::init_options::AnonymousName

);
    ros::NodeHandle node;    
    ros::NodeHandle private_nh("~");
    signal(SIGINT, signalHandler);

    ServoController controller(node, private_nh);

    double freq;
    private_nh.getParam("frequency", freq);

    const ros::Duration dt(1.0/freq);
    ros::Rate rate(freq);

    // start the driver
    ros::AsyncSpinner spinner(1);
    spinner.start();
    while(ros::ok())
    {
        controller.update(ros::Time::now(), dt);
        rate.sleep();
    }
    spinner.stop();
    
    return 0;
}

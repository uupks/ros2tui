
#include "ros2tui/ros2tui.h"

int main(int argc, char const *argv[])
{
    (void)argc;
    (void)argv;
    ros2tui::ROS2Tui ros2tui;
    ros2tui.run();
    return 0;
}

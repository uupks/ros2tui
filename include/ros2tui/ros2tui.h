#include <memory>

namespace ros2tui {

class ROS2TuiImpl;

class ROS2Tui
{
public:
    ROS2Tui();
    ~ROS2Tui();

    void run();

private:
    std::unique_ptr<ROS2TuiImpl> pimpl_;
};

}

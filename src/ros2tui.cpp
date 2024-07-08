#include "ftxui/component/screen_interactive.hpp"  // for Component, ScreenInteractive
#include "ftxui/component/component.hpp"  // for Checkbox, Renderer, Horizontal, Vertical, Input, Menu, Radiobox, ResizableSplitLeft, Tab
#include "ftxui/component/component_base.hpp"  // for ComponentBase, Component
#include "ftxui/component/component_options.hpp"  // for MenuOption, InputOption
#include "ftxui/component/event.hpp"              // for Event, Event::Custom
#include "ftxui/component/loop.hpp"       // for Loop

#include "ros2tui/ros2tui.h"
#include "rclcpp/rclcpp.hpp"

#include "yaml-cpp/yaml.h"
#include "dynmsg/message_reading.hpp"
#include "dynmsg/msg_parser.hpp"
#include "dynmsg/typesupport.hpp"
#include "dynmsg/yaml_utils.hpp"

#include "spdlog/fmt/fmt.h"

namespace ros2tui {

class TopicMeta
{
public:
    TopicMeta() {}

    TopicMeta(const std::string& topic_name, const std::string& topic_type)
        :   topic_name_(topic_name), 
            topic_type_(topic_type), 
            allocator_(rcutils_get_default_allocator()),
            first_message_(true),
            msg_count_(-1),
            first_msg_ts_(0),
            cur_msg_ts_(0),
            fps_(0.0)
    { 

    }

    ~TopicMeta() {
        sub_.reset();
        if (ros_cppmsg_.data != nullptr) {
            dynmsg::cpp::ros_message_destroy_with_allocator(&ros_cppmsg_, &allocator_);
        }
    }

public:
    std::string topic_name_;
    std::string topic_type_;
    RosMessage_Cpp ros_cppmsg_;
    rcutils_allocator_t allocator_;
    rclcpp::GenericSubscription::SharedPtr sub_;

    // for static
    bool first_message_;
    int64_t msg_count_;
    uint64_t first_msg_ts_;
    uint64_t cur_msg_ts_;
    float fps_;
};


class ROS2TuiImpl
{
public:
    ROS2TuiImpl(/* args */);
    ~ROS2TuiImpl();

    void run();

    void destroy();

private:
    void create_node_monitor();
    void create_topic_monitor();
    void create_service_monitor();

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::CallbackGroup::SharedPtr callback_group_;
};

ROS2TuiImpl::ROS2TuiImpl()
{
    rclcpp::InitOptions init_options;
    init_options.auto_initialize_logging(false);

    rclcpp::init(0, 0, init_options);

    rclcpp::NodeOptions node_options;
    node_options.start_parameter_services(false);
    node_options.start_parameter_event_publisher(false);

    node_ = std::make_shared<rclcpp::Node>("ros2tui", node_options);

    callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);

    std::thread spin_thread([this]() {
        rclcpp::executors::StaticSingleThreadedExecutor exec;
        exec.add_node(node_);
        exec.spin();
    });

    spin_thread.detach();
}

ROS2TuiImpl::~ROS2TuiImpl()
{
    rclcpp::shutdown();
}

void ROS2TuiImpl::run()
{
    using namespace ftxui;
    auto screen = ScreenInteractive::FitComponent();
    auto btn_nodes_monitor = Button("Node Monitor", [this] () { this->create_node_monitor(); });
    auto btn_topic_monitor = Button("Topic Monitor", [this] () { this->create_topic_monitor(); });
    auto btn_service_monitor = Button("Service Monitor", [this] () { this->create_service_monitor(); });
    auto btn_quit = Button("Exit", screen.ExitLoopClosure());

    auto home_component = Container::Vertical({
        btn_nodes_monitor,
        btn_topic_monitor,
        btn_service_monitor,
        btn_quit
    });

    screen.Loop(home_component);
}

void ROS2TuiImpl::destroy()
{
    ftxui::ScreenInteractive::Active()->Exit();
}

void ROS2TuiImpl::create_node_monitor()
{
    using namespace ftxui;
    auto screen = ScreenInteractive::Fullscreen();
    screen.TrackMouse(false);

    std::vector<std::string> node_names;
    auto graph_interface = node_->get_node_graph_interface();
    
    auto update_node_list = [&] () {
        auto nodes = graph_interface->get_node_names();
        nodes.insert(nodes.begin(), "None");
        node_names.swap(nodes);
    };

    auto update_node_info = [&] (const std::string& node_name) {
        Elements paragraph;
        if (node_name.empty()) {
            return vflow(paragraph);
        }

        auto node_name_with_ns = graph_interface->get_node_names_and_namespaces();
        for (const auto& n : node_name_with_ns) {
            std::string name_with_ns;
            if (n.second.size() == 1) {
                name_with_ns = "/" + n.first;
            } else {
                name_with_ns = n.second + "/" + n.first;
            }
            if (name_with_ns == node_name) {
                // Publishers
                auto pubs = graph_interface->get_publisher_names_and_types_by_node(n.first, n.second);
                paragraph.push_back(ftxui::paragraph("Publishers:"));
                for (const auto& p : pubs) {
                    paragraph.push_back(ftxui::paragraph(fmt::format("  {}: {}", p.first, p.second[0])));
                }
                paragraph.push_back(ftxui::text(""));
                
                // Subscribers
                auto subs = graph_interface->get_subscriber_names_and_types_by_node(n.first, n.second);
                paragraph.push_back(ftxui::paragraph("Subscribers:"));
                for (const auto& s : subs) {
                    paragraph.push_back(ftxui::paragraph(fmt::format("  {}: {}", s.first, s.second[0])));
                }
                paragraph.push_back(ftxui::text(""));
                
                // Service Servers
                auto sss = graph_interface->get_service_names_and_types_by_node(n.first, n.second);
                paragraph.push_back(ftxui::paragraph("Service Servers:"));
                for (const auto& ss : sss) {
                    paragraph.push_back(ftxui::paragraph(fmt::format("  {}: {}", ss.first, ss.second[0])));
                }
                paragraph.push_back(ftxui::text(""));

                // Service Clients
                auto scs = graph_interface->get_client_names_and_types_by_node(n.first, n.second);
                paragraph.push_back(ftxui::paragraph("Service Clients:"));
                for (const auto& sc : scs) {
                    paragraph.push_back(ftxui::paragraph(fmt::format("  {}: {}", sc.first, sc.second[0])));
                }
                paragraph.push_back(ftxui::text(""));
            }
        }
        return vflow(paragraph);
    };

    update_node_list();

    int selected = -1;
    int focused_entry = 0;
    std::string node_name = "";
    RadioboxOption options;
    options.focused_entry = &focused_entry;
    options.on_change = [&] {
        if (selected == focused_entry && focused_entry > 0) {
            node_name = node_names[selected];
            update_node_info(node_name);
        } else if (selected == 0) {
            node_name = "";
            update_node_info(node_name);
        }
    };

    auto list_radiobox = Radiobox(&node_names, &selected, options);

    auto node_info_win = Renderer([&] {
        return update_node_info(node_name);
    });

    auto main_container = Container::Horizontal({
        list_radiobox,
        node_info_win,
    });

    main_container |= CatchEvent([&](Event event) {
        if (event == Event::Escape) {
            screen.Exit();
            return true;
        } else {
            return false;
        }
    });

    auto main_renderer = Renderer(main_container, [&] {
        return vbox({
                text("Node Monitor") | bold | hcenter,
                hbox({
                    list_radiobox->Render() | border | notflex,
                    node_info_win->Render() | border | notflex,
                }),
        });
    });

    Loop loop(&screen, main_renderer);
    int custom_loop_count = 0;
    while (!loop.HasQuitted()) {
        custom_loop_count++;

        if (custom_loop_count % 20 == 0) { 
            update_node_list();
        }
        
        screen.PostEvent(Event::Custom);
        loop.RunOnce();
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

void ROS2TuiImpl::create_topic_monitor()
{
    using namespace ftxui;
    auto screen = ScreenInteractive::Fullscreen();
    screen.TrackMouse(false);

    std::string topic_name = "None";
    std::string topic_type = "None";
    std::string debug_str = "";

    std::vector<std::string> topic_list;
    std::map<std::string, std::vector<std::string>> topic_list_with_type;
    std::map<std::string, std::shared_ptr<TopicMeta>> topic_manager;

    auto update_topic_list = [&] () {
        topic_list_with_type = node_->get_topic_names_and_types();
        topic_list.clear();
        topic_list.push_back("None");
        for (const auto& t : topic_list_with_type) {
            topic_list.push_back(t.first);

            std::string type_string;
            for (const auto& type : t.second) {
                type_string += fmt::format("{}  ", type);
            }
        }
    };

    auto create_generic_sub2 = [&] (const std::string& topic_name, const std::string& topic_type) {

        auto get_topic_type_from_string_type = [&] (const std::string & type) {
            std::string::size_type split_first = type.find_first_of('/');
            std::string::size_type split_last = type.find_last_of('/');
            if (split_first == std::string::npos) {
                throw std::runtime_error("invalid type specification");
            }
            if (split_last == std::string::npos) {
                throw std::runtime_error("invalid type specification");
            }
            return InterfaceTypeName(type.substr(0, split_first), type.substr(split_last + 1));
        };

        if (topic_manager.find(topic_name) != topic_manager.end()) return;

        topic_manager[topic_name] = std::make_shared<TopicMeta>(topic_name, topic_type);

        auto subscription_options = rclcpp::SubscriptionOptions();
        subscription_options.callback_group = callback_group_;

        topic_manager[topic_name]->sub_ = node_->create_generic_subscription(
            topic_name, 
            topic_type, 
            rclcpp::SensorDataQoS().keep_last(50).durability_volatile(),
            [this](std::shared_ptr<const rclcpp::SerializedMessage> message) { (void)message; },
            subscription_options
        );

        auto& ros_cppmsg = topic_manager[topic_name]->ros_cppmsg_;
        InterfaceTypeName interface = get_topic_type_from_string_type(topic_type);
        ros_cppmsg.type_info = dynmsg::cpp::get_type_info(interface);
        dynmsg::cpp::ros_message_with_typeinfo_init(ros_cppmsg.type_info, &ros_cppmsg, &topic_manager[topic_name]->allocator_);
    };

    update_topic_list();

    int selected = 0;
    int focused_entry = 0;

    RadioboxOption options;
    options.focused_entry = &focused_entry;
    options.on_change = [&] () {
        if (selected == focused_entry && focused_entry > 0) {
            // topic name
            topic_name = topic_list[focused_entry];
            // topic type, use the first one when there are multiplt type 
            auto it = topic_list_with_type.find(topic_name);
            if (it != topic_list_with_type.end()) {
                topic_type = it->second[0];
                create_generic_sub2(topic_name, topic_type);
            }
        } else if (selected == 0) {
            topic_name = "None";
            topic_type = "None";
        }
    };

    auto list_radiobox = Radiobox(&topic_list, &selected, options);

    auto echo_renderer = Renderer([&] {
        ftxui::Elements paragraph;
        if (topic_manager.find(topic_name) == topic_manager.end()) {
            return vflow(paragraph);
        }

        paragraph.push_back(ftxui::paragraph(topic_name + " @ [" + topic_type + "]"));
        paragraph.push_back(ftxui::text(""));

        rclcpp::MessageInfo msg_info;
        auto& meta = topic_manager[topic_name];

        // continues to take messages from the subscription queue until the queue is empty
        while (topic_manager[topic_name]->sub_->take_type_erased(meta->ros_cppmsg_.data, msg_info)) {
            if (meta->first_message_) {
                meta->first_msg_ts_ = msg_info.get_rmw_message_info().source_timestamp;
                meta->first_message_ = false;
            }
            meta->cur_msg_ts_ = msg_info.get_rmw_message_info().source_timestamp;
            meta->msg_count_++;
        }

        auto time_window = (meta->cur_msg_ts_ - meta->first_msg_ts_) / 1e9;

        if (time_window > 0) {
            meta->fps_ = (double)meta->msg_count_ / (double)time_window;
        }

        if (0) {
            paragraph.push_back(ftxui::text(fmt::format("first source_timestamp: {}\n", meta->first_msg_ts_)));
            paragraph.push_back(ftxui::text(fmt::format("source_timestamp: {}\n", meta->cur_msg_ts_)));
            paragraph.push_back(ftxui::text(fmt::format("time window: {}\n", time_window)));
            paragraph.push_back(ftxui::text(fmt::format("count: {}\n", meta->msg_count_)));
        }

        paragraph.push_back(ftxui::text(fmt::format("Hz: {:.1f}\n", meta->fps_)));
        paragraph.push_back(ftxui::text(""));

        // reset every 5 seconds
        if (time_window > 5.0) {
            meta->first_message_ = true;
            meta->msg_count_ = -1;
            meta->first_msg_ts_ = 0;
            meta->cur_msg_ts_ = 0;
        }

        // only echo the latest message
        if (meta->ros_cppmsg_.data != nullptr) {
            YAML::Node yaml_msg = dynmsg::cpp::message_to_yaml(meta->ros_cppmsg_);
            auto yaml_string = dynmsg::yaml_to_string(yaml_msg);
            std::stringstream ss(yaml_string);
            std::string line;
            std::vector<std::string> lines;
            while (std::getline(ss, line, '\n')) {
                paragraph.push_back(ftxui::paragraph(line));
            }
        }
        return vflow(paragraph);
    });

    auto info_renderer = Renderer([&] {
        ftxui::Elements paragraph;
        auto pubs = node_->get_publishers_info_by_topic(topic_name);
        auto subs = node_->get_subscriptions_info_by_topic(topic_name);
        paragraph.push_back(ftxui::paragraph(fmt::format("Type : {}\n", topic_type)));
        paragraph.push_back(ftxui::text(""));
        paragraph.push_back(ftxui::paragraph(fmt::format("Publisher count: {}\n", pubs.size())));
        paragraph.push_back(ftxui::text(""));
        
        for (const auto& p : pubs) {
            auto rmw_qos_profile = p.qos_profile().get_rmw_qos_profile();

            paragraph.push_back(ftxui::paragraph(fmt::format("Node name: {}\n", p.node_name())));
            paragraph.push_back(ftxui::paragraph(fmt::format("QoS Profile:\n")));
            paragraph.push_back(ftxui::paragraph(fmt::format("  Reliability: {}\n", rmw_qos_reliability_policy_to_str(rmw_qos_profile.reliability))));
            paragraph.push_back(ftxui::paragraph(fmt::format("  History(Depth): {}({})\n", rmw_qos_history_policy_to_str(rmw_qos_profile.history), rmw_qos_profile.depth)));
            paragraph.push_back(ftxui::paragraph(fmt::format("  Durability: {}\n", rmw_qos_durability_policy_to_str(rmw_qos_profile.durability))));
            paragraph.push_back(ftxui::text(""));
        }

        paragraph.push_back(ftxui::text(""));
        paragraph.push_back(ftxui::paragraph(fmt::format("Subscription count: {}\n", subs.size())));
        paragraph.push_back(ftxui::text(""));
        for (const auto& s : subs) {
            auto rmw_qos_profile = s.qos_profile().get_rmw_qos_profile();
            paragraph.push_back(ftxui::paragraph(fmt::format("Node name: {}\n", s.node_name())));
            paragraph.push_back(ftxui::paragraph(fmt::format("QoS Profile:\n")));
            paragraph.push_back(ftxui::paragraph(fmt::format("  Reliability: {}\n", rmw_qos_reliability_policy_to_str(rmw_qos_profile.reliability))));
            paragraph.push_back(ftxui::paragraph(fmt::format("  History(Depth): {}({})\n", rmw_qos_history_policy_to_str(rmw_qos_profile.history), rmw_qos_profile.depth)));
            paragraph.push_back(ftxui::paragraph(fmt::format("  Durability: {}\n", rmw_qos_durability_policy_to_str(rmw_qos_profile.durability))));
            paragraph.push_back(ftxui::text(""));
        }
        return vflow(paragraph);
    });

    auto pub_renderer = Renderer([&] {
        return text("Pub");
    });

    int tab_index = 0;
    std::vector<std::string> tab_entries = {
        "echo", "info", "pub"
    };

    auto tab_selection = Menu(&tab_entries, &tab_index, MenuOption::HorizontalAnimated());

    auto tab_content = Container::Tab(
        {
            echo_renderer,
            info_renderer,
            pub_renderer,
        },
        &tab_index);

    auto main_container = Container::Horizontal({
        list_radiobox,
        Container::Vertical({
            tab_selection,
            tab_content,
        }),
    });

    main_container |= CatchEvent([&](Event event) {
        if (event == Event::Escape) {
            screen.Exit();
            return true;
        } else {
            return false;
        }
    });

    auto main_renderer = Renderer(main_container, [&] {
        return vbox({
                text("Topic Monitor") | bold | hcenter,
                hbox({
                    list_radiobox->Render() | border | notflex,
                    vbox({
                        tab_selection->Render() | notflex,
                        tab_content->Render() | notflex,
                    }) | border | notflex,
                }),
        });
    });

    Loop loop(&screen, main_renderer);
    int custom_loop_count = 0;
    while (!loop.HasQuitted()) {
        custom_loop_count++;

        if (custom_loop_count % 20 == 0) { 
            update_topic_list();
        }
        
        screen.PostEvent(Event::Custom);
        loop.RunOnce();
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

void ROS2TuiImpl::create_service_monitor()
{
    using namespace ftxui;
    auto screen = ScreenInteractive::FitComponent();
    screen.TrackMouse(false);
    
    bool is_support_service_monitor = false;
    auto ros_distro = std::getenv("ROS_DISTRO");
    if (ros_distro != nullptr && std::string(ros_distro) == std::string("jazzy")) {
        is_support_service_monitor = true;
    }

    auto btn_quit = Button("Back", screen.ExitLoopClosure());

    auto warning_text = Renderer([&] {
        if (is_support_service_monitor) {
            return text("Service Monitor is not Implemented");
        } else {
            return text("Service Monitor is only supported in ROS Jazzy");
        }
    });

    auto main_container = Container::Vertical({
        warning_text | bold | hcenter | border,
        btn_quit | border | hcenter,
    });

    screen.Loop(main_container);
}

//////////////////////
// ROS2Tui public interface

ROS2Tui::ROS2Tui()
{
    pimpl_ = std::make_unique<ROS2TuiImpl>();
}

ROS2Tui::~ROS2Tui()
{
    pimpl_.reset();
}

void ROS2Tui::run()
{
    pimpl_->run();
}

}

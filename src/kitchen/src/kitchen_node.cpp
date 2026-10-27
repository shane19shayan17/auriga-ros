#include "rclcpp/rclcpp.hpp"
#include "interface/msg/robot.hpp"
#include "interface/srv/food_ready.hpp"
#include "std_srvs/srv/empty.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <memory>
#include <string>
#include <deque>
#include <cstdint>

#include "../include/kitchen/Logger.hpp"
#include "../include/kitchen/Menu.hpp"

using Odometry = nav_msgs::msg::Odometry;

class KithcenHandler
{
public:
    KithcenHandler(int period_ms)
        : period_second_(period_ms / 1000.0)
    {
        kitchen_odom_.header.frame_id = "baase_link";
        kitchen_odom_.pose.pose.position.x = 0;
        kitchen_odom_.pose.pose.position.y = 0;
        kitchen_odom_.pose.pose.orientation.w = 1;
    }

    struct OrderInfo
    {
        OrderInfo() = default;
        OrderInfo(const Food &food_)
        {
            food = food_;
            remaining_time = food_.preparationTime;
            is_ready = false;
        }
        Food food;
        bool is_ready;
        float remaining_time;
    };

    enum State
    {
        FREE = 1,
        COMMING_BACK = 2,
        DELIVERING = 3,
    };
    

    void run()
    {
        has_ready_food = false;

        for (auto &o : orders)
        {
            if (o.remaining_time <= 0)
            {
                o.is_ready = true;
            }
            o.remaining_time -= period_second_;
        }

        if (robot_state_ == State::FREE)
        {
            for (size_t i = 0; i < orders.size(); i++)
            {
                if (orders.at(i).is_ready)
                {
                    has_ready_food = true;
                    ready_food_info = orders.at(i);
                    orders.erase(orders.begin() + i);
                    break;
                }
            }
        }

        if (menu.empty())
        {
            is_close = true;
        }
    }

    void set_robot_state(int16_t state)
    {
        robot_state_ = state;
    }

    int16_t get_robot_state()
    {
        return robot_state_;
    }

    bool get_order(std::string name)
    {
        for (size_t i = 0; i < menu.size(); i++)
        {
            auto m = menu.at(i);
            if (m.name == name && m.remaining > 0)
            {
                LOG_INFO("kitchen", "%s added to kitchen's que !", m.name.c_str());
                orders.push_back(menu.at(i)); // add food to orders que
                menu.at(i).remaining--;       // one of stock was used
                if (menu.at(i).remaining <= 0)
                {
                    LOG_INFO("kitchen", "we are out of %s from now!", m.name.c_str());
                    menu.erase(menu.begin() + i);
                }
                return true; // response was successful
            }
        }
        LOG_INFO("kitchen", "sory robot we can not provide this order !!!");
        return false;
    }

    OrderInfo get_ready_food_info()
    {
        return ready_food_info;
    }

    bool get_has_food_ready()
    {
        return has_ready_food;
    }

    bool get_is_close()
    {
        return is_close;
    }

    Odometry get_kitchen_odometry()
    {
        return kitchen_odom_;
    }

private:
    int16_t robot_state_;

    std::deque<OrderInfo> orders = {};
    float period_second_;

    bool has_ready_food = false;
    OrderInfo ready_food_info;

    bool is_close = false;

    Odometry kitchen_odom_ = Odometry();
};

using std::placeholders::_1;
using std::placeholders::_2;
using Robot = interface::msg::Robot;
using FoodReady = interface::srv::FoodReady;
using Empty = std_srvs::srv::Empty;

class KitchenNode : public rclcpp::Node
{
public:
    KitchenNode()
        : Node("kitchen_node")
    {
        // robot state subscriber
        robot_state_subscriber_ = this->create_subscription<Robot>(
            "robot_state",
            10,
            std::bind(&KitchenNode::robot_state_callback, this, _1));

        // timer
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(timer_period_ms),
            std::bind(&KitchenNode::timer_callback, this));

        food_ready_client_ = this->create_client<FoodReady>(
            "food_ready");

        closing_client_ = this->create_client<Empty>(
            "closing");

        auto keep_last_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
        kitchen_position_publisher_ = this->create_publisher<Odometry>(
            "/kitchen_position",
            keep_last_qos);

        kitchen_position_publisher_->publish(kithcenHandler.get_kitchen_odometry());
    }

private:
    void timer_callback()
    {
        kithcenHandler.run();

        if (kithcenHandler.get_has_food_ready())
        {
            send_food_ready_request(kithcenHandler.get_ready_food_info().food.name);
        }

        if (kithcenHandler.get_is_close())
        {
            send_closing_request();
        }
    }

    void send_food_ready_request(std::string name)
    {
        if (!food_ready_client_->service_is_ready())
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for food service...");
            return;
        }

        auto request = std::make_shared<FoodReady::Request>();
        request->name = name;

        food_ready_client_->async_send_request(
            request);
    }

    void send_closing_request()
    {
        if (!closing_client_->service_is_ready())
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for closing service...");
            return;
        }

        auto request = std::make_shared<Empty::Request>();

        closing_client_->async_send_request(
            request);
    }

    void robot_state_callback(const Robot::SharedPtr msg)
    {
        kithcenHandler.set_robot_state(msg->robot_state);
    }

    rclcpp::Subscription<Robot>::SharedPtr robot_state_subscriber_;
    rclcpp::Client<FoodReady>::SharedPtr food_ready_client_;
    rclcpp::Client<Empty>::SharedPtr closing_client_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<Odometry>::SharedPtr kitchen_position_publisher_;
    int timer_period_ms = 200;

    // main controller for kitchen
    KithcenHandler kithcenHandler{timer_period_ms};
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KitchenNode>());
    rclcpp::shutdown();
    return 0;
}
#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "interface/srv/order.hpp"
#include "std_srvs/srv/empty.hpp"
#include "interface/msg/matrix2_d.hpp"

#include "../../kitchen/include/kitchen/Menu.hpp"

#include <vector>
#include <random>

using namespace std::chrono_literals;

using Order = interface::srv::Order;
using Empty = std_srvs::srv::Empty;
using Matrix2D = interface::msg::Matrix2D;

using std::placeholders::_1;
using std::placeholders::_2;

class CustomerNode : public rclcpp::Node
{
public:
  CustomerNode()
      : Node("customer_node")
  {
    // configuration
    srand(time(0));

    order_client_ = this->create_client<Order>(
        "order");

    timer_ = this->create_wall_timer(
        std::chrono::seconds(5),
        std::bind(&CustomerNode::timer_callback, this));

    closing_service_ = this->create_service<Empty>(
        "closing",
        std::bind(&CustomerNode::closing_service_callback, this, _1, _2));

    auto keep_last_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    tables_pose_subscriber_ = this->create_subscription<Matrix2D>(
        "tables_pose",
        keep_last_qos,
        std::bind(&CustomerNode::tables_pose_callback, this, _1));
  }

  void send_request(Order::Request order)
  {
    if (!order_client_->service_is_ready())
    {
      RCLCPP_WARN(this->get_logger(), "Waiting for order service...");
      return;
    }

    auto request = std::make_shared<Order::Request>(order);
    std::string sent_name = request->name;

    order_client_->async_send_request(
        request,
        [this, sent_name](rclcpp::Client<Order>::SharedFuture future)
        {
          auto response = future.get();
          if (!response->success)
          {
            RCLCPP_INFO(this->get_logger(), "kithen can not provide this order !");
          }
        });
  }

  // change order every loop
  Order::Request get_order()
  {
    Order::Request order = Order::Request();
    auto item = menu.at(order_counter_);
    order_counter_ = (order_counter_ + 1) % menu.size();

    order.name = item.name;

    int table = rand() % tables_pose_.size();
    order.table_pose = {tables_pose_.at(table).first, tables_pose_.at(table).second};

    RCLCPP_INFO(get_logger(), "table (%d, %d) want to order !", order.table_pose[0], order.table_pose[1]);

    return order;
  }

private:
  void timer_callback()
  {
    if (!is_close)
    {
      send_request(get_order());
    }
    else
    {
      RCLCPP_WARN_ONCE(this->get_logger(), "closing the restaurant !!!");
    }
  }

  void closing_service_callback(
      const std::shared_ptr<Empty::Request> request,
      std::shared_ptr<Empty::Response> response)
  {
    (void)request;
    (void)response;
    is_close = true;
  }

  void tables_pose_callback(
      const Matrix2D::SharedPtr msg)
  {
    int rows = msg->rows;

    for (int r = 0; r < rows; r++)
    {
      std::pair<int, int> pose = {msg->poses[r].x, msg->poses[r].y};
      tables_pose_.push_back(pose);
    }
    RCLCPP_INFO(get_logger(), "map recieved. there are %d tables !", rows);
  }

  rclcpp::Client<Order>::SharedPtr order_client_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Service<Empty>::SharedPtr closing_service_;
  rclcpp::Subscription<Matrix2D>::SharedPtr tables_pose_subscriber_;

  int order_counter_ = 0;
  bool is_close = false;
  std::vector<std::pair<int, int>> tables_pose_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  auto customer_node = std::make_shared<CustomerNode>();

  rclcpp::spin(customer_node);

  rclcpp::shutdown();

  return 0;
}
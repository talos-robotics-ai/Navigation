// Type-agnostic rate limiter: for each (in, out, type, hz) it forwards at most `hz` serialized messages per
// second, without deserializing anything. One small C++ process instead of thousands of Python wakeups/s.
//
//   params (parallel lists): inputs, outputs, types, rates   (rate <= 0: forward everything)
//   qos: BEST_EFFORT depth 1 both ways (a BEST_EFFORT reader matches RELIABLE writers too)
#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

class Throttle : public rclcpp::Node {
 public:
  Throttle() : Node("x2_throttle", rclcpp::NodeOptions().enable_rosout(false).start_parameter_services(false)) {
    auto in = declare_parameter<std::vector<std::string>>("inputs", std::vector<std::string>{});
    auto out = declare_parameter<std::vector<std::string>>("outputs", std::vector<std::string>{});
    auto types = declare_parameter<std::vector<std::string>>("types", std::vector<std::string>{});
    auto rates = declare_parameter<std::vector<double>>("rates", std::vector<double>{});
    if (in.size() != out.size() || in.size() != types.size() || in.size() != rates.size()) {
      throw std::runtime_error("inputs/outputs/types/rates must have the same length");
    }
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
    for (size_t i = 0; i < in.size(); ++i) {
      auto ch = std::make_shared<Channel>();
      ch->min_dt = rates[i] > 0.0 ? 1.0 / rates[i] : 0.0;
      ch->pub = create_generic_publisher(out[i], types[i], qos);
      ch->sub = create_generic_subscription(
          in[i], types[i], qos, [this, ch](std::shared_ptr<rclcpp::SerializedMessage> msg) {
            ch->n_in++;
            const double t = now_s();
            if (t - ch->last < ch->min_dt) return;
            ch->last = t;
            ch->n_out++;
            ch->pub->publish(*msg);
          });
      ch->name = in[i] + " -> " + out[i];
      channels_.push_back(ch);
      RCLCPP_INFO(get_logger(), "%s (%s) at <= %.1f Hz", ch->name.c_str(), types[i].c_str(), rates[i]);
    }
    timer_ = create_wall_timer(std::chrono::seconds(5), [this]() {
      std::string s;
      for (auto &c : channels_) {
        s += c->name + " " + std::to_string(c->n_in / 5) + "->" + std::to_string(c->n_out / 5) + " Hz; ";
        c->n_in = c->n_out = 0;
      }
      RCLCPP_INFO(get_logger(), "%s", s.c_str());
    });
  }

 private:
  struct Channel {
    rclcpp::GenericSubscription::SharedPtr sub;
    rclcpp::GenericPublisher::SharedPtr pub;
    double min_dt{0.0}, last{-1e9};
    long n_in{0}, n_out{0};
    std::string name;
  };
  static double now_s() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
  }
  std::vector<std::shared_ptr<Channel>> channels_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Throttle>());
  rclcpp::shutdown();
  return 0;
}

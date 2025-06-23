#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <vector>
#include <queue>
#include <utility>
#include <cstdint>

using std::placeholders::_1;
using namespace std::chrono_literals;

using SCAN = sensor_msgs::msg::LaserScan;
using TWIST = geometry_msgs::msg::Twist;
using OCCUPANCY = nav_msgs::msg::OccupancyGrid;
using POSESTAMPED = geometry_msgs::msg::PoseStamped;

typedef enum {
  FORWARD = 0,
  STOP,
  TURN,
  PATH_FOLLOW
} StateType;

class Robot : public rclcpp::Node {
 public:
    Robot();
 private:
    void scan_callback(const SCAN&);
    void timer_callback();
    void map_callback(const OCCUPANCY&);
    void goal_callback(const POSESTAMPED&);
    bool detect_obstacle();

    // --- A*核心 ---
    bool astar_search(
        const std::vector<int8_t>& map_data, int width, int height,
        std::pair<int, int> start, std::pair<int, int> goal,
        std::vector<std::pair<int, int>>& path);

    bool world_to_map(double wx, double wy, int& mx, int& my);
    bool map_to_world(int mx, int my, double& wx, double& wy);

    rclcpp::Publisher<TWIST>::SharedPtr publisher_;
    rclcpp::Subscription<SCAN>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<OCCUPANCY>::SharedPtr map_sub_;
    rclcpp::Subscription<POSESTAMPED>::SharedPtr goal_sub_;

    SCAN scan_;
    OCCUPANCY map_;
    bool map_received_ = false;
    POSESTAMPED goal_;
    bool goal_received_ = false;

    std::vector<std::pair<int, int>> current_path_;
    size_t path_index_ = 0;

    StateType state_;
};


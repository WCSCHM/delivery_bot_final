#include "../include/Dijkstra.hpp"

#include <cmath>
#include <set>
#include <limits>
#include <algorithm>

Robot::Robot()
    : Node("walker"),
      state_(STOP) {
    auto pubTopicName = "cmd_vel";
    publisher_ = this->create_publisher<TWIST>(pubTopicName, 10);

    auto subTopicName = "/scan";
    subscription_ = this->create_subscription<SCAN>(
        subTopicName, 10, std::bind(&Robot::scan_callback, this, _1));

    // 订阅地图（nav_msgs/OccupancyGrid）和目标点
    map_sub_ = this->create_subscription<OCCUPANCY>(
        "/map", 1, std::bind(&Robot::map_callback, this, _1));
    goal_sub_ = this->create_subscription<POSESTAMPED>(
        "/goal_pose", 1, std::bind(&Robot::goal_callback, this, _1));

    timer_ = this->create_wall_timer(100ms, std::bind(&Robot::timer_callback, this));
}

void Robot::scan_callback(const SCAN& msg) {
    scan_ = msg;
}

void Robot::map_callback(const OCCUPANCY& msg) {
    map_ = msg;
    map_received_ = true;
}

void Robot::goal_callback(const POSESTAMPED& msg) {
    goal_ = msg;
    goal_received_ = true;
    // 重新规划路径
    if (map_received_) {
        int start_x, start_y, goal_x, goal_y;
        // 机器人当前位置（假设odom与map对齐，这里应实际取TF变换后的base_link到map坐标）
        double robot_x = goal_.pose.position.x;  // 实际上应为机器人的当前坐标，此处为演示
        double robot_y = goal_.pose.position.y;
        world_to_map(robot_x, robot_y, start_x, start_y);
        world_to_map(goal_.pose.position.x, goal_.pose.position.y, goal_x, goal_y);

        std::vector<std::pair<int, int>> path;
        bool found = dijkstra_search(
            map_.data, map_.info.width, map_.info.height,
            {start_x, start_y}, {goal_x, goal_y}, path
        );
        if (found) {
            current_path_ = path;
            path_index_ = 0;
            state_ = PATH_FOLLOW;
            RCLCPP_INFO(this->get_logger(), "Dijkstra: Path planned! Size: %ld", path.size());
        } else {
            RCLCPP_WARN(this->get_logger(), "Dijkstra: No path found!");
        }
    }
}

void Robot::timer_callback() {
    auto message = TWIST();
    switch (state_) {
        case FORWARD:
            // 可保留或省略
            break;
        case STOP:
            // 可保留或省略
            break;
        case TURN:
            // 可保留或省略
            break;
        case PATH_FOLLOW:
            if (current_path_.empty()) {
                message.linear.x = 0.0;
                publisher_->publish(message);
                state_ = STOP;
                break;
            }
            // 获取下一个目标点
            if (path_index_ >= current_path_.size()) {
                message.linear.x = 0.0;
                publisher_->publish(message);
                RCLCPP_INFO(this->get_logger(), "Goal reached!");
                state_ = STOP;
                break;
            }
            int mx = current_path_[path_index_].first;
            int my = current_path_[path_index_].second;
            double wx, wy;
            map_to_world(mx, my, wx, wy);
            // TODO: 这里应该获取机器人当前位置
            // double rx, ry = ...;
            // if (距离(wx, wy) < 阈值) path_index_++;

            // 演示：直接前进
            message.linear.x = 0.2;
            // 可以加入方向调整（angular.z控制转向）

            publisher_->publish(message);
            path_index_++;
            break;
    }
}

// ======= 关键函数：Dijkstra算法 =========
bool Robot::dijkstra_search(
    const std::vector<int8_t>& map_data, int width, int height,
    std::pair<int, int> start, std::pair<int, int> goal,
    std::vector<std::pair<int, int>>& path)
{
    using Node = std::pair<int, std::pair<int, int>>; // 距离, (x, y)
    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> pq;
    std::vector<std::vector<int>> dist(height, std::vector<int>(width, std::numeric_limits<int>::max()));
    std::vector<std::vector<std::pair<int, int>>> prev(height, std::vector<std::pair<int, int>>(width, {-1, -1}));

    pq.push({0, start});
    dist[start.second][start.first] = 0;

    int dx[4] = {1, -1, 0, 0};
    int dy[4] = {0, 0, 1, -1};

    while (!pq.empty()) {
        auto [cost, u] = pq.top(); pq.pop();
        int ux = u.first, uy = u.second;

        if (u == goal) break;

        for (int d = 0; d < 4; ++d) {
            int vx = ux + dx[d], vy = uy + dy[d];
            if (vx < 0 || vx >= width || vy < 0 || vy >= height) continue;
            if (map_data[vy * width + vx] > 50) continue; // 认为50以上为障碍

            int ncost = cost + 1;
            if (ncost < dist[vy][vx]) {
                dist[vy][vx] = ncost;
                prev[vy][vx] = {ux, uy};
                pq.push({ncost, {vx, vy}});
            }
        }
    }
    // 回溯路径
    std::vector<std::pair<int, int>> rev_path;
    auto u = goal;
    while (u != start) {
        rev_path.push_back(u);
        u = prev[u.second][u.first];
        if (u.first == -1) {
            path.clear();
            return false;
        }
    }
    rev_path.push_back(start);
    std::reverse(rev_path.begin(), rev_path.end());
    path = rev_path;
    return true;
}

// 辅助：世界坐标->栅格坐标
bool Robot::world_to_map(double wx, double wy, int& mx, int& my) {
    if (!map_received_) return false;
    double origin_x = map_.info.origin.position.x;
    double origin_y = map_.info.origin.position.y;
    double resolution = map_.info.resolution;
    mx = int((wx - origin_x) / resolution);
    my = int((wy - origin_y) / resolution);
    if (mx < 0 || mx >= (int)map_.info.width || my < 0 || my >= (int)map_.info.height)
        return false;
    return true;
}
bool Robot::map_to_world(int mx, int my, double& wx, double& wy) {
    if (!map_received_) return false;
    double origin_x = map_.info.origin.position.x;
    double origin_y = map_.info.origin.position.y;
    double resolution = map_.info.resolution;
    wx = origin_x + (mx + 0.5) * resolution;
    wy = origin_y + (my + 0.5) * resolution;
    return true;
}

// 原有障碍检测可保留用于安全
bool Robot::detect_obstacle() {
    return false; // 若直接用路径规划可忽略
}

// 主函数
int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Robot>());
  rclcpp::shutdown();
  return 0;
}


#include "../include/Astar.hpp"

#include <cmath>
#include <set>
#include <limits>
#include <algorithm>

// 建议命名AstarNode而不是Node，避免与ROS2 Node混淆
struct AstarNode {
    std::pair<int,int> pt;
    int g; // 距起点距离
    int f; // 估价值 f = g + h
    AstarNode(std::pair<int,int> pt, int g, int f) : pt(pt), g(g), f(f) {}
    bool operator>(const AstarNode& other) const { return f > other.f; }
};

Robot::Robot()
    : Node("walker"),
      state_(STOP) {
    publisher_ = this->create_publisher<TWIST>("cmd_vel", 10);
    subscription_ = this->create_subscription<SCAN>(
        "/scan", 10, std::bind(&Robot::scan_callback, this, _1));

    map_sub_ = this->create_subscription<OCCUPANCY>(
        "/map", 1, std::bind(&Robot::map_callback, this, _1));
    goal_sub_ = this->create_subscription<POSESTAMPED>(
        "/goal_pose", 1, std::bind(&Robot::goal_callback, this, _1));

    timer_ = this->create_wall_timer(100ms, std::bind(&Robot::timer_callback, this));
}

void Robot::scan_callback(const SCAN& msg) { scan_ = msg; }
void Robot::map_callback(const OCCUPANCY& msg) { map_ = msg; map_received_ = true; }

void Robot::goal_callback(const POSESTAMPED& msg) {
    goal_ = msg; goal_received_ = true;
    if (map_received_) {
        int start_x, start_y, goal_x, goal_y;
        // 实际应取机器人当前位置，这里为演示
        world_to_map(goal_.pose.position.x, goal_.pose.position.y, goal_x, goal_y);
        world_to_map(goal_.pose.position.x, goal_.pose.position.y, start_x, start_y); // todo: 用机器人实际坐标
        std::vector<std::pair<int, int>> path;
        bool found = astar_search(
            map_.data, map_.info.width, map_.info.height,
            {start_x, start_y}, {goal_x, goal_y}, path);
        if (found) {
            current_path_ = path;
            path_index_ = 0;
            state_ = PATH_FOLLOW;
            RCLCPP_INFO(this->get_logger(), "A*: Path planned! Size: %ld", path.size());
        } else {
            RCLCPP_WARN(this->get_logger(), "A*: No path found!");
        }
    }
}

void Robot::timer_callback() {
    auto message = TWIST();
    switch (state_) {
        case FORWARD: break;
        case STOP: break;
        case TURN: break;
        case PATH_FOLLOW:
            if (current_path_.empty()) {
                message.linear.x = 0.0;
                publisher_->publish(message);
                state_ = STOP;
                break;
            }
            if (path_index_ >= current_path_.size()) {
                message.linear.x = 0.0;
                publisher_->publish(message);
                RCLCPP_INFO(this->get_logger(), "Goal reached!");
                state_ = STOP;
                break;
            }
            {
                int mx = current_path_[path_index_].first;
                int my = current_path_[path_index_].second;
                double wx, wy;
                map_to_world(mx, my, wx, wy);
                message.linear.x = 0.2;
                publisher_->publish(message);
                path_index_++;
            }
            break;
    }
}

bool Robot::astar_search(
    const std::vector<int8_t>& map_data, int width, int height,
    std::pair<int, int> start, std::pair<int, int> goal,
    std::vector<std::pair<int, int>>& path)
{
    auto heuristic = [](std::pair<int,int> a, std::pair<int,int> b) {
        return std::abs(a.first-b.first) + std::abs(a.second-b.second); // 曼哈顿距离
    };

    std::priority_queue<AstarNode, std::vector<AstarNode>, std::greater<AstarNode>> open;
    std::vector<std::vector<int>> g_score(height, std::vector<int>(width, std::numeric_limits<int>::max()));
    std::vector<std::vector<std::pair<int, int>>> prev(height, std::vector<std::pair<int, int>>(width, {-1, -1}));
    std::vector<std::vector<bool>> closed(height, std::vector<bool>(width, false));

    g_score[start.second][start.first] = 0;
    open.emplace(start, 0, heuristic(start, goal));

    int dx[4] = {1, -1, 0, 0};
    int dy[4] = {0, 0, 1, -1};

    while (!open.empty()) {
        AstarNode node = open.top(); open.pop();
        int ux = node.pt.first, uy = node.pt.second;
        if (closed[uy][ux]) continue;
        closed[uy][ux] = true;

        if (node.pt == goal) break;

        for (int d = 0; d < 4; ++d) {
            int vx = ux + dx[d], vy = uy + dy[d];
            if (vx < 0 || vx >= width || vy < 0 || vy >= height) continue;
            if (map_data[vy * width + vx] > 50) continue; // 50以上视为障碍
            int tentative_g = g_score[uy][ux] + 1;
            if (tentative_g < g_score[vy][vx]) {
                g_score[vy][vx] = tentative_g;
                prev[vy][vx] = {ux, uy};
                int f = tentative_g + heuristic({vx, vy}, goal);
                open.emplace(std::make_pair(vx, vy), tentative_g, f);
            }
        }
    }
    // 回溯路径
    std::vector<std::pair<int, int>> rev_path;
    auto u = goal;
    while (u != start) {
        rev_path.push_back(u);
        u = prev[u.second][u.first];
        if (u.first == -1) { path.clear(); return false; }
    }
    rev_path.push_back(start);
    std::reverse(rev_path.begin(), rev_path.end());
    path = rev_path;
    return true;
}

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

bool Robot::detect_obstacle() { return false; }

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Robot>());
  rclcpp::shutdown();
  return 0;
}


#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/empty.hpp>
#include <all_seaing_interfaces/srv/plan_path.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>

#include <queue>
#include <vector>
#include <cmath>
#include <algorithm>
#include <limits>
#include <float.h>
#include <atomic>

class AStarServer : public rclcpp::Node {
public:
    AStarServer() : Node("a_star_server") {
        abort_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/dynamic_map", 10,
            [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) { map_ = msg; });

        rclcpp::SubscriptionOptions abort_sub_opts;
        abort_sub_opts.callback_group = abort_cb_group_;
        abort_sub_ = this->create_subscription<std_msgs::msg::Empty>(
            "abort_planning", 10,
            [this](const std_msgs::msg::Empty::SharedPtr) { abort_flag_.store(true); },
            abort_sub_opts);

        service_ = this->create_service<all_seaing_interfaces::srv::PlanPath>(
            "plan_path",
            std::bind(&AStarServer::plan_callback, this,
                std::placeholders::_1, std::placeholders::_2));
    }

private:
    static constexpr int NUM_DIRS = 16;
    static constexpr int DX[16] = {1, 0, -1, 0, 1, -1, -1, 1, 1, 1, -1, -1, 2, -2, 2, -2};
    static constexpr int DY[16] = {0, 1, 0, -1, 1, -1, 1, -1, -2, 2, 2, -2, 1, 1, -1, -1};
    static constexpr double DD[16] = {
        1.0, 1.0, 1.0, 1.0, 1.4142135623730951, 1.4142135623730951, 1.4142135623730951, 1.4142135623730951,
        2.23606797749979, 2.23606797749979, 2.23606797749979, 2.23606797749979, 2.23606797749979, 2.23606797749979, 2.23606797749979, 2.23606797749979
    };

    struct PQNode {
        double f_score;
        int index;
        bool operator>(const PQNode& rhs) const { return f_score > rhs.f_score; }
    };

    nav_msgs::msg::OccupancyGrid::SharedPtr map_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr abort_sub_;
    rclcpp::CallbackGroup::SharedPtr abort_cb_group_;
    rclcpp::Service<all_seaing_interfaces::srv::PlanPath>::SharedPtr service_;
    std::atomic<bool> abort_flag_{false};

    std::vector<double> gscore_;
    std::vector<int> parent_;

    void plan_callback(
        const all_seaing_interfaces::srv::PlanPath::Request::SharedPtr request,
        all_seaing_interfaces::srv::PlanPath::Response::SharedPtr response)
    {
        geometry_msgs::msg::PoseArray result;
        RCLCPP_INFO(this->get_logger(), "Astar server received request");
        abort_flag_.store(false);

        if (!map_) {
            RCLCPP_WARN(this->get_logger(), "No map received yet");
            response->path = result;
            return;
        }

        result.header = map_->header;

        const auto& info = map_->info;
        int width = int(info.width);
        int height = int(info.height);
        double resolution = info.resolution;
        double origin_x = info.origin.position.x;
        double origin_y = info.origin.position.y;
        int64_t obstacle_tol = request->obstacle_tol;
        double goal_tol = request->goal_tol;
        const auto& map_data = map_->data;

        int start_x = int(std::floor((request->start.x - origin_x) / resolution));
        int start_y = int(std::floor((request->start.y - origin_y) / resolution));
        int goal_x = int(std::floor((request->goal.x - origin_x) / resolution));
        int goal_y = int(std::floor((request->goal.y - origin_y) / resolution));

        auto in_bounds = [&](int x, int y) -> bool {
            return x >= 0 && x < width && y >= 0 && y < height;
        };

        auto is_occupied = [&](int x, int y) -> bool {
            int val = int(map_data[x + y * width]);
            return val >= obstacle_tol; // assume val == -1 is not occupied
        };

        auto rect_occupied = [&](int x1, int y1, int x2, int y2) -> bool {
            int minx = std::min(x1, x2);
            int maxx = std::max(x1, x2);
            int miny = std::min(y1, y2);
            int maxy = std::max(y1, y2);
            for (int y = miny; y <= maxy; ++y) {
                for (int x = minx; x <= maxx; ++x) {
                    if (is_occupied(x, y)) return true;
                }
            }
            return false;
        };

        if (!in_bounds(start_x, start_y) || !in_bounds(goal_x, goal_y) || is_occupied(start_x, start_y) || is_occupied(goal_x, goal_y)) {
            RCLCPP_INFO(this->get_logger(), "Astar out of bounds/occupied, give up.");

            response->path = result;
            return;
        }

        double goal_tol_sq = goal_tol * goal_tol;

        auto heuristic = [&](int x, int y) -> double {
            double dx = double(x - goal_x);
            double dy = double(y - goal_y);
            return std::sqrt(dx * dx + dy * dy);
        };

        int map_size = width * height;

        if (int(gscore_.size()) < map_size) {
            gscore_.resize(map_size);
            parent_.resize(map_size);
        }
        std::fill(gscore_.begin(), gscore_.begin() + map_size, DBL_MAX);
        std::fill(parent_.begin(), parent_.begin() + map_size, -1);

        int start_idx = start_x + start_y * width;
        gscore_[start_idx] = 0.0;
        parent_[start_idx] = start_idx;

        std::priority_queue<PQNode, std::vector<PQNode>, std::greater<PQNode>> pq;
        pq.push({heuristic(start_x, start_y), start_idx});

        constexpr double EPSILON = 0.005;
        int found_idx = -1;

        RCLCPP_INFO(this->get_logger(), "Astar server astar");

        while (!pq.empty()) {
            if (abort_flag_.load()) {
                RCLCPP_INFO(this->get_logger(), "A* search aborted");
                response->path = result;
                return;
            }

            PQNode node = pq.top();
            pq.pop();

            int ci = node.index;
            int cx = ci % width;
            int cy = ci / width;

            if (std::abs(node.f_score - (gscore_[ci] + heuristic(cx, cy))) > EPSILON) {
                continue;
            }

            double gdx = double(cx - goal_x);
            double gdy = double(cy - goal_y);
            if (gdx * gdx + gdy * gdy < goal_tol_sq) {
                found_idx = ci;
                break;
            }

            for (int d = 0; d < NUM_DIRS; ++d) {
                int nx = cx + DX[d];
                int ny = cy + DY[d];

                if (!in_bounds(nx, ny) || rect_occupied(cx, cy, nx, ny)) continue;

                int ni = nx + ny * width;
                double nd = gscore_[ci] + DD[d];

                if (nd < gscore_[ni]) {
                    gscore_[ni] = nd;
                    parent_[ni] = ci;
                    pq.push({nd + heuristic(nx, ny), ni});
                }
            }
        }

        if (found_idx == -1) {
            response->path = result;        
            RCLCPP_INFO(this->get_logger(), "No path found..");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Astar trace");

        int ci = found_idx;
        while (ci != start_idx) {
            int gx = ci % width;
            int gy = ci / width;
            RCLCPP_INFO(this->get_logger(), "Point %d %d %d\n", gx, gy, map_data[gx + gy * width]);

            geometry_msgs::msg::Pose pose;
            pose.position.x = gx * resolution + origin_x;
            pose.position.y = gy * resolution + origin_y;
            result.poses.push_back(pose);
            ci = parent_[ci];
        }
        
        geometry_msgs::msg::Pose pose;
        pose.position.x = start_x * resolution + origin_x;
        pose.position.y = start_y * resolution + origin_y;
        result.poses.push_back(pose);

        std::reverse(result.poses.begin(), result.poses.end());
        response->path = result;
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor = rclcpp::executors::MultiThreadedExecutor(rclcpp::ExecutorOptions(), 3);
    auto node = std::make_shared<AStarServer>();
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}

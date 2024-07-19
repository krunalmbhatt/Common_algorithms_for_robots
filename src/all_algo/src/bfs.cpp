#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>
#include <queue>
#include <vector>

class BFSPlanner {
public:
    BFSPlanner() {
        map_sub_ = nh_.subscribe("map", 10, &BFSPlanner::mapCallback, this);
        marker_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber map_sub_;
    ros::Publisher marker_pub_;
    nav_msgs::OccupancyGrid map_;

    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
        map_ = *msg;
        planPath();
    }

    void planPath() {
        // BFS implementation
        std::queue<std::pair<int, int>> queue;
        std::vector<std::vector<bool>> visited(map_.info.width, std::vector<bool>(map_.info.height, false));

        // Start and goal points (for example purposes, set as hardcoded values)
        std::pair<int, int> start(10, 10);
        std::pair<int, int> goal(50, 50);

        queue.push(start);
        visited[start.first][start.second] = true;

        std::vector<std::pair<int, int>> directions = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};

        while (!queue.empty()) {
            auto current = queue.front();
            queue.pop();

            if (current == goal) {
                ROS_INFO("Path found!");
                visualizePath(current);
                return;
            }

            for (const auto& dir : directions) {
                std::pair<int, int> neighbor(current.first + dir.first, current.second + dir.second);

                if (neighbor.first >= 0 && neighbor.first < map_.info.width && neighbor.second >= 0 && neighbor.second < map_.info.height &&
                    !visited[neighbor.first][neighbor.second] && map_.data[neighbor.second * map_.info.width + neighbor.first] == 0) {
                    queue.push(neighbor);
                    visited[neighbor.first][neighbor.second] = true;
                }
            }
        }

        ROS_WARN("No path found!");
    }

    void visualizePath(const std::pair<int, int>& goal) {
        visualization_msgs::Marker points;
        points.header.frame_id = map_.header.frame_id;
        points.header.stamp = ros::Time::now();
        points.ns = "path";
        points.action = visualization_msgs::Marker::ADD;
        points.pose.orientation.w = 1.0;
        points.id = 0;
        points.type = visualization_msgs::Marker::POINTS;
        points.scale.x = 0.1;
        points.scale.y = 0.1;
        points.color.g = 1.0;
        points.color.a = 1.0;

        geometry_msgs::Point p;
        p.x = goal.first * map_.info.resolution;
        p.y = goal.second * map_.info.resolution;
        p.z = 0;

        points.points.push_back(p);
        marker_pub_.publish(points);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "bfs_planner");
    BFSPlanner bfs_planner;
    ros::spin();
    return 0;
}

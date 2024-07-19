#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>
#include <queue>
#include <vector>
#include <unordered_map>

struct Node {
    int x, y;
    double cost;
    Node* parent;
    
    Node(int x, int y, double cost, Node* parent = nullptr)
        : x(x), y(y), cost(cost), parent(parent) {}
    
    bool operator>(const Node& other) const {
        return cost > other.cost;
    }
};

class DijkstraPlanner {
public:
    DijkstraPlanner() {
        map_sub_ = nh_.subscribe("map", 10, &DijkstraPlanner::mapCallback, this);
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
        // Dijkstra implementation
        std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open_set;
        std::unordered_map<int, Node*> all_nodes;

        // Start and goal points (for example purposes, set as hardcoded values)
        Node* start = new Node(10, 10, 0);
        Node* goal = new Node(50, 50, 0);

        open_set.push(*start);
        all_nodes[start->y * map_.info.width + start->x] = start;

        std::vector<std::pair<int, int>> directions = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};

        while (!open_set.empty()) {
            Node current = open_set.top();
            open_set.pop();

            if (current.x == goal->x && current.y == goal->y) {
                ROS_INFO("Path found!");
                visualizePath(&current);
                return;
            }

            for (const auto& dir : directions) {
                int nx = current.x + dir.first;
                int ny = current.y + dir.second;
                double nc = current.cost + 1; // Assuming uniform cost

                if (nx >= 0 && nx < map_.info.width && ny >= 0 && ny < map_.info.height &&
                    map_.data[ny * map_.info.width + nx] == 0) {
                    
                    Node* neighbor = new Node(nx, ny, nc, &current);

                    if (all_nodes.find(neighbor->y * map_.info.width + neighbor->x) == all_nodes.end() ||
                        neighbor->cost < all_nodes[neighbor->y * map_.info.width + neighbor->x]->cost) {
                        
                        open_set.push(*neighbor);
                        all_nodes[neighbor->y * map_.info.width + neighbor->x] = neighbor;
                    }
                }
            }
        }

        ROS_WARN("No path found!");
    }

    void visualizePath(const Node* goal) {
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

        const Node* current = goal;
        while (current != nullptr) {
            geometry_msgs::Point p;
            p.x = current->x * map_.info.resolution;
            p.y = current->y * map_.info.resolution;
            p.z = 0;

            points.points.push_back(p);
            current = current->parent;
        }

        marker_pub_.publish(points);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "dijkstra_planner");
    DijkstraPlanner dijkstra_planner;
    ros::spin();
    return 0;
}

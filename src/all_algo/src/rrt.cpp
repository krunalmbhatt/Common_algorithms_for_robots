#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>
#include <random>
#include <vector>
#include <cmath>

struct Node {
    int x, y;
    Node* parent;
    
    Node(int x, int y, Node* parent = nullptr)
        : x(x), y(y), parent(parent) {}
};

class RRTPlanner {
public:
    RRTPlanner() {
        map_sub_ = nh_.subscribe("map", 10, &RRTPlanner::mapCallback, this);
        marker_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber map_sub_;
    ros::Publisher marker_pub_;
    nav_msgs::OccupancyGrid map_;
    std::vector<Node*> tree_;

    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
        map_ = *msg;
        planPath();
    }

    void planPath() {
        // RRT implementation
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<> dis_width(0, map_.info.width - 1);
        std::uniform_int_distribution<> dis_height(0, map_.info.height - 1);

        // Start and goal points (for example purposes, set as hardcoded values)
        Node* start = new Node(10, 10);
        Node* goal = new Node(50, 50);

        tree_.push_back(start);

        while (true) {
            int rand_x = dis_width(gen);
            int rand_y = dis_height(gen);

            Node* nearest = findNearest(rand_x, rand_y);
            Node* new_node = steer(nearest, rand_x, rand_y);

            if (new_node) {
                tree_.push_back(new_node);
                if (distance(new_node->x, new_node->y, goal->x, goal->y) < 5) { // Assuming 5 as a threshold
                    goal->parent = new_node;
                    ROS_INFO("Path found!");
                    visualizePath(goal);
                    return;
                }
            }
        }

        ROS_WARN("No path found!");
    }

    Node* findNearest(int x, int y) {
        Node* nearest = nullptr;
        double min_dist = std::numeric_limits<double>::max();

        for (auto node : tree_) {
            double dist = distance(x, y, node->x, node->y);
            if (dist < min_dist) {
                nearest = node;
                min_dist = dist;
            }
        }

        return nearest;
    }

    Node* steer(Node* nearest, int x, int y) {
        double theta = atan2(y - nearest->y, x - nearest->x);
        int new_x = nearest->x + cos(theta) * 5; // Assuming 5 as step size
        int new_y = nearest->y + sin(theta) * 5;

        if (new_x >= 0 && new_x < map_.info.width && new_y >= 0 && new_y < map_.info.height &&
            map_.data[new_y * map_.info.width + new_x] == 0) {
            return new Node(new_x, new_y, nearest);
        }

        return nullptr;
    }

    double distance(int x1, int y1, int x2, int y2) {
        return std::hypot(x2 - x1, y2 - y1);
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
    ros::init(argc, argv, "rrt_planner");
    RRTPlanner rrt_planner;
    ros::spin();
    return 0;
}

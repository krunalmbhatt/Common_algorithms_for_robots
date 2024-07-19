#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <limits>
#include <cmath>

bool collisionCheck(cv::Point p1, cv::Point p2, const cv::Mat &map);

class RRTStar {
public:
    RRTStar(int width, int height, const cv::Mat &map);
    void buildTree(cv::Point start, cv::Point goal);
    void visualize(cv::Mat &image);

private:
    bool isCollisionFree(cv::Point p1, cv::Point p2);
    cv::Point getRandomPoint();
    cv::Point getNearestPoint(cv::Point p);
    void rewire(cv::Point new_point);

    int width_;
    int height_;
    cv::Mat map_;
    std::vector<cv::Point> nodes_;
    std::vector<std::vector<cv::Point>> edges_;
};

RRTStar::RRTStar(int width, int height, const cv::Mat &map) : width_(width), height_(height), map_(map) {}

void RRTStar::buildTree(cv::Point start, cv::Point goal) {
    nodes_.push_back(start);

    while (true) {
        cv::Point random_point = getRandomPoint();
        cv::Point nearest_point = getNearestPoint(random_point);

        if (isCollisionFree(nearest_point, random_point)) {
            nodes_.push_back(random_point);
            edges_.push_back({nearest_point, random_point});
            rewire(random_point);

            if (cv::norm(random_point - goal) < 10.0) {
                edges_.push_back({random_point, goal});
                break;
            }
        }
    }
}

void RRTStar::visualize(cv::Mat &image) {
    for (const auto &edge : edges_) {
        cv::line(image, edge[0], edge[1], cv::Scalar(255, 0, 0), 2);
    }
    for (const auto &node : nodes_) {
        cv::circle(image, node, 3, cv::Scalar(0, 255, 0), -1);
    }
}

bool RRTStar::isCollisionFree(cv::Point p1, cv::Point p2) {
    return collisionCheck(p1, p2, map_);
}

cv::Point RRTStar::getRandomPoint() {
    return cv::Point(rand() % width_, rand() % height_);
}

cv::Point RRTStar::getNearestPoint(cv::Point p) {
    cv::Point nearest_point;
    double min_dist = std::numeric_limits<double>::max();

    for (const auto &node : nodes_) {
        double dist = cv::norm(node - p);
        if (dist < min_dist) {
            min_dist = dist;
            nearest_point = node;
        }
    }
    return nearest_point;
}

void RRTStar::rewire(cv::Point new_point) {
    // Rewire implementation omitted for brevity
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "rrt_star");
    ros::NodeHandle nh;

    cv::Mat map = cv::imread("src/all_algo/maps/map.pgm", cv::IMREAD_GRAYSCALE);
    if (map.empty()) {
        ROS_ERROR("Failed to load the map image.");
        return -1;
    }

    RRTStar rrt_star(800, 800, map);
    rrt_star.buildTree(cv::Point(50, 50), cv::Point(750, 750));

    cv::Mat image = cv::Mat::zeros(800, 800, CV_8UC3);
    rrt_star.visualize(image);

    cv::imshow("RRT* Star", image);
    cv::waitKey(0);

    ros::spin();
    return 0;
}

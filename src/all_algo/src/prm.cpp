#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <vector>

bool collisionCheck(cv::Point p1, cv::Point p2, const cv::Mat &map);

class PRM {
public:
    PRM(int width, int height, const cv::Mat &map);
    void createRoadmap(int num_samples);
    std::vector<cv::Point> findPath(cv::Point start, cv::Point goal);
    void visualize(cv::Mat &image);

private:
    bool isCollisionFree(cv::Point p1, cv::Point p2);
    void addEdge(cv::Point p1, cv::Point p2);

    int width_;
    int height_;
    cv::Mat map_;
    std::vector<cv::Point> samples_;
    std::vector<std::vector<cv::Point>> edges_;
};

PRM::PRM(int width, int height, const cv::Mat &map) : width_(width), height_(height), map_(map) {}

void PRM::createRoadmap(int num_samples) {
    // Generate random samples
    for (int i = 0; i < num_samples; ++i) {
        cv::Point p(rand() % width_, rand() % height_);
        samples_.push_back(p);
    }

    // Add edges between samples
    for (size_t i = 0; i < samples_.size(); ++i) {
        for (size_t j = i + 1; j < samples_.size(); ++j) {
            if (isCollisionFree(samples_[i], samples_[j])) {
                addEdge(samples_[i], samples_[j]);
            }
        }
    }
}

std::vector<cv::Point> PRM::findPath(cv::Point start, cv::Point goal) {
    // Simple Dijkstra's Algorithm for path finding (implementation omitted for brevity)
    return std::vector<cv::Point>();
}

void PRM::visualize(cv::Mat &image) {
    for (const auto &edge : edges_) {
        cv::line(image, edge[0], edge[1], cv::Scalar(255, 0, 0), 2);
    }
    for (const auto &sample : samples_) {
        cv::circle(image, sample, 3, cv::Scalar(0, 255, 0), -1);
    }
}

bool PRM::isCollisionFree(cv::Point p1, cv::Point p2) {
    return collisionCheck(p1, p2, map_);
}

void PRM::addEdge(cv::Point p1, cv::Point p2) {
    edges_.push_back({p1, p2});
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "prm");
    ros::NodeHandle nh;

    cv::Mat map = cv::imread("src/all_algo/maps/map.pgm", cv::IMREAD_GRAYSCALE);
    if (map.empty()) {
        ROS_ERROR("Failed to load the map image.");
        return -1;
    }

    PRM prm(800, 800, map);
    prm.createRoadmap(100);

    cv::Mat image = cv::Mat::zeros(800, 800, CV_8UC3);
    prm.visualize(image);

    cv::imshow("PRM", image);
    cv::waitKey(0);

    ros::spin();
    return 0;
}

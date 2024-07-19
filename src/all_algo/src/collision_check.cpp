// Implement collision check for robot

#include <opencv2/opencv.hpp>

bool collisionCheck(cv::Point p1, cv::Point p2, const cv::Mat &map) {
    // Check if the points are within the map boundaries
    if (p1.x < 0 || p1.x >= map.cols || p1.y < 0 || p1.y >= map.rows ||
        p2.x < 0 || p2.x >= map.cols || p2.y < 0 || p2.y >= map.rows) {
        return false;
    }

    // Create a line iterator
    cv::LineIterator it(map, p1, p2, 8);

    // Iterate over the pixels of the line
    for (int i = 0; i < it.count; i++, ++it) {
        // Check if the pixel is an obstacle (assumes obstacles are marked with a value of 0)
        if (map.at<uchar>(it.pos()) == 0) {
            return false;
        }
    }

    return true;
}

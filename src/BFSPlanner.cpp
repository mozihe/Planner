//
// Created by mozihe on 25-2-7.
//
#include "BFSPlanner.h"
#include <queue>

std::vector<cv::Point> BFSPlanner::plan() {
    std::vector<cv::Point> path;
    if (map_.empty() || start_ == goal_)
        return path;

    cv::Mat visited = cv::Mat::zeros(map_.size(), CV_8U);
    std::queue<cv::Point> queue;
    std::map<cv::Point, cv::Point, cmpPoints> parent;

    queue.push(start_);
    visited.at<uchar>(start_) = 1;

    const std::vector<cv::Point> directions = {{1, 0}, {0, 1}, {-1, 0}, {0, -1}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1}};

    while (!queue.empty()) {
        cv::Point current = queue.front();
        queue.pop();

        if (current == goal_) {
            while (current != start_) {
                path.push_back(current);
                current = parent[current];
            }
            path.push_back(start_);
            std::ranges::reverse(path);
            return path;
        }

        for (const auto &dir: directions) {
            cv::Point next = current + dir;
            if (isValid(next, visited)) {
                queue.push(next);
                visited.at<uchar>(next) = 1;
                parent[next] = current;
            }
        }
    }
    return {};
}

bool BFSPlanner::isValid(const cv::Point &p, const cv::Mat &visited) {
    return p.x >= 0 && p.x < map_.cols && p.y >= 0 && p.y < map_.rows && map_.at<uchar>(p) == 255 &&
           visited.at<uchar>(p) == 0;
}

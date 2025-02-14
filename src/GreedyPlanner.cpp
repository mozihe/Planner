//
// Created by mozihe on 25-2-14.
//

#include "GreedyPlanner.h"

std::vector<cv::Point> GreedyPlanner::plan() {
    std::vector<cv::Point> path;
    cv::Point current = start_;

    cv::Mat visited = cv::Mat::zeros(map_.size(), CV_8U);

    const std::vector<cv::Point> directions = {{1, 0}, {0, 1}, {-1, 0}, {0, -1}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1}};

    while (current != goal_) {
        visited.at<uchar>(current) = 1;
        path.push_back(current);
        cv::Point bestNext = current;
        double minDist = std::numeric_limits<double>::infinity();
        for (const auto &dir: directions) {
            cv::Point neighbor = current + dir;

            if (neighbor.x < 0 || neighbor.x >= map_.cols || neighbor.y < 0 || neighbor.y >= map_.rows) {
                continue;
            }
            if (visited.at<uchar>(neighbor) == 1 || map_.at<uchar>(neighbor) == 0) {
                continue;
            }

            double dist = cv::norm(neighbor - goal_);
            if (dist < minDist) {
                minDist = dist;
                bestNext = neighbor;
            }
        }

        if (bestNext == current) {
            return {};
        }

        current = bestNext;
    }

    path.push_back(goal_);
    return path;
}

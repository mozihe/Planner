//
// Created by mozihe on 25-2-14.
//

#include "MonkeyPlanner.h"

std::vector<cv::Point> MonkeyPlanner::plan() {

    if (start_ == goal_) {
        return {start_};
    }

    if (!isValid(start_) || !isValid(goal_)) {
        return {};
    }

    std::srand(static_cast<unsigned int>(std::time(nullptr)));

    std::vector<cv::Point> path;
    cv::Point current = start_;

    const std::vector<cv::Point> directions = {{1, 0}, {0, 1}, {-1, 0}, {0, -1}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1}};

    while (current != goal_) {
        path.push_back(current);

        cv::Point next = current;
        while (true) {
            int dirIndex = std::rand() % directions.size();
            next = current + directions[dirIndex];
            if (isValid(next)) {
                break;
            }
        }
        current = next;
    }

    path.push_back(goal_);
    return path;
}

bool MonkeyPlanner::isValid(const cv::Point &p) {
    return p.x >= 0 && p.x < map_.cols && p.y >= 0 && p.y < map_.rows && map_.at<uchar>(p) == 255;
}

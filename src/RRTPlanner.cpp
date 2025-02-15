//
// Created by mozihe on 25-2-14.
//

#include "RRTPlanner.h"

std::vector<cv::Point> RRTPlanner::plan() {

    if (!isValid(start_) || !isValid(goal_)) {
        return {};
    }

    std::map<cv::Point, cv::Point, cmpPoints> tree;
    tree[start_] = start_;


    for (int i = 0; i < max_iter_; ++i) {
        cv::Point random_point = sampleRandomPoint();
        cv::Point nearest = nearestNode(tree, random_point);
        cv::Point new_node = nearest + step_size_ * (random_point - nearest) / cv::norm(random_point - nearest);
        if (isValidMove(nearest, new_node) && !tree.contains(new_node)) {
            tree[new_node] = nearest;
            if (cv::norm(new_node - goal_) < step_size_) {
                if (isValidMove(new_node, goal_)) {
                    tree[goal_] = new_node;
                    break;
                }
            }
        }
    }

    return tracePath(tree, goal_);
}

cv::Point RRTPlanner::sampleRandomPoint() {
    std::uniform_int_distribution<int> dis_x(0, map_.cols);
    std::uniform_int_distribution<int> dis_y(0, map_.rows);
    return {dis_x(gen), dis_y(gen)};
}

cv::Point RRTPlanner::nearestNode(const std::map<cv::Point, cv::Point, cmpPoints>& tree, const cv::Point& random_point) {
    cv::Point nearest = tree.begin()->first;
    double min_dist = cv::norm(nearest - random_point);
    for (const auto& node : tree) {
        double dist = cv::norm(node.first - random_point);
        if (dist < min_dist) {
            min_dist = dist;
            nearest = node.first;
        }
    }
    return nearest;
}

bool RRTPlanner::isValidMove(const cv::Point& start, const cv::Point& end) {
    cv::LineIterator it(map_, start, end, 8);
    for (int i = 0; i < it.count; ++i, ++it) {
        if (map_.at<uchar>(it.pos()) == 0) {
            return false;
        }
    }
    return true;
}

std::vector<cv::Point> RRTPlanner::tracePath(const std::map<cv::Point, cv::Point, cmpPoints>& tree, const cv::Point& goal) {
    std::vector<cv::Point> path;
    cv::Point current = goal;
    path.push_back(current);

    while (current != start_) {
        current = tree.at(current);
        path.push_back(current);
    }

    std::ranges::reverse(path);
    return path;
}

bool RRTPlanner::isValid(const cv::Point &point) const {
    return point.x >= 0 && point.x < map_.cols && point.y >= 0 && point.y < map_.rows && map_.at<uchar>(point) == 255;
}
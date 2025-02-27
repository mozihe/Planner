//
// Created by mozihe on 25-2-14.
//

#include "RRTSharpPlanner.h"

std::vector<cv::Point> RRTSharpPlanner::plan() {

    parent_map_.clear();
    cost_map_.clear();
    tree_.clear();


    if (!isValid(start_) || !isValid(goal_)) {
        return {};
    }

    tree_.add(start_);
    parent_map_[start_] = start_;
    cost_map_[start_] = 0.0;

    double best_cost = std::numeric_limits<double>::max();
    std::vector<cv::Point> best_path;

    for (int i = 0; i < max_iter_; ++i) {
        cv::Point random_point = sampleRandomPoint();
        std::vector<cv::Point> nearest = tree_.kNearestNeighbors(random_point, 1);
        cv::Point nearest_node = nearest.empty() ? start_ : nearest[0];

        cv::Point direction = random_point - nearest_node;
        double dist = cv::norm(direction);
        cv::Point new_node = dist <= step_size_ ? random_point : nearest_node + (direction / dist) * step_size_;

        if (!isValidMove(nearest_node, new_node) || tree_.contains(new_node)) {
            continue;
        }

        if (new_node == goal_) {
            continue;
        }

        std::vector<cv::Point> neighbors = findNearNodes(new_node, rewire_r_);

        cv::Point min_node = nearest_node;
        double min_cost = cost_map_[nearest_node] + cv::norm(nearest_node - new_node);
        double min_total_cost = min_cost + heuristic(new_node, goal_);

        for (const auto &node: neighbors) {
            if (!isValidMove(node, new_node)) {
                continue;
            }

            double actual_cost = cost_map_[node] + cv::norm(node - new_node);
            double total_cost = actual_cost + heuristic(new_node, goal_);

            if (total_cost < min_total_cost) {
                min_total_cost = total_cost;
                min_cost = actual_cost;
                min_node = node;
            }
        }



        tree_.add(new_node);
        parent_map_[new_node] = min_node;
        cost_map_[new_node] = min_cost;


        for (const auto &node : neighbors) {
            double rewire_cost = cost_map_[new_node] + cv::norm(new_node - node);
            if (rewire_cost < cost_map_[node] && isValidMove(new_node, node)) {
                parent_map_[node] = new_node;
                cost_map_[node] = rewire_cost;
            }
        }


        if (cv::norm(new_node - goal_) < step_size_ && isValidMove(new_node, goal_)) {

            parent_map_[goal_] = new_node;
            std::vector<cv::Point> current_path = tracePath(goal_);
            double current_cost = cost_map_[goal_];

            if (current_cost < best_cost) {
                best_cost = current_cost;
                best_path = current_path;
            }
        }

    }

    return best_path;
}

cv::Point RRTSharpPlanner::sampleRandomPoint() {
    std::uniform_real_distribution<double> dis_bias(0.0, 1.0);
    if (dis_bias(gen) < goal_bias_) {
        return goal_;
    } else {
        std::uniform_int_distribution<int> dis_x(0, map_.cols - 1);
        std::uniform_int_distribution<int> dis_y(0, map_.rows - 1);
        return {dis_x(gen), dis_y(gen)};
    }
}
cv::Point RRTSharpPlanner::nearestNode(const std::map<cv::Point, cv::Point, cmpPoints> &tree,
                                      const cv::Point &random_point) {
    cv::Point nearest = tree.begin()->first;
    double min_dist = cv::norm(nearest - random_point);
    for (const auto &node: tree) {
        double dist = cv::norm(node.first - random_point);
        if (dist < min_dist) {
            min_dist = dist;
            nearest = node.first;
        }
    }
    return nearest;
}

bool RRTSharpPlanner::isValidMove(const cv::Point &start, const cv::Point &end) {
    cv::LineIterator it(map_, start, end, 8);
    for (int i = 0; i < it.count; ++i, ++it) {
        if (map_.at<uchar>(it.pos()) == 0) {
            return false;
        }
    }
    return true;
}

std::vector<cv::Point> RRTSharpPlanner::tracePath(const cv::Point &goal) {
    std::vector<cv::Point> path;
    if (!parent_map_.contains(goal))
        return path;

    cv::Point current = goal;
    while (current != start_) {
        path.push_back(current);
        current = parent_map_[current];
    }
    path.push_back(start_);
    std::ranges::reverse(path);
    return path;
}


bool RRTSharpPlanner::isValid(const cv::Point &point) const {
    return point.x >= 0 && point.x < map_.cols && point.y >= 0 && point.y < map_.rows && map_.at<uchar>(point) == 255;
}

std::vector<cv::Point> RRTSharpPlanner::findNearNodes(const cv::Point &query, double radius) {
    std::vector<cv::Point> candidates = tree_.kNearestNeighbors(query, 20);
    std::vector<cv::Point> result;
    for (const auto &p: candidates) {
        if (cv::norm(p - query) <= radius)
            result.push_back(p);
    }
    return result;
}


double RRTSharpPlanner::heuristic(const cv::Point &a, const cv::Point &b) {
    // return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
    int dx = std::abs(a.x - b.x);
    int dy = std::abs(a.y - b.y);
    return static_cast<double>(dx + dy) + (std::sqrt(2) - 2.0) * std::min(dx, dy);
}



//
// Created by mozihe on 25-2-14.
//

#include "PRMPlanner.h"

std::vector<cv::Point> PRMPlanner::plan() {
    generateRoadmap();
    std::vector<cv::Point> path = findPath();
    return path;
}

void PRMPlanner::generateRoadmap() {
    nodes_.clear();
    edges_.clear();

    num_nodes_ = static_cast<int>(map_.cols * map_.rows * target_density_);
    max_distance_ = sqrt(map_.cols * map_.rows / num_nodes_) * factor_;

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis_x(0, map_.cols - 1);
    std::uniform_int_distribution<> dis_y(0, map_.rows - 1);

    for (int i = 0; i < num_nodes_; ++i) {
        cv::Point node(dis_x(gen), dis_y(gen));
        if (isValid(node)) {
            nodes_.push_back(node);
        }
    }

    nodes_.push_back(start_);
    nodes_.push_back(goal_);

    KDTree tree(nodes_);

    std::set<std::pair<cv::Point, cv::Point>, PointPairCompare> edgeSet;

    for (const auto& node : nodes_) {
        auto neighbors = tree.kNearestNeighbors(node, k_neighbors_);
        for (const auto& neighbor : neighbors) {
            if (node != neighbor && cv::norm(node - neighbor) <= max_distance_ && !checkCollision(node, neighbor)) {
                if (node.x < neighbor.x || (node.x == neighbor.x && node.y < neighbor.y)) {
                    edgeSet.emplace(node, neighbor);
                } else {
                    edgeSet.emplace(neighbor, node);
                }
            }
        }
    }

    for (const auto& edge : edgeSet) {
        edges_.emplace_back(edge.first, edge.second);
    }
}

std::vector<cv::Point> PRMPlanner::findPath() {
    std::vector<cv::Point> path;
    if (isValid(start_) && isValid(goal_)) {
        std::unordered_map<cv::Point, cv::Point> cameFrom;
        std::unordered_map<cv::Point, double> gScore, fScore;
        std::priority_queue<std::pair<double, cv::Point>, std::vector<std::pair<double, cv::Point>>, PairCompare> openSet;

        for (const auto& node : nodes_) {
            gScore[node] = std::numeric_limits<double>::infinity();
            fScore[node] = std::numeric_limits<double>::infinity();
        }

        gScore[start_] = 0;
        fScore[start_] = heuristic(start_, goal_);
        openSet.emplace(fScore[start_], start_);

        while (!openSet.empty()) {
            auto current = openSet.top().second;
            if (current == goal_) {
                while (current != start_) {
                    path.push_back(current);
                    current = cameFrom[current];
                }
                path.push_back(start_);
                std::ranges::reverse(path);
                return path;
            }

            openSet.pop();

            for (const auto& edge : edges_) {
                cv::Point neighbor;
                if (edge.first == current) {
                    neighbor = edge.second;
                } else if (edge.second == current) {
                    neighbor = edge.first;
                } else {
                    continue;
                }

                double tentative_gScore = gScore[current] + cv::norm(current - neighbor);
                if (tentative_gScore < gScore[neighbor]) {
                    cameFrom[neighbor] = current;
                    gScore[neighbor] = tentative_gScore;
                    fScore[neighbor] = tentative_gScore + heuristic(neighbor, goal_);
                    openSet.emplace(fScore[neighbor], neighbor);
                }
            }
        }
    }
    return path; // 返回空路径表示未找到
}
double PRMPlanner::heuristic(const cv::Point& a, const cv::Point& b) const {
    return std::abs(a.x - b.x) + std::abs(a.y - b.y);
}

bool PRMPlanner::isValid(const cv::Point &point) const {
    return point.x >= 0 && point.x < map_.cols && point.y >= 0 && point.y < map_.rows && map_.at<uchar>(point) == 255;
}

bool PRMPlanner::checkCollision(const cv::Point& start, const cv::Point& end) const {
    cv::LineIterator it(map_, start, end);
    for (int i = 0; i < it.count; ++i, ++it) {
        if (map_.at<uchar>(it.pos()) != 255) {
            return true;
        }
    }
    return false;
}
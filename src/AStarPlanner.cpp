//
// Created by mozihe on 25-2-7.
//
#include <AStarPlanner.h>

std::vector<cv::Point> AStarPlanner::plan() {
    std::vector<cv::Point> path;
    if (map_.empty() || start_ == goal_)
        return path;

    cv::Mat visited = cv::Mat::zeros(map_.size(), CV_8U);
    using State = std::tuple<double, double, cv::Point>;
    std::priority_queue<State, std::vector<State>, compareTuple> pq;
    std::map<cv::Point, cv::Point, cmpPoints> parent;
    std::map<cv::Point, double, cmpPoints> distance;

    pq.emplace(heuristic(start_, goal_), 0.0, start_);
    distance[start_] = 0.0;

    const std::vector<cv::Point> directions = {{1, 0}, {0, 1}, {-1, 0}, {0, -1}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1}};

    while (!pq.empty()) {
        auto [currentF, currentDistance, current] = pq.top();
        pq.pop();
        visited.at<uchar>(current) = 1;

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
            double newDistance = currentDistance + std::sqrt(std::pow(dir.x, 2) + std::pow(dir.y, 2));
            if (isValid(next, visited) && (!distance.contains(next) || newDistance < distance[next])) {
                pq.emplace(heuristic(next, goal_) + newDistance, newDistance, next);
                parent[next] = current;
                distance[next] = newDistance;
            }
        }
    }

    return {};
}

bool AStarPlanner::isValid(const cv::Point &p, const cv::Mat &visited) {
    return p.x >= 0 && p.x < map_.cols && p.y >= 0 && p.y < map_.rows && map_.at<uchar>(p) == 255 &&
           visited.at<uchar>(p) == 0;
}

double AStarPlanner::heuristic(const cv::Point &a, const cv::Point &b) {
    // return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
    int dx = std::abs(a.x - b.x);
    int dy = std::abs(a.x - b.x);
    return static_cast<double>(dx + dy) + (std::sqrt(2) - 2.0) * std::min(dx, dy);
}

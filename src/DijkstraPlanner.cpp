//
// Created by mozihe on 25-2-7.
//

#include "DijkstraPlanner.h"
#include <queue>


std::vector<cv::Point> DijkstraPlanner::plan() {
    std::vector<cv::Point> path;
    if (map_.empty() || start_ == goal_)
        return path;

    cv::Mat visited = cv::Mat::zeros(map_.size(), CV_8U);
    std::priority_queue<std::pair<double, cv::Point>, std::vector<std::pair<double, cv::Point>>, ComparePairs> pq;
    std::map<cv::Point, cv::Point, cmpPoints> parent;
    std::map<cv::Point, double, cmpPoints> distance;

    pq.emplace(0.0, start_);
    distance[start_] = 0.0;


    const std::vector<cv::Point> directions = {{1, 0}, {0, 1}, {-1, 0}, {0, -1}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1}};

    while (!pq.empty()) {
        auto [currentDistance, current] = pq.top();
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
                pq.emplace(newDistance, next);
                parent[next] = current;
                distance[next] = newDistance;
            }
        }
    }

    return {};
}

bool DijkstraPlanner::isValid(const cv::Point &p, const cv::Mat &visited) {
    return p.x >= 0 && p.x < map_.cols && p.y >= 0 && p.y < map_.rows && map_.at<uchar>(p) == 255 &&
           visited.at<uchar>(p) == 0;
}

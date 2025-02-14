//
// Created by mozihe on 25-2-10.
//
#include "JPSPlanner.h"

std::vector<cv::Point> JPSPlanner::plan() {
    std::vector<cv::Point> path;
    if (map_.empty() || start_ == goal_)
        return path;

    cv::Mat visited = cv::Mat::zeros(map_.size(), CV_8U);
    using State = std::tuple<double, double, cv::Point>;
    std::priority_queue<State, std::vector<State>, compareTuple> pq;

    distance.clear();
    parent.clear();

    pq.emplace(heuristic(start_, goal_), 0.0, start_);
    distance[start_] = 0.0;

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

        std::vector<State> neighbors = getJumpNeighbors(current);

        for (const auto &neighbor: neighbors) {
            auto [newF, newDistance, next] = neighbor;

            if (isValid(next, visited) && (!distance.contains(next) || newDistance < distance[next])) {
                pq.emplace(newF, newDistance, next);
                parent[next] = current;
                distance[next] = newDistance;
            }
        }

    }

    return {};
}

bool JPSPlanner::isValid(const cv::Point &p, const cv::Mat &visited) {
    return p.x >= 0 && p.x < map_.cols && p.y >= 0 && p.y < map_.rows && map_.at<uchar>(p) == 255 &&
           visited.at<uchar>(p) == 0;
}

bool JPSPlanner::isValidPoint(const cv::Point &p) {
    return p.x >= 0 && p.x < map_.cols && p.y >= 0 && p.y < map_.rows && map_.at<uchar>(p) == 255;
}

double JPSPlanner::heuristic(const cv::Point &a, const cv::Point &b) {
    // return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
    int dx = std::abs(a.x - b.x);
    int dy = std::abs(a.y - b.y);
    return static_cast<double>(dx + dy) + (std::sqrt(2) - 2.0) * std::min(dx, dy);
}

std::vector<std::tuple<double, double, cv::Point>> JPSPlanner::getJumpNeighbors(const cv::Point &p) {
    std::vector<std::tuple<double, double, cv::Point>> neighbors;



    cv::Point parentPoint = parent.contains(p) ? parent[p] : cv::Point(-1, -1);
    cv::Point dir(0, 0);

    if (parentPoint != cv::Point(-1, -1)) {
        dir = p - parentPoint;
        dir.x = dir.x == 0 ? 0 : dir.x / std::abs(dir.x);
        dir.y = dir.y == 0 ? 0 : dir.y / std::abs(dir.y);
    }

    if (dir == cv:: Point(0, 0)) {
        const std::vector<cv::Point> directions = {{1, 0}, {0, 1}, {-1, 0}, {0, -1}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1}};
        for (const auto &d: directions) {
            cv::Point next = jump(p, d);
            if (next != cv::Point(-1, -1)) {
                double newDistance = distance[p] + std::sqrt(std::pow(p.x - next.x, 2) + std::pow(p.y - next.y, 2));
                double newF = newDistance + heuristic(next, goal_);
                neighbors.emplace_back(newF, newDistance, next);
            }

        }

    } else {

        cv::Point next = jump(p, dir);
        if (next != cv::Point(-1, -1)) {
            double newDistance = distance[p] + std::sqrt(std::pow(p.x - next.x, 2) + std::pow(p.y - next.y, 2));
            double newF = newDistance + heuristic(next, goal_);
            neighbors.emplace_back(newF, newDistance, next);
        }


        if (dir.x != 0 && dir.y != 0) {

            cv::Point jump1 = jump(p, cv::Point(dir.x, 0));
            cv::Point jump2 = jump(p, cv::Point(0, dir.y));

            if (jump1 != cv::Point(-1, -1)) {
                double newDistance = distance[p] + std::sqrt(std::pow(p.x - jump1.x, 2) + std::pow(p.y - jump1.y, 2));
                double newF = newDistance + heuristic(jump1, goal_);
                neighbors.emplace_back(newF, newDistance, jump1);
            }

            if (jump2 != cv::Point(-1, -1)) {
                double newDistance = distance[p] + std::sqrt(std::pow(p.x - jump2.x, 2) + std::pow(p.y - jump2.y, 2));
                double newF = newDistance + heuristic(jump2, goal_);
                neighbors.emplace_back(newF, newDistance, jump2);
            }
        }



        if (hasForceNeighbor(p, dir)) {
            const std::vector<cv::Point> forceNeighbors = getForceNeighbor(p, dir);
            for (const auto &forceNeighbor: forceNeighbors) {
                cv::Point jumpPoint = jump(p, forceNeighbor);
                if (jumpPoint != cv::Point(-1, -1)) {
                    double newDistance = distance[p] + std::sqrt(std::pow(jumpPoint.x - p.x, 2) + std::pow(jumpPoint.y - p.y, 2));
                    double newF = newDistance + heuristic(jumpPoint, goal_);
                    neighbors.emplace_back(newF, newDistance, jumpPoint);
                }
            }
        }
    }


    return neighbors;
}

bool JPSPlanner::hasForceNeighbor(const cv::Point &p, const cv::Point &dir) {
    return !getForceNeighbor(p, dir).empty();
}

std::vector<cv::Point> JPSPlanner::getForceNeighbor(const cv::Point &p, const cv::Point &dir) {
    std::vector<cv::Point> neighbors;
    if (dir.x == 0 && dir.y == 0) {
        return neighbors;
    }
    if (dir.x != 0 && dir.y != 0) {
        cv::Point need_check1 = p + cv::Point(-dir.x, 0);
        cv::Point need_check2 = p + cv::Point(0, -dir.y);
        if (!isValidPoint(need_check1) && isValidPoint(need_check1 + cv::Point(0, dir.y))) {
            neighbors.emplace_back(-dir.x, dir.y);
        }
        if (!isValidPoint(need_check2) && isValidPoint(need_check2 + cv::Point(dir.x, 0))) {
            neighbors.emplace_back(dir.x, -dir.y);
        }
        return neighbors;
    }
    if (dir.x != 0) {
        cv::Point need_check1 = p + cv::Point(0, 1);
        cv::Point need_check2 = p + cv::Point(0, -1);
        if (!isValidPoint(need_check1) && isValidPoint(need_check1 + cv::Point(dir.x, 0))) {
            neighbors.emplace_back(dir.x, 1);
        }
        if (!isValidPoint(need_check2) && isValidPoint(need_check2 + cv::Point(dir.x, 0))) {
            neighbors.emplace_back(dir.x, -1);
        }
        return neighbors;
    }
    cv::Point need_check1 = p + cv::Point(1, 0);
    cv::Point need_check2 = p + cv::Point(-1, 0);
    if (!isValidPoint(need_check1) && isValidPoint(need_check1 + cv::Point(0, dir.y))) {
        neighbors.emplace_back(1, dir.y);
    }
    if (!isValidPoint(need_check2) && isValidPoint(need_check2 + cv::Point(0, dir.y))) {
        neighbors.emplace_back(-1, dir.y);
    }
    return neighbors;
}

cv::Point JPSPlanner::jump(const cv::Point &p, const cv::Point &dir) {
    cv::Point next = p + dir;
    if (!isValidPoint(next)) {
        return {-1, -1};;
    }
    if (next == goal_) {
        return next;
    }
    if (hasForceNeighbor(next, dir)) {
        return next;
    }

    if (dir.x != 0 && dir.y != 0) {
        if (jump(next, cv::Point(dir.x, 0)) != cv::Point(-1, -1) || jump(next, cv::Point(0, dir.y)) != cv::Point(-1, -1)) {
            return next;
        }
    }

    return jump(next, dir);
}

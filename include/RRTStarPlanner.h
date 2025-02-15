//
// Created by mozihe on 25-2-14.
//

#ifndef RRTSTARPLANNER_H
#define RRTSTARPLANNER_H

#include "BasePlanner.h"

#include "KDTree.h"
#include <random>

class RRTStarPlanner : public BasePlanner {
public:
    RRTStarPlanner(double step_size = 40.0, int max_iter = 10000, double rewire_r = 50.0)
        : step_size_(step_size), max_iter_(max_iter), rewire_r_(rewire_r), gen(std::random_device{}()){}

    std::vector<cv::Point> plan() override;

private:

    struct PointHash {
        size_t operator()(const cv::Point& p) const {
            return std::hash<int>()(p.x) ^ (std::hash<int>()(p.y) << 1);
        }
    };

    double step_size_;
    int max_iter_;

    double rewire_r_;

    KDTree tree_;
    std::unordered_map<cv::Point, cv::Point, PointHash> parent_map_;
    std::unordered_map<cv::Point, double, PointHash> cost_map_;

    std::mt19937 gen;

    struct cmpPoints {
        bool operator()(const cv::Point &a, const cv::Point &b) const {
            return std::tie(a.x, a.y) < std::tie(b.x, b.y);
        }
    };

    cv::Point sampleRandomPoint();

    bool isValid(const cv::Point &point) const;
    static cv::Point nearestNode(const std::map<cv::Point, cv::Point, cmpPoints>& tree, const cv::Point& random_point);
    bool isValidMove(const cv::Point& start, const cv::Point& end);
    std::vector<cv::Point> tracePath(const cv::Point& goal);
    std::vector<cv::Point> findNearNodes(const cv::Point& query, double radius);
};
#endif //RRTSTARPLANNER_H

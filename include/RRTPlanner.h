//
// Created by mozihe on 25-2-14.
//

#ifndef RRTPLANNER_H
#define RRTPLANNER_H

#include "BasePlanner.h"

#include <random>

class RRTPlanner : public BasePlanner {
public:
    RRTPlanner(double step_size = 40.0, int max_iter = 10000)
        : step_size_(step_size), max_iter_(max_iter), gen(std::random_device{}()) {}

    std::vector<cv::Point> plan() override;

private:
    double step_size_;
    int max_iter_;

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
    std::vector<cv::Point> tracePath(const std::map<cv::Point, cv::Point, cmpPoints>& tree, const cv::Point& goal);
};
#endif //RRTPLANNER_H

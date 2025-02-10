//
// Created by mozihe on 25-2-7.
//

#ifndef DIJKSTRAPLANNER_H
#define DIJKSTRAPLANNER_H

#include <BasePlanner.h>
#include <opencv2/opencv.hpp>

class DijkstraPlanner : public BasePlanner {
public:
    std::vector<cv::Point> plan() override;

private:
    struct cmpPoints {
        bool operator()(const cv::Point &a, const cv::Point &b) const {
            return std::tie(a.x, a.y) < std::tie(b.x, b.y);
        }
    };

    struct ComparePairs {
        bool operator()(const std::pair<double, cv::Point>& a, const std::pair<double, cv::Point>& b) const {
            return a.first > b.first;
        }
    };

    bool isValid(const cv::Point &p, const cv::Mat &visited);
};

#endif //DIJKSTRAPLANNER_H

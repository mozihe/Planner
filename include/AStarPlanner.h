//
// Created by mozihe on 25-2-7.
//

#ifndef ASTARPLANNER_H
#define ASTARPLANNER_H

#include <BasePlanner.h>
#include <opencv2/opencv.hpp>

class AStarPlanner : public BasePlanner {
public:
    std::vector<cv::Point> plan() override;

private:
    struct cmpPoints {
        bool operator()(const cv::Point &a, const cv::Point &b) const {
            return std::tie(a.x, a.y) < std::tie(b.x, b.y);
        }
    };

    struct compareTuple {
        bool operator()(const std::tuple<double, double, cv::Point> &a,
                        const std::tuple<double, double, cv::Point> &b) const {
            return std::get<0>(a) > std::get<0>(b);
        }
    };

    bool isValid(const cv::Point &p, const cv::Mat &visited);

    static double heuristic(const cv::Point &a, const cv::Point &b);
};

#endif //ASTARPLANNER_H

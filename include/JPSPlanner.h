//
// Created by mozihe on 25-2-10.
//

#ifndef JPSPLANNER_H
#define JPSPLANNER_H

#include <BasePlanner.h>
#include <opencv2/opencv.hpp>

class JPSPlanner : public BasePlanner {
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

    std::map<cv::Point, cv::Point, cmpPoints> parent;
    std::map<cv::Point, double, cmpPoints> distance;

    bool isValid(const cv::Point &p, const cv::Mat &visited);

    static double heuristic(const cv::Point &a, const cv::Point &b);

    std::vector<std::tuple<double, double, cv::Point>> getJumpNeighbors(const cv::Point &p);

    bool isValidPoint(const cv::Point &p);

    bool hasForceNeighbor(const cv::Point &p, const cv::Point &dir);

    std::vector<cv::Point> getForceNeighbor(const cv::Point &p, const cv::Point &dir);

    cv::Point jump(const cv::Point &p, const cv::Point &dir);
};

#endif // JPSPLANNER_H

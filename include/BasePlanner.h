//
// Created by mozihe on 25-2-7.
//

#ifndef BASE_PLANNER_H
#define BASE_PLANNER_H

#include <opencv2/opencv.hpp>
#include <vector>

class BasePlanner {
public:
    BasePlanner() = default;
    virtual ~BasePlanner() = default;
    virtual std::vector<cv::Point> plan() = 0;
    void setMap(const cv::Mat &map);
    void setStart(const cv::Point &start);
    void setGoal(const cv::Point &goal);

protected:
    cv::Mat map_;
    cv::Point start_;
    cv::Point goal_;
};

#endif // BASE_PLANNER_H

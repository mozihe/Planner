//
// Created by mozihe on 25-2-7.
//

#ifndef DFS_PLANNER_H
#define DFS_PLANNER_H

#include <BasePlanner.h>

class DFSPlanner : public BasePlanner {
public:
    std::vector<cv::Point> plan() override;

private:
    struct cmpPoints {
        bool operator()(const cv::Point &a, const cv::Point &b) const {
            return std::tie(a.x, a.y) < std::tie(b.x, b.y);
        }
    };

    bool isValid(const cv::Point &p, const cv::Mat &visited);
};


#endif // DFS_PLANNER_H

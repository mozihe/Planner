//
// Created by mozihe on 25-2-14.
//

#ifndef MONKEYPLANNER_H
#define MONKEYPLANNER_H

#include "BasePlanner.h"

class MonkeyPlanner : public BasePlanner {
public:
    std::vector<cv::Point> plan() override;

    bool isValid(const cv::Point &p);
};


#endif // MONKEYPLANNER_H

//
// Created by mozihe on 25-2-14.
//

#ifndef GREEDYPLANNER_H
#define GREEDYPLANNER_H

#include "BasePlanner.h"

class GreedyPlanner : public BasePlanner {
public:

    std::vector<cv::Point> plan() override;
};



#endif //GREEDYPLANNER_H

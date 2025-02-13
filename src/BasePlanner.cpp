//
// Created by mozihe on 25-2-7.
//
#include <BasePlanner.h>

void BasePlanner::setMap(const cv::Mat &map) { map_ = map; }

void BasePlanner::setStart(const cv::Point &start) { start_ = start; }

void BasePlanner::setGoal(const cv::Point &goal) { goal_ = goal; }

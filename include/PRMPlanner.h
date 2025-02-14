//
// Created by mozihe on 25-2-14.
//

#ifndef PRMPLANNER_H
#define PRMPLANNER_H

#include <random>
#include "BasePlanner.h"
#include "KDTree.h"

namespace std {
    template<>
    struct hash<cv::Point> {
        size_t operator()(const cv::Point &p) const noexcept {
            size_t h1 = hash<int>{}(p.x);
            size_t h2 = hash<int>{}(p.y);
            return h1 ^ (h2 << 1);
        }
    };
} // namespace std


class PRMPlanner : public BasePlanner {
public:
    PRMPlanner(double target_density = 0.005, double factor = 3.0) : target_density_(target_density), factor_(factor) {}

    ~PRMPlanner() override = default;

    std::vector<cv::Point> plan() override;


private:
    struct PairCompare {
        bool operator()(const std::pair<double, cv::Point_<int>> &lhs,
                        const std::pair<double, cv::Point_<int>> &rhs) const {
            return lhs.first > rhs.first;
        }
    };

    struct PointPairCompare {
        bool operator()(const std::pair<cv::Point, cv::Point> &a, const std::pair<cv::Point, cv::Point> &b) const {
            if (a.first.x != b.first.x)
                return a.first.x < b.first.x;
            if (a.first.y != b.first.y)
                return a.first.y < b.first.y;
            if (a.second.x != b.second.x)
                return a.second.x < b.second.x;
            return a.second.y < b.second.y;
        }
    };

    int num_nodes_;
    int k_neighbors_;
    double max_distance_;

    double target_density_;
    double factor_;

    std::vector<cv::Point> nodes_;
    std::vector<std::pair<cv::Point, cv::Point>> edges_;

    void generateRoadmap();
    std::vector<cv::Point> findPath();
    double heuristic(const cv::Point &a, const cv::Point &b) const;
    bool isValid(const cv::Point &point) const;
    bool checkCollision(const cv::Point &start, const cv::Point &end) const;
};

#endif // PRMPLANNER_H

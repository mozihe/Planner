//
// Created by mozihe on 25-2-13.
//

#ifndef KDTREE_H
#define KDTREE_H

#include <opencv2/opencv.hpp>

struct KDNode {
    cv::Point point;
    KDNode* left;
    KDNode* right;

    KDNode(const cv::Point& pt) : point(pt), left(nullptr), right(nullptr) {}
};

class KDTree {
public:
    KDTree(const std::vector<cv::Point>& points);
    std::vector<cv::Point> kNearestNeighbors(const cv::Point& query, int k);
private:
    KDNode* root_;
    KDNode* buildTree(const std::vector<cv::Point>& points, int depth);
    void kNearestNeighbors(KDNode* node, const cv::Point& query, int k, int depth, std::vector<std::pair<cv::Point, double>>& neighbors);
    static bool compareDistance(const std::pair<cv::Point, double>& a, const std::pair<cv::Point, double>& b);
};

#endif //KDTREE_H

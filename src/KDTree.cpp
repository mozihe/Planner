//
// Created by mozihe on 25-2-13.
//

#include "KDTree.h"

KDTree::KDTree(const std::vector<cv::Point> &points) { root_ = buildTree(points, 0); }

std::vector<cv::Point> KDTree::kNearestNeighbors(const cv::Point &query, int k) {
    std::vector<std::pair<cv::Point, double>> neighbors;
    kNearestNeighbors(root_, query, k, 0, neighbors);
    std::vector<cv::Point> result;
    for (auto &n: neighbors) {
        result.push_back(n.first);
    }
    return result;
}

KDNode *KDTree::buildTree(const std::vector<cv::Point> &points, int depth) {
    if (points.empty())
        return nullptr;

    int axis = depth % 2;
    auto sorted_points = points;
    std::sort(sorted_points.begin(), sorted_points.end(),
              [axis](const cv::Point &a, const cv::Point &b) { return axis == 0 ? a.x < b.x : a.y < b.y; });

    int medianIdx = sorted_points.size() / 2;
    auto *node = new KDNode(sorted_points[medianIdx]);

    std::vector<cv::Point> left_points(sorted_points.begin(), sorted_points.begin() + medianIdx);
    std::vector<cv::Point> right_points(sorted_points.begin() + medianIdx + 1, sorted_points.end());

    node->left = buildTree(left_points, depth + 1);
    node->right = buildTree(right_points, depth + 1);

    return node;
}

void KDTree::kNearestNeighbors(KDNode *node, const cv::Point &query, int k, int depth,
                               std::vector<std::pair<cv::Point, double>> &neighbors) {
    if (node == nullptr)
        return;

    int axis = depth % 2;
    double dist = cv::norm(node->point - query);
    if (neighbors.size() < k) {
        neighbors.emplace_back(node->point, dist);
        std::ranges::push_heap(neighbors, compareDistance);
    } else if (dist < neighbors.front().second) {
        std::ranges::pop_heap(neighbors, compareDistance);
        neighbors.pop_back();
        neighbors.emplace_back(node->point, dist);
        std::ranges::push_heap(neighbors, compareDistance);
    }

    KDNode *nextNode = (axis == 0 ? query.x < node->point.x : query.y < node->point.y) ? node->left : node->right;
    KDNode *otherNode = (nextNode == node->left) ? node->right : node->left;

    kNearestNeighbors(nextNode, query, k, depth + 1, neighbors);

    if (neighbors.size() < k ||
        std::abs((axis == 0 ? query.x - node->point.x : query.y - node->point.y)) < neighbors.front().second) {
        kNearestNeighbors(otherNode, query, k, depth + 1, neighbors);
    }
}

bool KDTree::compareDistance(const std::pair<cv::Point, double> &a, const std::pair<cv::Point, double> &b) {
    return a.second > b.second;
}

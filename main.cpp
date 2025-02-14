#include "BasePlanner.h"

#include "AStarPlanner.h"
#include "BFSPlanner.h"
#include "DFSPlanner.h"
#include "DijkstraPlanner.h"
#include "JPSPlanner.h"
#include "PRMPlanner.h"
#include "GreedyPlanner.h"
#include "ExtendAStarPlanner.h"
#include "MonkeyPlanner.h"
#include "RRTPlanner.h"

bool rePlan = false;
cv::Point goal;

void onMouse(int event, int x, int y, int flags, void *userdata) {
    if (event == cv::EVENT_LBUTTONDOWN) {
        goal = cv::Point(x, y);
        std::cout << "Goal: " << goal << std::endl;
        rePlan = true;
    }
}

int main() {
    cv::Mat map = cv::imread("../RMUC_2024.png", cv::IMREAD_GRAYSCALE);
    if (map.empty()) {
        std::cerr << "Failed to load image" << std::endl;
        return -1;
    }


    cv::Point start(100, 144);
    BasePlanner *planner = new RRTPlanner();

    planner->setMap(map);
    planner->setStart(start);

    cv::namedWindow("Planner");
    cv::setMouseCallback("Planner", onMouse);

    cv::Mat display = map.clone();
    cv::cvtColor(display, display, cv::COLOR_GRAY2BGR);

    while (true) {

        cv::circle(display, start, 5, cv::Scalar(0, 255, 0), -1);
        cv::circle(display, goal, 5, cv::Scalar(0, 0, 255), -1);

        if (rePlan) {
            planner->setGoal(goal);
            std::chrono::high_resolution_clock::time_point begin = std::chrono::high_resolution_clock::now();
            auto path = planner->plan();
            std::chrono::high_resolution_clock::time_point end = std::chrono::high_resolution_clock::now();
            std::cout << "Time difference = "
                      << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]"
                      << std::endl;
            display = map.clone();
            cv::cvtColor(display, display, cv::COLOR_GRAY2BGR);
            std::cout << "Path length: " << path.size() << std::endl;
            for (const auto &point: path) {
                cv::circle(display, point, 1, cv::Scalar(255, 0, 0), -1);
            }
            rePlan = false;
        }

        cv::imshow("Planner", display);
        int key = cv::waitKey(1);
        if (key == 27) {
            break;
        }
    }

    return 0;
}

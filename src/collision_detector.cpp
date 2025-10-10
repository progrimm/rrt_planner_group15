#include <rrt_planner/collision_detector.h>
#include <cmath>  // for std::hypot and std::floor

namespace rrt_planner {

CollisionDetector::CollisionDetector(costmap_2d::Costmap2DROS* costmap_ros) {
    costmap_ = costmap_ros->getCostmap();
    resolution_ = costmap_->getResolution();
    origin_x_ = costmap_->getOriginX();
    origin_y_ = costmap_->getOriginY();
}

bool CollisionDetector::inFreeSpace(const double* world_pos) {
    unsigned int mx, my;
    if (!costmap_->worldToMap(world_pos[0], world_pos[1], mx, my)) {
        return false; // outside the map
    }
    return (costmap_->getCost(mx, my) == 0);
}

double CollisionDetector::computeDistance(const double* a, const double* b) {
    return std::hypot(b[0] - a[0], b[1] - a[1]);
}

bool CollisionDetector::obstacleBetween(const double* point_a, const double* point_b) {
    double dist = computeDistance(point_a, point_b);

    if (dist < resolution_) {
        return !inFreeSpace(point_b);
    }

    int num_steps = static_cast<int>(std::floor(dist / resolution_));
    double point_i[2];

    for (int n = 1; n <= num_steps; ++n) {
        point_i[0] = point_a[0] + n * (point_b[0] - point_a[0]) / num_steps;
        point_i[1] = point_a[1] + n * (point_b[1] - point_a[1]) / num_steps;
        if (!inFreeSpace(point_i)) {
            return true;
        }
    }
    return false;
}

} // namespace rrt_planner

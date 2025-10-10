
#include <rrt_planner/collision_detector.h>

namespace rrt_planner {

    CollisionDetector::CollisionDetector(costmap_2d::Costmap2DROS* costmap) {

        costmap_ = costmap->getCostmap();

        resolution_ = costmap_->getResolution();
        origin_x_ = costmap_->getOriginX();
        origin_y_ = costmap_->getOriginY();

    }

    bool CollisionDetector::inFreeSpace(const double* world_pos) {

        unsigned int mx, my;
        if (!costmap_->worldToMap(world_pos[0], world_pos[1], mx, my)) {
            return false; // outside the map
        }
        unsigned char cost = costmap_->getCost(mx, my);
        // Consider free if cost is less than inscribed inflated obstacle (253)
        // This allows navigation through cells with some cost but not lethal obstacles
        return (cost < costmap_2d::INSCRIBED_INFLATED_OBSTACLE);

    }

    bool CollisionDetector::obstacleBetween(const double* point_a, const double* point_b) {
        if (!inFreeSpace(point_a) || !inFreeSpace(point_b)) {
            return true; // treat points outside map as obstacles
        }

        double dist = computeDistance(point_a, point_b);
        if (dist < resolution_) {
            return !inFreeSpace(point_b);
        }

        int num_steps = static_cast<int>(std::floor(dist / resolution_));
        double point_i[2];

        for (int n = 1; n < num_steps; ++n) { // no need to check point_b again
            point_i[0] = point_a[0] + n * (point_b[0] - point_a[0]) / num_steps;
            point_i[1] = point_a[1] + n * (point_b[1] - point_a[1]) / num_steps;
            if (!inFreeSpace(point_i)) {
                return true;
            }
        }
        return false;
    }


};
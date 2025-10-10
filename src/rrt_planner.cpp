
#include <rrt_planner/rrt_planner.h>

namespace rrt_planner {

    RRTPlanner::RRTPlanner(costmap_2d::Costmap2DROS *costmap, 
            const rrt_params& params) : params_(params), collision_dect_(costmap) {

        costmap_ = costmap->getCostmap();
        map_width_  = costmap_->getSizeInMetersX();
        map_height_ = costmap_->getSizeInMetersY();

        random_double_x.setRange(-map_width_, map_width_);
        random_double_y.setRange(-map_height_, map_height_);
3
        nodes_.reserve(params_.max_num_nodes);
    }

    bool RRTPlanner::planPath() {

        // clear everything before planning
        nodes_.clear();

        // Start Node
        createNewNode(start_, -1);

        double *p_rand, *p_new;
        Node nearest_node;

        for (unsigned int k = 1; k <= params_.max_num_nodes; k++) {

            p_rand = sampleRandomPoint();
            nearest_node = nodes_[getNearestNodeId(p_rand)];
            p_new = extendTree(nearest_node.pos, p_rand); // new point and node candidate

            if (!collision_dect_.obstacleBetween(nearest_node.pos, p_new)) {
                createNewNode(p_new, nearest_node.node_id);

            } else {
                continue;
            }

            if(k > params_.min_num_nodes) {
                
                if(computeDistance(p_new, goal_) <= params_.goal_tolerance){
                    return true;
                }
            }
        }

        return false;
    }

    int RRTPlanner::getNearestNodeId(const double *point) {

        min_dist_node = 0;
        min_dist = std::numeric_limits<double>::max();
        for (int i = nodes_.size() - 1; i >= 0; i--) {
            dist = computeDistance(nodes_[i].pos, point);
            if (dist < min_dist) {
                min_dist = dist;
                min_dist_node = i;
            }

        }

        return min_dist_node;

    }

     double pos[2]; // 2D coordinates (x, y)
        int node_id;
        int parent_id;
        float cost_to_go{0.0};


    void RRTPlanner::createNewNode(const double* pos, int parent_node_id) {

        Node new_node;

        new_node.pos[0] = pos[0];
        new_node.pos[1] = pos[1];
        new_node.node_id = nodes_.size();
        new_node.parent_id = parent_node_id;

        nodes_.emplace_back(new_node);
        
    }

    double* RRTPlanner::sampleRandomPoint() {


        rand_point_[0] = random_double_x.generate();
        rand_point_[1] = random_double_y.generate();
        
        return rand_point_;
    }

    double* RRTPlanner::extendTree(const double* point_nearest, const double* point_rand) {

        /** 
        double dir_x = point_rand[0] - point_nearest[0];
        double dir_y = point_rand[1] - point_nearest[1];
        double dist = computeDistance(point_nearest, point_rand);

        if (dist < 1e-6) dist = 1e-6; // ikke dele på null
        dir_x /= dist;
        dir_y /= dist;

        double step = std::min(step_size_, dist);
        candidate_point_[0] = point_nearest[0] + step * dir_x;
        candidate_point_[1] = point_nearest[1] + step * dir_y;
        */
        candidate_point_[0] = point_rand[0];
        candidate_point_[1] = point_rand[1];

        return candidate_point_;
    }

    const std::vector<Node>& RRTPlanner::getTree() {

        return nodes_;
    }

    void RRTPlanner::setStart(double *start) {

        start_[0] = start[0];
        start_[1] = start[1];
    }

    void RRTPlanner::setGoal(double *goal) {

        goal_[0] = goal[0];
        goal_[1] = goal[1];
    }

};
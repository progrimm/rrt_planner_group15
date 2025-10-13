#include <rrt_planner/rrt_planner.h>
#include <limits>

namespace rrt_planner {

    RRTPlanner::RRTPlanner(costmap_2d::Costmap2DROS *costmap, 
            const rrt_params& params) : params_(params), collision_dect_(costmap) {

        costmap_ = costmap->getCostmap();
        map_width_  = costmap_->getSizeInMetersX();
        map_height_ = costmap_->getSizeInMetersY();

        random_double_x.setRange(-map_width_, map_width_);
        random_double_y.setRange(-map_height_, map_height_);
        nodes_.reserve(params_.max_num_nodes);
    }

    bool RRTPlanner::planPath() {

        // clear everything before planning
        nodes_.clear();

        // Start Node
        createNewNode(start_, -1, 0.0);

        double *p_rand, *p_new;
        Node nearest_node;

        for (unsigned int k = 1; k <= params_.max_num_nodes; k++) {

            p_rand = sampleRandomPoint();
            nearest_node = nodes_[getNearestNodeId(p_rand)];
            p_new = extendTree(nearest_node.pos, p_rand); // new point and node candidate

            if (!collision_dect_.obstacleBetween(nearest_node.pos, p_new)) {
                
                // Find nodes within radius
                std::vector<int> X_near = getNodesInRadius(p_new, params_.radius);
                
                // Find optimal parent (x_min)
                int x_min_id = nearest_node.node_id;
                double c_min = calculateNodeCost(nearest_node.node_id) + calculatePathCost(p_new, nearest_node.pos);
                
                for (int x_near_id : X_near) {
                    if (!collision_dect_.obstacleBetween(nodes_[x_near_id].pos, p_new)) {
                        double cost_through_x_near = calculateNodeCost(x_near_id) + calculatePathCost(p_new, nodes_[x_near_id].pos);
                        if (cost_through_x_near < c_min) {
                            x_min_id = x_near_id;
                            c_min = cost_through_x_near;
                        }
                    }
                }
                
                // Create new node with optimal parent and cost
                createNewNode(p_new, x_min_id, c_min);
                int new_node_id = nodes_.size() - 1;
                
                // Rewire nearby nodes
                for (int x_near_id : X_near) {
                    if (!collision_dect_.obstacleBetween(nodes_[x_near_id].pos, p_new)) {
                        double cost_through_new = calculateNodeCost(new_node_id) + calculatePathCost(p_new, nodes_[x_near_id].pos);
                        if (cost_through_new < calculateNodeCost(x_near_id)) {
                            nodes_[x_near_id].parent_id = new_node_id;
                            nodes_[x_near_id].cost_to_go = cost_through_new;
                        }
                    }
                }

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

        int min_dist_node = 0;
        double min_dist = std::numeric_limits<double>::max();
        double dist = 0.0;
        
        for (int i = static_cast<int>(nodes_.size()) - 1; i >= 0; --i) {
            dist = computeDistance(nodes_[i].pos, point);
            if (dist < min_dist) {
                min_dist = dist;
                min_dist_node = i;
            }
        }

        return min_dist_node;
    }

    std::vector<int> RRTPlanner::getNodesInRadius(const double* point, double radius) {
        std::vector<int> nodes_in_radius;
        for (int i = 0; i < nodes_.size(); ++i) {
            if (computeDistance(nodes_[i].pos, point) <= radius) {
                nodes_in_radius.push_back(i);
            }
        }
        return nodes_in_radius;
    }

    void RRTPlanner::createNewNode(const double* pos, int parent_node_id, double cost_to_go) {

        Node new_node;

        new_node.pos[0] = pos[0];
        new_node.pos[1] = pos[1];
        new_node.node_id = nodes_.size();
        new_node.parent_id = parent_node_id;
        new_node.cost_to_go = cost_to_go;

        nodes_.emplace_back(new_node);
    }

    double* RRTPlanner::sampleRandomPoint() {

        rand_point_[0] = random_double_x.generate();
        rand_point_[1] = random_double_y.generate();
        
        return rand_point_;
    }

    double RRTPlanner::calculatePathCost(const double* from, const double* to) const {
        // The cost is simply the Euclidean distance between points
        return computeDistance(from, to);
    }

    double RRTPlanner::calculateNodeCost(int node_id) const {
        if (node_id < 0 || node_id >= nodes_.size()) {
            return std::numeric_limits<double>::max();
        }

        // Return the stored cost-to-go (accumulated cost from start)
        return nodes_[node_id].cost_to_go;
    }

    double* RRTPlanner::extendTree(const double* point_nearest, const double* point_rand) {

        double dir_x = point_rand[0] - point_nearest[0];
        double dir_y = point_rand[1] - point_nearest[1];
        double dist = computeDistance(point_nearest, point_rand);

        if (dist < 1e-6) dist = 1e-6;
        dir_x /= dist;
        dir_y /= dist;

        double step = std::min(params_.step, dist);
        candidate_point_[0] = point_nearest[0] + step * dir_x;
        candidate_point_[1] = point_nearest[1] + step * dir_y;

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
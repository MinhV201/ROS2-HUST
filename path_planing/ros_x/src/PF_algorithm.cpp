#include "ros_x/PF_algorithm.h"
#include "ros_x/PathLibs.h"
using namespace path_following;


LosController::LosController() 
    : lookahead_dist_(1.0), last_closest_index_(0) 
{

}

void LosController::setLookAheadDist(double distance){
    this->lookahead_dist_ = distance;
}

void LosController::setPath(const std::vector<Point>& path){
    this->waypoints_ = path;
    this->reset();
}

LosOutput LosController::update(double robot_x, double robot_y) {
    LosOutput out;
    out.is_spot_turn = false; // Mặc định là di chuyển bình thường
    
    ::Point robot_pos = {robot_x, robot_y};

    if (path_.empty() || current_idx_ >= (int)path_.size()) {
        out.end_of_path = true;
        return out;
    }

    PathSegment* current_seg = path_[current_idx_];


    if (current_seg->type == SPIN) {
        SpinSegment* spin_seg = static_cast<SpinSegment*>(current_seg);
        
        out.is_spot_turn = true;
        out.target_heading = spin_seg->target_yaw;
        out.cross_track_error = 0.0;
        

        out.current_index = current_idx_;
        out.end_of_path = false;
        return out;
        

    } 
    else {
        
        if (current_seg->isFinished(robot_pos)) {
            current_idx_++;
            if (current_idx_ >= (int)path_.size()) {
                out.end_of_path = true;
                return out;
            }
            current_seg = path_[current_idx_];

            if (current_seg->type == SPIN) {
                SpinSegment* spin_seg = static_cast<SpinSegment*>(current_seg);
                out.is_spot_turn = true;
                out.target_heading = spin_seg->target_yaw;
                out.cross_track_error = 0.0;
                
                out.current_index = current_idx_;
                out.end_of_path = false;
                return out;
            }
        }
        
        double cross_track_error = current_seg->getCrossTrackError(robot_pos);
        double psi_path = current_seg->getPathHeading(robot_pos);
        double psi_los = std::atan(-cross_track_error / lookahead_dist_);
        out.target_heading = psi_path + psi_los;
        out.cross_track_error = cross_track_error;
    }

    out.current_index = current_idx_;
    out.end_of_path = false;
    return out;
}

void LosController::reset(){
    this->last_closest_index_ = 0;
}

void LosController::clearPath() {
    for (auto seg : path_) {
        delete seg;
    }
    path_.clear();
    current_idx_ = 0;
}

void LosController::buildTestPath() {
    clearPath();
    
    path_.push_back(new LineSegment({0.0, 0.0}, {5.0, 0.0}));
    
    path_.push_back(new ArcSegment({5.0, 0.0}, {8.0, 3.0}, 3.0, LEFT));
    
    path_.push_back(new LineSegment({8.0, 3.0}, {8.0, 10.0}));
}





#if !defined(PF_algorithm)
#define PF_algorithm

#include <cmath>
#include <vector>
#include <algorithm>
#include <limits>
#include "PathLibs.h"
namespace path_following{
    struct Point
    {
        double x;
        double y;
    };
    struct LosOutput{
        double target_heading;     
        double cross_track_error;   //y1
        int current_index;          
        bool end_of_path;
        bool is_spot_turn;
    };

    class LosController{
        public:
        LosController();
        ~LosController() = default;

        void setLookAheadDist(double distance);
        void setPath(const std::vector<Point>& path);

        LosOutput update(double robot_x, double robot_y);

        void buildTestPath();
        void clearPath();

        void reset();
        double normalizeAngle(double angle){
            while (angle > M_PI) angle -= 2.0 * M_PI;
            while (angle <= -M_PI) angle += 2.0 * M_PI;
            return angle;
        }
        void addSegment(PathSegment* seg) {
            path_.push_back(seg);
        }

        void forceNextSegment(){
            current_idx_++;
        }
        private:
        std::vector<Point> waypoints_;
        std::vector<PathSegment*> path_;
        double lookahead_dist_;       // Delta_h 
        int current_idx_ = 0;
        int last_closest_index_;
        double dist(const Point& p1, const Point& p2){
            return std::hypot(p1.x-p2.x, p1.y-p2.y);
        }
        Point getProjectionPoint(const Point& p_robot, const Point& p_start, const Point& p_end){
            double dx = p_end.x - p_start.x;
            double dy = p_end.y - p_start.y;
            
            double len_sq = dx * dx + dy * dy;

            if (len_sq < 1e-6) {
                return p_start;
            }

            double r_dx = p_robot.x - p_start.x;
            double r_dy = p_robot.y - p_start.y;

            
            double t = (r_dx * dx + r_dy * dy) / len_sq;

            if (t < 0.0) t = 0.0;
            if (t > 1.0) t = 1.0;

            // Tính toạ độ điểm chiếu H
            Point projection;
            projection.x = p_start.x + t * dx;
            projection.y = p_start.y + t * dy;

            return projection;
        }
        
    };
    
};



#endif // PF_algorithm

#include "ros_x/PathLibs.h"
#include <cmath>
#include <algorithm>

LineSegment::LineSegment(Point s, Point e)
    : PathSegment(s, e, LINE) {}

double LineSegment::getCrossTrackError(Point robot_pos) {
    double dx = end.x - start.x;
    double dy = end.y - start.y;
    double cross = (robot_pos.x - start.x) * dy - (robot_pos.y - start.y) * dx;
    return -cross / std::hypot(dx, dy);
}

double PathSegment::getPathHeading(Point robot_pos){
    (void)robot_pos;
    double dx = end.x - start.x;
    double dy = end.y - start.y;
    return std::atan2(dy, dx);
}

void ArcSegment::calculateCenter(){
    double d2 = std::pow(end.x - start.x, 2) + std::pow(end.y - start.y, 2);
    double d = std::sqrt(d2);

    if (2 * radius < d) {
        radius = d / 2.0;
    }

    // Tính toạ độ trung điểm M
    double mx = (start.x + end.x) / 2.0;
    double my = (start.y + end.y) / 2.0;

    // Khoảng cách từ trung điểm M đến tâm O
    double h = std::sqrt(std::max(0.0, radius * radius - d2 / 4.0));

    // Tính offset vector (vuông góc với đoạn start-end)
    double dx = end.x - start.x;
    double dy = end.y - start.y;

    // Logic xác định tâm dựa trên hướng quay
    if (direction == LEFT) { 
        center.x = mx - h * (dy / d);
        center.y = my + h * (dx / d);
    } else { 
        center.x = mx + h * (dy / d);
        center.y = my - h * (dx / d);
    }
}

double ArcSegment::getCrossTrackError(Point robot_pos) {
    double dist_to_center = std::hypot(robot_pos.x - center.x, robot_pos.y - center.y);
        
    double error = dist_to_center - radius;

    if (direction == LEFT) return error; 
    else return -error;
}

double ArcSegment::getPathHeading(Point robot_pos){
    double dx = robot_pos.x - center.x;
    double dy = robot_pos.y - center.y;
    double angle_from_center = std::atan2(dy, dx);

    double tangent_angle;
    if (direction == LEFT) {
        tangent_angle = angle_from_center + M_PI_2;
    } else {
        tangent_angle = angle_from_center - M_PI_2;
    }
    return tangent_angle;
}

bool LineSegment::isFinished(Point robot_pos){
    double dx = end.x - start.x;
    double dy = end.y - start.y;

    double len_sq = dx*dx + dy*dy;
    double dot = (robot_pos.x - start.x)*dx + (robot_pos.y - start.y)*dy;
    
    return (dot / len_sq) >= 1.0;
}

bool ArcSegment::isFinished(Point robot_pos){
    double dist_to_end = std::hypot(robot_pos.x - end.x, robot_pos.y - end.y);
    return dist_to_end < 0.5;
}
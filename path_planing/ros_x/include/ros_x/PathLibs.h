#if !defined(PathLibs)
#define PathLibs

#include <iostream>
#include <cmath>

struct Point{
    double x, y;
};
enum PathType{LINE, ARC, SPIN};
enum TurnDirection { LEFT, RIGHT };

class PathSegment{
    public:
    Point start;
    Point end;
    PathType type;

    PathSegment(Point s, Point e, PathType t) : start(s), end(e), type(t) {};
    virtual ~PathSegment() = default;

    virtual double getCrossTrackError(Point robot_pos) = 0;
    virtual bool isFinished(Point robot_pos) = 0;
    virtual double getPathHeading(Point robot_pos);
};

// ==========================================
// 1. Line Segment 
// ==========================================
class LineSegment : public PathSegment{
    public:
    LineSegment(Point s, Point e);
    double getCrossTrackError(Point robot_pos) override;
    bool isFinished(Point robot_pos) override;
};

// ==========================================
// 2. Arc Segment 
// ==========================================
class ArcSegment : public PathSegment{
    public:
    Point center;
    double radius;
    TurnDirection direction;

    ArcSegment(Point s, Point e, double r, TurnDirection dir) 
        : PathSegment(s, e, ARC), radius(r), direction(dir) 
    {
        calculateCenter();
    };

    void calculateCenter();
    double getCrossTrackError(Point robot_pos) override;
    bool isFinished(Point robot_pos) override;
    double getPathHeading(Point robot_pos) override;
};
// ==========================================
// 3. Spin Segment 
// ==========================================
class SpinSegment : public PathSegment {
public:
    double target_yaw;

    SpinSegment(Point p_start, Point p_end) 
        : PathSegment(p_start, p_end, SPIN) 
    {
        
        double dx = end.x - start.x;
        double dy = end.y - start.y;
        target_yaw = std::atan2(dy, dx);
    }

    double getCrossTrackError(Point robot_pos) override {
        (void)robot_pos;
        return 0.0;
    }


    bool isFinished(Point robot_pos) override {
        (void)robot_pos;
        return false; 
    }

    double getPathHeading(Point robot_pos) override {
        (void)robot_pos;
        return target_yaw; 
    }
};

#endif // PathLibs

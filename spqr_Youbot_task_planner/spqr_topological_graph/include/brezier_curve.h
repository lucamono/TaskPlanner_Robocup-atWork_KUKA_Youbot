#ifndef BREZIER_CURVE_H
#define BREZIER_CURVE_H

#include <iostream>
#include <Eigen/Geometry>

typedef Eigen::Matrix<double, 2, 1> Vector2;
typedef Eigen::Matrix<double, 2, 4> Coefficients;
typedef Eigen::Matrix<double, 1, 2> Row2;
typedef std::pair<Vector2, Vector2> Segment;

class BrezierCurve {
  private:
    int _num_points;
    double _interval;
    Row2 _vi;
    Row2 _vf;
    
    
    
    std::vector<Vector2> _waypoints_p;
    std::vector<Vector2> _waypoints_v;
    std::vector<Vector2> _waypoints_a;
    
    std::vector<Segment> _path_segments; //points
    std::vector<Vector2> _cubic_spline_p;
    std::vector<Vector2> _cubic_spline_a; // accelerations given from CublicSpline
    
    std::vector<Vector2> _brezier_p;
    std::vector<Vector2> _brezier_v;
    std::vector<Vector2> _brezier_a;
    
  //wv private methods
  private:
    void computePathSegments();
    void computeVelocitiesAtWaypoints();
    void computeAccelerationsAtWaypoints();
    void computeBrezierCurves();
    double getDistance(Vector2& g1, Vector2& g2);
    
  public:
    BrezierCurve(const std::vector<Vector2>& waypoints_p_, const std::vector<Vector2>& cubic_spline_p_, const std::vector<Vector2>& cubic_spline_a_,  const Vector2& vi_ = Row2(0.0,0.0), const Vector2& vf_ = Vector2(0.0,0.0), double interval_=0.01);
    
    inline void setInitVelocity(Row2& vi_){_vi = vi_;}
    inline void setEndVelocity( Row2& vf_){_vf = vf_;}
    inline void setTimeInterval(double interval_){_interval = interval_;}
    inline void setCubicSplinePositions(const std::vector<Vector2>& cubic_spline_p_){_cubic_spline_p = cubic_spline_p_;}
    inline void setCubicSplineAccelerations(const std::vector<Vector2>& cubic_spline_a_){_cubic_spline_a = cubic_spline_a_;}
    
    inline const std::vector<Vector2>& getPositionsAtWaypoints() const{return _waypoints_p;}
    inline const std::vector<Vector2>& getVelocitiesAtWaypoints() const {return _waypoints_v;}
    inline const std::vector<Vector2>& getAccelerationsAtWaypoints() const {return _waypoints_a;}
    
    inline const std::vector<Vector2>& getPositions() const{return _brezier_p;}
    inline const std::vector<Vector2>& getVelocities() const {return _brezier_v;}
    inline const std::vector<Vector2>& getAccelerations() const {return _brezier_a;}
    
    void compute();

};
#endif /* BREZIER_CURVE_H */
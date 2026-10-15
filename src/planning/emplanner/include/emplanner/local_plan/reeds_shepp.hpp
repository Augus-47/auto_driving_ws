#ifndef REEDS_SHEPP_HPP_
#define REEDS_SHEPP_HPP_

#include <cassert>
#include <functional>
#include <limits>

using ReedsSheppPathSamplingCallback = std::function<int(double q[3], void * user_data)>;
using ReedsSheppPathTypeCallback = std::function<int(int t, void * user_data)>;

class ReedsSheppStateSpace
{
public:
  /** \brief The Reeds-Shepp path segment types */
  enum ReedsSheppPathSegmentType { RS_NOP = 0, RS_LEFT = 1, RS_STRAIGHT = 2, RS_RIGHT = 3 };

  /** \brief Reeds-Shepp path types */
  static const ReedsSheppPathSegmentType reedsSheppPathType[18][5];

  /** \brief Complete description of a ReedsShepp path */
  class ReedsSheppPath
  {
  public:
    ReedsSheppPath(
      const ReedsSheppPathSegmentType * type = reedsSheppPathType[0],
      double t = std::numeric_limits<double>::max(), double u = 0., double v = 0., double w = 0.,
      double x = 0.);

    double length() const { return totalLength_; }

    /** Path segment types */
    const ReedsSheppPathSegmentType * type_;
    /** Path segment lengths */
    double length_[5];
    /** Total length */
    double totalLength_;
  };

  struct StateXYT
  {
    double x;
    double y;
    double yaw;
  };

  explicit ReedsSheppStateSpace(double turningRadius) : rho_(turningRadius) {}

  double distance(const StateXYT & s0, const StateXYT & s1) const;

  /** \brief Return the shortest Reeds-Shepp path from SE(2) state state1 to SE(2) state state2 */
  ReedsSheppPath reedsShepp(const StateXYT & s0, const StateXYT & s1) const;

  StateXYT interpolate(const StateXYT & s0, ReedsSheppPath & path, double seg) const;

  /** \brief Turning radius */
  double rho_;
};

#endif  // AUTOWARE__FREESPACE_PLANNING_ALGORITHMS__REEDS_SHEPP_HPP_

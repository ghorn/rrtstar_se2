#pragma once

#include "src/tree/tree_base.hpp"
#include "src/tagged.hpp"

namespace rrts {
  namespace tree {

    template <typename Point>
    class Naive : public TreeBase<Point> {
    public:
      Naive(const Point &, const Point &) {};
      ~Naive() = default;

      void Insert(const Tagged<Point> &new_point) {
        points_.push_back(new_point);
      }
    
      Tagged<Point> Nearest(const Point &test_point) const {
        assert(!points_.empty());

        Tagged<Point> nearest_point = points_.at(0);
        double nearest_distance_squared = nearest_point.point.DistanceSquared(test_point);

        for (size_t k=1; k<points_.size(); k++) {
          Tagged<Point> x = points_.at(k);
          double distance_squared = x.point.DistanceSquared(test_point);
          if (distance_squared < nearest_distance_squared) {
            nearest_distance_squared = distance_squared;
            nearest_point = x;
          }
        }

        return nearest_point;
      };
    
      std::vector<Tagged<Point>> Near(const Point &test_point, const double radius) const {
        const double radius2 = radius*radius;
        std::vector<Tagged<Point> > near_points;
        for (const Tagged<Point> &x : points_) {
          if (x.point.DistanceSquared(test_point) <= radius2) {
            near_points.push_back(x);
          }
        }
        return near_points;
      }
    
      double Cardinality() const {
        // TODO(greg): this might be wrong
        return static_cast<double>(points_.size());
      };

      //private:
      std::vector<Tagged<Point> > points_;
    };

  }  // namespace tree
}  // namespace rrts

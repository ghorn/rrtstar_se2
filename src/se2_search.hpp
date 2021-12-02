namespace rrts {

//template <class Point, class Line, class Tree>
//class Space {
//  
//}
template <>
class Algorithm<Se2::Point> {
public:
//  Algorithm(Se2::Point initial_point, double eta) : tree_(initial_point), eta_(eta) {
//    sphere_obstacles.push_back({{0.5, 0.5, 0}, 0.5});
//    sphere_obstacles.push_back({{0.25, 0.75, 0}, 0.3});
//  };
//  ~Algorithm(){};

private:
  Se2::Tree tree_;
//  const int d_ = 3;
//  const double d = static_cast<double>(d_);

  virtual Point SampleFree() = 0;
  virtual std::vector<Point> Near(Point) = 0;
  virtual Point Steer(Point, Point) = 0;
  virtual bool CollisionFree(Point, Point) = 0;
  virtual void InsertPoint(Point) = 0;

  std::mt19937_64 rng_engine;
  std::uniform_real_distribution<double> uniform_distribution;


}

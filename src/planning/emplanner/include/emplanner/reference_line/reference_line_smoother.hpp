#include "reference_line_smoother_config.hpp"
#include "reference_line.hpp"
#include <memory>
#include <vector>

class ReferenceLineSmoother {
public:
  explicit ReferenceLineSmoother(const ReferenceLineSmootherConfig &config);
  explicit ReferenceLineSmoother();
  ~ReferenceLineSmoother() = default;
  
  //参考线平滑函数
  void Smooth(const std::vector<MapPoint> &raw_reference_points, std::vector<MapPoint> &smoothed_reference_points);
              
private:

  // 平滑配置
  ReferenceLineSmootherConfig config_;
  //对2d曲线进行平滑处理
  bool DiscretePointsSmooth(const std::vector<std::pair<double, double>> &raw_point2d,
      std::vector<std::pair<double, double>> *ptr_smoothed_point2d);
};
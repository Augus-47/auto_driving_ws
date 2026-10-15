
class ReferenceLineSmootherConfig {
public:
  double weight_smooth = 70;
  double weight_path_length = 10;
  double weight_ref_deviation = 20;

  double x_lower_bound = -2;
  double x_upper_bound = 2;
  double y_lower_bound = -2;
  double y_upper_bound = 2;
};
// Example only using a single variable (x) minimize (1-x)^2 while providing a gradient.
// Note: one key feature of the Fuzzy-PSR is that the approach provides the error + the gradient at each point.
// Note2: this is unconstrainted. Maybe it makes sence to not allow the solution to wander off to far.

#include "ceres/ceres.h"
#include "glog/logging.h"
// f(x) = (1-x)^2;
class Test : public ceres::FirstOrderFunction {
 public:
  virtual ~Test() {}
  virtual bool Evaluate(const double* parameters,
                        double* cost,
                        double* gradient) const {
    const double x = parameters[0];
    cost[0] = (1.0 - x) * (1.0 - x);
    if (gradient != NULL) {
      gradient[0] = -2.0 * (1.0 - x);
    }
    return true;
  }
  virtual int NumParameters() const { return 1; }
};
int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  double parameters[1] = {-1.2 };
  ceres::GradientProblemSolver::Options options;
  options.minimizer_progress_to_stdout = true;
  ceres::GradientProblemSolver::Summary summary;
  ceres::GradientProblem problem(new Test());
  ceres::Solve(options, problem, parameters, &summary);
  std::cout << summary.FullReport() << "\n";
  std::cout << "Initial x: " << -1212345 << "\n";
  std::cout << "Final   x: " << parameters[0] << "\n";
  return 0;
}

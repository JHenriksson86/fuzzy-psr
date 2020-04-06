#include <Eigen/Core>
#include <Eigen/Geometry>

#include <iostream>
#include <iomanip>

using namespace std;

int main()
{
  Eigen::Vector3d center;

  std::vector<double> tmp;

  // Generate a set of clusters
  size_t nb_centers = 10;
  for (size_t i = 0; i < nb_centers; i++) {

    center(0) = i; center(1) = i; center(2) = i;

    tmp.push_back(center(0));
    tmp.push_back(center(1));
    tmp.push_back(center(2));
  }

  // Create a matrix respresentation...
  Eigen::MatrixXd matrix = Eigen::MatrixXd::Map(tmp.data(), 3, nb_centers);

  std::cout << "Matrix : \n" << matrix << std::endl;
}

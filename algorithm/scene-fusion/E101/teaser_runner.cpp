#include <fstream>
#include <iomanip>
#include <iostream>
#include <vector>

#include "teaser/registration.h"

int main(int argc, char **argv) {
    if (argc != 2) return 2;
    std::ifstream file(argv[1]);
    std::vector<Eigen::Vector3d> source, target;
    Eigen::Vector3d a, b;
    while (file >> a.x() >> a.y() >> a.z() >> b.x() >> b.y() >> b.z()) {
        source.push_back(a);
        target.push_back(b);
    }
    if (source.size() < 5) return 2;
    Eigen::Matrix<double, 3, Eigen::Dynamic> x(3, source.size()), y(3, target.size());
    for (size_t i = 0; i < source.size(); ++i) {
        x.col(i) = source[i];
        y.col(i) = target[i];
    }
    teaser::RobustRegistrationSolver::Params options;
    options.noise_bound = 2.;
    options.estimate_scaling = false;
    options.max_clique_num_threads = 1;
    options.max_clique_time_limit = 1;
    teaser::RobustRegistrationSolver solver(options);
    solver.solve(x, y);
    auto result = solver.getSolution();
    if (!result.valid) return 3;
    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    transform.block<3, 3>(0, 0) = result.rotation;
    transform.block<3, 1>(0, 3) = result.translation;
    std::cout << "E101_TRANSFORM\n" << std::setprecision(16) << transform << '\n';
}

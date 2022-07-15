#include <iostream>
#include <iomanip>
#include <fstream>

#include "calibration.hpp"
#include "lm.hpp"

using Functor = IntrinsicsFunctor<FisheyeCamera>;

int main(int argc, char ** argv)
{
    if (argc != 10)
    {
        std::cout << "Parameters: <patterns-file> <f_u> <f_v> <u_0> <v_0> <k_2> <k_3> <k_4> <k_5>" << std::endl;
        return 0;
    }

    std::ifstream in(argv[1]);
    cv::Size imageSize;
    Patterns patterns = createPatternsFromFile(in, imageSize);
    in.close();

    // calibration parameters
    double fu = std::atof(argv[2]);
    double fv = std::atof(argv[3]);
    double u0 = std::atof(argv[4]);
    double v0 = std::atof(argv[5]);
    double k2 = std::atof(argv[6]);
    double k3 = std::atof(argv[7]);
    double k4 = std::atof(argv[8]);
    double k5 = std::atof(argv[9]);

    // minimize
    Functor f(std::move(patterns));
    Functor::Vector x0;
    x0 << fu, fv, u0, v0, k2, k3, k4, k5;
    lm::Options<Functor> options;
    options.maxNumberOfIterations = 100;

    auto summary = lm::minimize(f, x0, options);

    std::cout << "Initial value: " << x0.transpose() << std::endl;
    std::cout << "Number of iterations: " << summary.numberOfIterations << std::endl;
    std::cout << "Number of evaluations: " << summary.numberOfEvaluations << std::endl;
    std::cout << "Value: " << summary.value << std::endl;
    std::cout << "Output: " << summary.solution.transpose() << std::endl;

    return 0;
}

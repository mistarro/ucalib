#include <iostream>
#include <iomanip>
#include <fstream>
#include <stdexcept>

#include "calibration.hpp"
#include "lm.hpp"

using Functor = IntrinsicsFunctor<FisheyeCamera>;

int main(int argc, char ** argv)
{
    if (argc != 3)
    {
        std::cout << "Parameters: <patterns-file> <fov-in-degrees>" << std::endl;
        return 0;
    }

    // create data
    std::ifstream in(argv[1]);
    double fov = std::atof(argv[2]);
    cv::Size imageSize;
    Patterns patterns = createPatternsFromFile(in, imageSize);
    in.close();

    // initial values
    double fu = imageSize.width*180.0/(fov*M_PI);
    double fv = imageSize.height*180.0/(fov*M_PI);
    double u0 = imageSize.width*0.5f;
    double v0 = imageSize.height*0.5f;

    // minimize
    Functor f(std::move(patterns));
    Functor::Vector x0;
    x0 << fu, fv, u0, v0, 0, 0, 0, 0;
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

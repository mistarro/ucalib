#include <iostream>
#include <iomanip>
#include <fstream>

#include "calibration.hpp"

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

    Functor::Vector x0;
    x0 << fu, fv, u0, v0, k2, k3, k4, k5;

    Functor f(std::move(patterns));
    auto y = f(x0);

    std::cout << "Argument: " << x0.transpose() << std::endl;
    std::cout << "Value: " << y.getValue() << std::endl;

    return 0;
}

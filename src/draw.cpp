#include <iostream>
#include <fstream>

#include "calibration.hpp"
#include "debug.hpp"

using Functor = IntrinsicsFunctor<FisheyeCamera>;

int main(int argc, char ** argv)
{
    if (argc != 16)
    {
        std::cout << "Parameters: <patterns-file> <f_u> <f_v> <u_0> <v_0> <k_2> <k_3> <k_4> <k_5> <X-axis-id> <Y-axis-id> <X-step> <Y-step> <X-resolution> <Y-resolution>" << std::endl;
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

    std::size_t const t1 = std::atoi(argv[10]);
    std::size_t const t2 = std::atoi(argv[11]);
    double const stepX = std::atof(argv[12]);
    double const stepY = std::atof(argv[13]);
    int const w = std::atoi(argv[14]);
    int const h = std::atoi(argv[15]);

    Functor::Vector x0;
    x0 << fu, fv, u0, v0, k2, k3, k4, k5;

    Functor f(std::move(patterns));

    draw(f, x0, t1, t2, stepX, stepY, w, h);

    return 0;
}

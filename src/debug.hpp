#ifndef DEBUG_HPP
#define DEBUG_HPP

#include <iostream>
#include <iomanip>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include "calibration.hpp"

template <typename IntrinsicsFunctor>
void checkGradient(IntrinsicsFunctor const & f, typename IntrinsicsFunctor::Vector const & x0, double h)
{
    using Vector = typename IntrinsicsFunctor::Vector;

    auto y0 = f(x0);

    Vector numericGradient;
    for (std::size_t t = 0; t < IntrinsicsFunctor::Dimension; ++t)
    {
        Vector x1 = x0, x2 = x0;
        x1(t) -= h;
        x2(t) += h;
        auto y1 = f(x1);
        auto y2 = f(x2);
        numericGradient(t) = (y2.getValue() - y1.getValue())/(h + h);
    }

    Vector diff = numericGradient - y0.getGradient();
    std::cout << "Gradient:\n";
    std::cout << "  symbolic:\n" << y0.getGradient() << std::endl;
    std::cout << "  numeric:\n" << numericGradient << std::endl;
    std::cout << "  diff:\n" << diff << std::endl;
}

template <typename IntrinsicsFunctor>
void checkHessian(IntrinsicsFunctor const & f, typename IntrinsicsFunctor::Vector const & x0, double h)
{
    using Vector = typename IntrinsicsFunctor::Vector;
    using Matrix = typename IntrinsicsFunctor::Matrix;

    auto y0 = f(x0);

    Matrix numericHessian;
    for (std::size_t t = 0; t < IntrinsicsFunctor::Dimension; ++t)
    {
        Vector x1 = x0, x2 = x0;
        x1(t) -= h;
        x2(t) += h;
        auto y1 = f(x1);
        auto y2 = f(x2);
        numericHessian.col(t) = (y2.getGradient() - y1.getGradient())/(h + h);
    }

    Matrix diff = numericHessian - y0.getHessian();
    std::cout << "Hessian:\n";
    std::cout << "  symbolic:\n" << y0.getHessian() << std::endl;
    std::cout << "  numeric:\n" << numericHessian << std::endl;
    std::cout << "  diff:\n" << diff << std::endl;
    Eigen::SelfAdjointEigenSolver<Matrix> eigensolver(y0.getHessian());
    std::cout << "  eigenvalues: " << eigensolver.eigenvalues().transpose() << std::endl;
    std::cout << "VALUE: " << y0.getValue() << std::endl;
}

template <typename IntrinsicsFunctor>
void draw(IntrinsicsFunctor const & f, typename IntrinsicsFunctor::Vector const & x0, std::size_t t1, std::size_t t2, double stepX, double stepY, int w, int h)
{
    using Vector = typename IntrinsicsFunctor::Vector;
    using Matrix = typename IntrinsicsFunctor::Matrix;

    cv::Mat values(h, w, CV_32FC1);
    cv::Mat slope(h, w, CV_32FC1);
    cv::Mat convexity(h, w, CV_32FC1);

    cv::Mat image(h, w, CV_32FC1);

    double vmin = std::numeric_limits<double>::infinity();
    double xmin, ymin;

    for (int i = 0; i < w; ++i)
    for (int j = 0; j < h; ++j)
    {
        cv::Point pixel(i, j);
        Vector x = x0;
        x(t1) += (i - w/2)*stepX;
        x(t2) += (j - h/2)*stepY;
        auto y = f(x);
        double v = y.getValue();
        if (vmin > v)
        {
            vmin = v;
            xmin = x(t1);
            ymin = x(t2);
        }

        values.at<float>(pixel) = y.getValue();
        slope.at<float>(pixel) = y.getGradient().norm();
        Matrix H = y.getHessian();
        Eigen::SelfAdjointEigenSolver<Matrix> eigensolver(H);
        convexity.at<float>(pixel) = eigensolver.eigenvalues().coeff(0);
    }


    double min, max;
    cv::minMaxLoc(values, &min, &max);
    image = (values - min)/(max - min);
    std::cout << "min: " << min << "\nmax: " << max << std::endl;
    std::cout << "min at: " << cv::Point2d(xmin, ymin) << std::endl;

    cv::minMaxLoc(slope, &min, &max);
    cv::Mat slope_img(h, w, CV_8UC3);
    slope_img = (slope - min)/(max - min);
    std::cout << "min slope: " << min << std::endl;
    std::cout << "max slope: " << max << std::endl;

    cv::minMaxLoc(convexity, &min, &max);
    cv::Mat convexity_img(h, w, CV_8UC3);
    std::cout << "max convexity: " << max << std::endl;
    convexity /= max;
    for (int i = 0; i < w; ++i)
    for (int j = 0; j < h; ++j)
    {
        cv::Point pixel(i, j);
        float c = convexity.at<float>(pixel);
        if (c < 0.0f)
            convexity_img.at<cv::Vec3b>(pixel) = cv::Vec3b(255, 0, 0);
        else
            convexity_img.at<cv::Vec3b>(pixel) = cv::Vec3b(0, uchar(c*255.0), 0);
    }

    cv::resize(image, image, cv::Size(), 10.0, 10.0, cv::INTER_NEAREST);
    cv::resize(slope_img, slope_img, cv::Size(), 10.0, 10.0, cv::INTER_NEAREST);
    cv::resize(convexity_img, convexity_img, cv::Size(), 10.0, 10.0, cv::INTER_NEAREST);
    cv::imshow("f", image);
    cv::imshow("f slope", slope_img);
    cv::imshow("f convexity", convexity_img);
    cv::waitKey(0);
}

#endif // DEBUG_HPP

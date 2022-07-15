#ifndef CALIBRATION_HPP
#define CALIBRATION_HPP

#include <iostream>
#include <vector>
#include <stdexcept>

#ifdef EIGEN_NO_DEBUG
#undef EIGEN_NO_DEBUG
#endif

#include <opencv2/core.hpp>
#include <Eigen/Dense>

#include "lm.hpp"
#include "pattern.hpp"

template <std::size_t D, typename S>
struct FunctorTypes
{
    static constexpr std::size_t Dimension = D;
    using Scalar = S;
    using Vector = Eigen::Matrix<Scalar, Dimension, 1>;
    using Matrix = Eigen::Matrix<Scalar, Dimension, Dimension>;

    using DerivativeMatrix = Eigen::Matrix<Scalar, 3, Dimension>;
    using Matrix3 = Eigen::Matrix<Scalar, 3, 3>;
    using Vector3 = Eigen::Matrix<Scalar, 3, 1>;
};

struct PinholeCamera : FunctorTypes<4, double>
{
    static void computeForPoint(Vector const & x, cv::Point2f const & point, Vector3 & m, DerivativeMatrix & md)
    {
        Scalar inv_fu = 1.0f/x(0);
        Scalar inv_fv = 1.0f/x(1);
        Scalar xx = (point.x - x(2))*inv_fu;
        Scalar yy = (point.y - x(3))*inv_fv;

        m << xx, yy, 1;
        Scalar s = m.norm();
        m /= s; // normalize

        Scalar xx2 = xx*xx;
        Scalar yy2 = yy*yy;

        md << -xx*(yy2 + 1)*inv_fu, xx*yy2*inv_fv, -(yy2 + 1)*inv_fu, xx*yy*inv_fv,
              xx2*yy*inv_fu, -(xx2 + 1)*yy*inv_fv, xx*yy*inv_fu, -(xx2 + 1)*inv_fv,
             xx2*inv_fu, yy2*inv_fv, xx*inv_fu, yy*inv_fv;

        md /= s*s*s;
    }
};

struct FisheyeCamera : FunctorTypes<8, double>
{
    static void computeForPoint(Vector const & x, cv::Point2f const & point, Vector3 & m, DerivativeMatrix & md)
    {
        Scalar inv_fu = 1.0f/x(0);
        Scalar inv_fv = 1.0f/x(1);
        Scalar xx = (point.x - x(2))*inv_fu;
        Scalar yy = (point.y - x(3))*inv_fv;
        Scalar r = std::sqrt(xx*xx + yy*yy);

        if (r < 1.0e-8)
        {
            m << 0, 0, 1;
            md << 0, 0, -inv_fu, 0, 0, 0, 0, 0,
                  0, 0, 0, -inv_fv, 0, 0, 0, 0,
                  0, 0, 0, 0, 0, 0, 0, 0;
        }
        else
        {
            Scalar theta = h(x, r);
            Scalar cos_theta = std::cos(theta);
            Scalar sin_theta = std::sin(theta);
            Scalar cos_phi = xx/r;
            Scalar sin_phi = yy/r;
            m << cos_phi*sin_theta, sin_phi*sin_theta, cos_theta;
            m.normalize();

            // derivatives
            Scalar dtheta_dr = 1.0/dr_dtheta(x, theta);

            Scalar theta2 = theta*theta;
            Scalar theta3 = theta*theta2;
            Scalar theta5 = theta3*theta2;
            Scalar theta7 = theta5*theta2;
            Scalar theta9 = theta7*theta2;

            // f_u
            md.col(0) <<
                -( sin_phi*sin_phi*sin_theta/r + cos_phi*cos_phi*dtheta_dr*cos_theta)*inv_fu*xx,
                -(-cos_phi*sin_phi*sin_theta/r + cos_phi*sin_phi*dtheta_dr*cos_theta)*inv_fu*xx,
                -(                -cos_phi*dtheta_dr*sin_theta                      )*inv_fu*xx,
            // f_v
            md.col(1) <<
                -(-cos_phi*sin_phi*sin_theta/r + sin_phi*cos_phi*dtheta_dr*cos_theta)*inv_fv*yy,
                -( cos_phi*cos_phi*sin_theta/r + sin_phi*sin_phi*dtheta_dr*cos_theta)*inv_fv*yy,
                -(                -sin_phi*dtheta_dr*sin_theta                      )*inv_fv*yy,
            // u_0
            md.col(2) <<
                -( sin_phi*sin_phi*sin_theta/r + cos_phi*cos_phi*dtheta_dr*cos_theta)*inv_fu,
                -(-cos_phi*sin_phi*sin_theta/r + cos_phi*sin_phi*dtheta_dr*cos_theta)*inv_fu,
                -(                -cos_phi*dtheta_dr*sin_theta                      )*inv_fu,
            // v_0
            md.col(3) <<
                -(-cos_phi*sin_phi*sin_theta/r + cos_phi*sin_phi*dtheta_dr*cos_theta)*inv_fv,
                -( cos_phi*cos_phi*sin_theta/r + sin_phi*sin_phi*dtheta_dr*cos_theta)*inv_fv,
                -(                -sin_phi*dtheta_dr*sin_theta                      )*inv_fv,
            // k_2
            md.col(4) <<
                -theta3*dtheta_dr*cos_phi*cos_theta,
                -theta3*dtheta_dr*sin_phi*cos_theta,
                 theta3*dtheta_dr*sin_theta,
            // k_3
            md.col(5) <<
                -theta5*dtheta_dr*cos_phi*cos_theta,
                -theta5*dtheta_dr*sin_phi*cos_theta,
                 theta5*dtheta_dr*sin_theta,
            // k_4
            md.col(6) <<
                -theta7*dtheta_dr*cos_phi*cos_theta,
                -theta7*dtheta_dr*sin_phi*cos_theta,
                 theta7*dtheta_dr*sin_theta,
            // k_5
            md.col(7) <<
                -theta9*dtheta_dr*cos_phi*cos_theta,
                -theta9*dtheta_dr*sin_phi*cos_theta,
                 theta9*dtheta_dr*sin_theta;
        }
    }

private:
    static Scalar rr(Vector const & x, Scalar theta)
    {
        Scalar theta2 = theta*theta;
        return theta*(1.0 + theta2*(x(4) + theta2*(x(5) + theta2*(x(6) + theta2*x(7)))));
    }

    static Scalar dr_dtheta(Vector const & x, Scalar theta)
    {
        Scalar theta2 = theta*theta;
        return 1.0 + theta2*(3.0*x(4) + theta2*(5.0*x(5) + theta2*(7.0*x(6) + theta2*9.0*x(7))));
    }

    // returns theta for given r
    static Scalar h(Vector const & x, Scalar r)
    {
        // theta = solve r = t + k2*t^3 + ... + k5*t^9 wrt t
        Scalar theta = r; // initial guess
        std::size_t const num_iter = 100;
        Scalar const eps = 1.0e-16;
        for (std::size_t i = 0; i < num_iter; ++i)
        {
            Scalar value = rr(x, theta) - r;
            if (std::abs(value) < eps)
                break;
            theta -= value/dr_dtheta(x, theta);
        }

        return theta;
    }
};

template <typename CameraModel>
struct IntrinsicsFunctor
{
    static constexpr std::size_t Dimension = CameraModel::Dimension;
    using Scalar = typename CameraModel::Scalar;
    using Vector = typename CameraModel::Vector;
    using Matrix = typename CameraModel::Matrix;

    explicit IntrinsicsFunctor(Patterns && patterns) : patterns(std::move(patterns)) {}
    IntrinsicsFunctor(IntrinsicsFunctor &)  = delete;
    IntrinsicsFunctor(IntrinsicsFunctor &&) = default;
    IntrinsicsFunctor & operator=(IntrinsicsFunctor &)  = delete;
    IntrinsicsFunctor & operator=(IntrinsicsFunctor &&) = default;

    class ProxyResult
    {
    public:
        explicit ProxyResult(Vector const & x, Patterns const & patterns)
        {
            if (!patterns.empty())
                result = compute(x, patterns);
            else
                throw std::logic_error("No pattern provided.");
        }

        Scalar getValue() const { return result.value; }
        Vector getGradient() const { return result.gradient; }
        Matrix getHessian() const { return result.hessian; }

        void print() const { result.print(); }

    private:
        struct Result
        {
            Scalar value;
            Vector gradient;
            Matrix hessian;

            Result() : value(0.0), gradient(Vector::Zero()), hessian(Matrix::Zero()) {}

            void operator+=(Result const & that)
            {
                value += that.value;
                gradient += that.gradient;
                hessian += that.hessian;
            }

            void operator/=(Scalar s)
            {
                value /= s;
                gradient /= s;
                hessian /= s;
            }

            void print() const
            {
                std::cout << "VALUE:    " << value << std::endl;
                std::cout << "GRADIENT: " << gradient.transpose() << std::endl;
                std::cout << "HESSIAN:\n" << hessian << std::endl;
            }
        };

        Result result;

        using DerivativeMatrix = typename CameraModel::DerivativeMatrix;
        using Matrix3 = typename CameraModel::Matrix3;
        using Vector3 = typename CameraModel::Vector3;

        static Result compute(Vector const & x, Patterns const & patterns)
        {
            Result result;

            for (auto const & pattern : patterns)
            {
                Result rPattern = computeForPattern(x, pattern);
                result += rPattern;
            }

            //result /= patterns.size();
            return result;
        }

        static Result computeForPattern(Vector const & x, Pattern const & pattern)
        {
            Vector3 l0, l1;
            DerivativeMatrix l0d, l1d;

            Result result_down = computeForBundle(x, pattern[0], l0, l0d);
            result_down += computeForBundle(x, pattern[1], l1, l1d);

            Result result;

            // value
            Scalar l0_l1 = l0.dot(l1);
            result.value = (2.0*l0_l1)*l0_l1;

            // gradient
            Vector t = l0d.transpose()*l1 + l1d.transpose()*l0;
            result.gradient = l0_l1*t;

            // hessian
            result.hessian = (2.0*t)*t.transpose();

            result += result_down;

            return result;
        }

        static Result computeForBundle(Vector const & x, Bundle const & bundle, Vector3 & l, DerivativeMatrix & ld)
        {
            Matrix3 N = Matrix3::Zero();
            std::vector<Matrix3> Nt(Dimension, Matrix3::Zero());
            std::vector<Matrix3> Ntt(Dimension*Dimension, Matrix3::Zero());

            Result result_down;
            for (auto const & line : bundle)
            {
                Vector3 n;
                DerivativeMatrix nd;

                result_down += computeForLine(x, line, n, nd);

                N += n*n.transpose();

                for (std::size_t j = 0; j < Dimension; ++j)
                {
                    Matrix3 A = n*nd.col(j).transpose();
                    Nt[j] += (A + A.transpose());

                    for (std::size_t k = j; k < Dimension; ++k)
                        Ntt[j + Dimension*k] += nd.col(j)*nd.col(k).transpose();
                }
            }

            Scalar mu, mu1, mu2;
            Vector3 l1, l2;
            eigenSolve(N, mu, mu1, mu2, l, l1, l2);

            Scalar s1 = Scalar(1.0)/(mu - mu1);
            Scalar s2 = Scalar(1.0)/(mu - mu2);

            DerivativeMatrix Nt_l;

            for (std::size_t j = 0; j < Dimension; ++j)
            {
                Nt_l.col(j) = Nt[j]*l;
                ld.col(j) = (s1*l1.dot(Nt_l.col(j)))*l1 + (s2*l2.dot(Nt_l.col(j)))*l2;
            }

            // set results for this direction
            Result result;
            result.value = mu;
            result.gradient = Nt_l.transpose()*l;

            for (std::size_t j = 0; j < Dimension; ++j)
            for (std::size_t k = j; k < Dimension; ++k)
                result.hessian(j, k) = result.hessian(k, j) = l.dot(Ntt[j + Dimension*k]*l);

            result.hessian += ld.transpose()*Nt_l;

            result.hessian *= 2.0;

            result += result_down;

            //result /= bundle.size();

            return result;
        }

        static Result computeForLine(Vector const & x, Line const & line, Vector3 & n, DerivativeMatrix & nd)
        {
            Matrix3 M = Matrix3::Zero();
            std::vector<Matrix3> Mt(Dimension, Matrix3::Zero());
            std::vector<Matrix3> Mtt(Dimension*Dimension, Matrix3::Zero());

            for (auto const & point : line)
            {
                Vector3 m;
                DerivativeMatrix md_temp;
                CameraModel::computeForPoint(x, point, m, md_temp);
                M += m*m.transpose();

                for (std::size_t j = 0; j < Dimension; ++j)
                {
                    Matrix3 A = m*md_temp.col(j).transpose();
                    Mt[j] += (A + A.transpose());

                    for (std::size_t k = j; k < Dimension; ++k)
                        Mtt[j + Dimension*k] += md_temp.col(j)*md_temp.col(k).transpose();
                }
            }

            Scalar lambda, lambda1, lambda2;
            Vector3 n1, n2;
            eigenSolve(M, lambda, lambda1, lambda2, n, n1, n2);

            DerivativeMatrix Mt_n;

            Scalar s1 = Scalar(1.0)/(lambda - lambda1);
            Scalar s2 = Scalar(1.0)/(lambda - lambda2);

            for (std::size_t j = 0; j < Dimension; ++j)
            {
                Mt_n.col(j) = Mt[j]*n;
                nd.col(j) = (s1*n1.dot(Mt_n.col(j)))*n1 + (s2*n2.dot(Mt_n.col(j)))*n2;
            }

            // set results for this direction
            Result result;
            result.value = lambda;
            result.gradient = Mt_n.transpose()*n;

            for (std::size_t j = 0; j < Dimension; ++j)
            for (std::size_t k = j; k < Dimension; ++k)
                result.hessian(j, k) = result.hessian(k, j) = n.dot(Mtt[j + Dimension*k]*n);

            result.hessian += nd.transpose().eval()*Mt_n;

            result.hessian *= 2.0;

            //result /= line.size();

            return result;
        }

        // A has to be symmetric
        static void eigenSolve(Matrix3 const & A, Scalar & a0, Scalar & a1, Scalar & a2, Vector3 & v0, Vector3 & v1, Vector3 & v2)
        {
            Eigen::SelfAdjointEigenSolver<Matrix3> eigensolver(A);
            v0 = eigensolver.eigenvectors().col(0);
            v1 = eigensolver.eigenvectors().col(1);
            v2 = eigensolver.eigenvectors().col(2);
            a0 = eigensolver.eigenvalues().coeff(0);
            a1 = eigensolver.eigenvalues().coeff(1);
            a2 = eigensolver.eigenvalues().coeff(2);
        }
    };

    ProxyResult operator()(Vector const & x) const { return ProxyResult(x, patterns); }

private:
    Patterns patterns;
};

Patterns createPatternsFromFile(std::istream & in, cv::Size & imageSize);

#endif // CALIBRATION_HPP

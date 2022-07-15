#ifndef LM_HPP
#define LM_HPP

#include <iostream>
#include <Eigen/Dense>

// reporters for info/debugging
struct NullReporter
{
    template <typename T>
    NullReporter const & operator<<(T const &) const { return *this; }

    NullReporter const & operator<<(std::ostream & (*)(std::ostream &)) const { return *this; }
    NullReporter const & operator<<(std::ostream & (*)(std::ios &)) const { return *this; }
    NullReporter const & operator<<(std::ostream & (*)(std::ios_base &)) const { return *this; }
};

struct StderrReporter
{
    template <typename T>
    StderrReporter const & operator<<(T const & item) const { std::cerr << item; return *this; }

    StderrReporter const & operator<<(std::ostream & (*f)(std::ostream &)) const { f(std::cerr); return *this; }
    StderrReporter const & operator<<(std::ostream & (*f)(std::ios &)) const { f(std::cerr); return *this; }
    StderrReporter const & operator<<(std::ostream & (*f)(std::ios_base &)) const { f(std::cerr); return *this; }
};

namespace lm {

// Helper class for functors
template <std::size_t D, typename S>
struct FunctorBase
{
    static constexpr std::size_t Dimension = D;
    using Scalar = S;
    using Vector = Eigen::Matrix<Scalar, Dimension, 1>;
    using Matrix = Eigen::Matrix<Scalar, Dimension, Dimension>;
};

// LMFunctor has to define (Vector, Matrix and ProxyResult are types defined below):
//   * static constexpr std::size_t LMFunctor::Dimension member (dimension of the domain)
//   * LMFunctor::Scalar member type (e.g. as float)
//   * LMFunctor::ProxyResult member type with following API:
//     * Scalar LMFunctor::ProxyResult::getValue() const
//     * Vector LMFunctor::ProxyResult::getGradient() const
//     * Matrix LMFunctor::ProxyResult::getHessian() const
//   * LMFunctor::ProxyResult LMFunctor::operator()(Vector const &) const member function
//   Easy way to do so is to derive LMFunctor from FunctorBase<Dimension, Scalar>
//   and then define LMFunctor::ProxyResult type as in the simple example below.
template <typename LMFunctor>
class Traits
{
public:
    static constexpr std::size_t Dimension = LMFunctor::Dimension;

    using Scalar = typename LMFunctor::Scalar;
    using Vector = Eigen::Matrix<Scalar, Dimension, 1>;
    using Matrix = Eigen::Matrix<Scalar, Dimension, Dimension>;

    using ProxyResult = typename LMFunctor::ProxyResult;
};

template <typename LMFunctor>
struct Options
{
    using Scalar = typename LMFunctor::Scalar;

    std::size_t maxNumberOfIterations;
    Scalar lambda_initial;
    Scalar lambda_multiplier;
    Scalar lambda_max;

    Options() :
        maxNumberOfIterations(50),
        lambda_initial(0.001),
        lambda_multiplier(10.0),
        lambda_max(1.0e10)
    {}
};

template <typename LMFunctor>
struct Summary
{
    std::size_t numberOfIterations;
    std::size_t numberOfEvaluations;
    typename Traits<LMFunctor>::Vector solution;
    typename Traits<LMFunctor>::Scalar value;
};

template <typename LMFunctor, typename Reporter = NullReporter>
Summary<LMFunctor> minimize(LMFunctor const & f, typename Traits<LMFunctor>::Vector const & x0, Options<LMFunctor> const & options = Options<LMFunctor>())
{
    using Scalar = typename Traits<LMFunctor>::Scalar;
    using Vector = typename Traits<LMFunctor>::Vector;
    using Matrix = typename Traits<LMFunctor>::Matrix;
    using ProxyResult = typename LMFunctor::ProxyResult;

    Reporter reporter;

    Vector x = x0;

    ProxyResult current = f(x);
    Scalar y = current.getValue();

    Scalar lambda = options.lambda_initial;

    reporter << "Initial argument: " << x0.transpose() << std::endl;
    reporter << "Initial value: " << y << std::endl;
    reporter << "Max number of iterations: " << options.maxNumberOfIterations << std::endl;

    std::size_t k = 0;
    std::size_t evaluations = 1;

    for (; k < options.maxNumberOfIterations; ++k)
    {
        reporter << "ITERATION " << k << std::endl;
        reporter << "Argument: " << x.transpose() << std::endl;
        reporter << "Value: " << y << std::endl;

        // compute gradient
        Vector g = current.getGradient();
        // compute hessian
        Matrix H = current.getHessian();

        reporter << "Gradient: " << g.transpose() << std::endl;
        reporter << "Hessian:\n" << H << std::endl;

        while (true)
        {
            if (lambda > options.lambda_max) // prevent infinite loop near minimum
                goto done;

            // solve (H + lambda*diag(H))*dx = -g for dx
            Matrix A = H;
            A.diagonal() *= (1.0 + lambda);
            Vector dx = A.ldlt().solve(-g);
            reporter << "  lambda: " << lambda << std::endl;
            reporter << "  correction: " << dx.transpose() << std::endl;

            Vector xnew = x + dx;
            ProxyResult candidate = f(xnew);
            ++evaluations;
            Scalar ynew = candidate.getValue();
            Scalar dy = y - ynew;
            reporter << "  new argument: " << xnew.transpose() << std::endl;
            reporter << "  new value: " << ynew << std::endl;
            if (dy > 0.0) // accept
            {
                x = std::move(xnew);
                y = std::move(ynew);
                current = std::move(candidate);
                lambda /= options.lambda_multiplier;
                break;
            }
            else
            {
                lambda *= options.lambda_multiplier;
            }
        }
    }

done:

    reporter << "Number of iterations: " << k << std::endl;
    reporter << "Number of evaluations: " << evaluations << std::endl;
    reporter << "Argument: " << x.transpose() << std::endl;
    reporter << "Value: " << y << std::endl;

    Summary<LMFunctor> summary;
    summary.numberOfIterations = k;
    summary.numberOfEvaluations = evaluations;
    summary.solution = x;
    summary.value = y;

    return summary;
}

} // namespace lm

#endif // LM_HPP

#if 0
// simple example
#include lm.hpp

struct MyLMFunctor : lm::FunctorBase<2, float>
{
    Scalar const a2, b2, x0, y0;

    MyLMFunctor(Scalar a, Scalar b, Scalar x0, Scalar y0) : a2(a*a), b2(b*b), x0(x0), y0(y0) {}

    class ProxyResult
    {
    public:
        explicit ProxyResult(Vector const & x, MyLMFunctor const * f) : x(x), f(f) {}

        Scalar getValue() const
        {
            Scalar dx = x(0)*x(0) - f->x0;
            Scalar dy = x(1)*x(1) - f->y0;
            return f->a2*dx*dx + f->b2*dy*dy;
        }

        Vector getGradient() const
        {
            Vector j;
            j << 4.0f*f->a2*(x(0)*x(0) - f->x0)*x(0), 4.0f*f->b2*(x(1)*x(1) - f->y0)*x(1);
            return j;
        }

        Matrix getHessian() const
        {
            Matrix H;
            H << 4.0f*f->a2*(3.0f*x(0)*x(0) - f->x0), 0.0f,
                 0.0f, 4.0f*f->b2*(3.0f*x(1)*x(1) - f->y0);
            return H;
        }

    private:
        Vector x;
        MyLMFunctor const * f;
    };

    friend class ProxyResult;

    ProxyResult operator()(Vector const & x) const { return ProxyResult(x, this); }
};

int main()
{
    lm::Traits<MyLMFunctor>::Vector x0;
    x0 << 0.f, 0.1f;
    std::cout << "Solution:\n" << lm::minimize(MyLMFunctor(1.0f, 1.0f, 2.0f, 3.0f), x0).solution << std::endl;

    x0 << 0.f, -0.1f;
    lm::Options<MyLMFunctor> options;
    options.maxNumberOfIterations = 20;
    // ...
    std::cout << "Solution:\n" << lm::minimize<MyLMFunctor, StderrReporter>(MyLMFunctor(1.0f, 1.0f, 2.0f, 3.0f), x0, options).solution << std::endl;

    return 0;
}
#endif // example

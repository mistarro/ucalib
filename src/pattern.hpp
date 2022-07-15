#ifndef PATTERN_HPP
#define PATTERN_HPP

#include <vector>
#include <array>
#include <opencv2/core.hpp>

using Line = std::vector<cv::Point2d>;
using Bundle = std::vector<Line>;
using Pattern = std::array<Bundle, 2>;
using Patterns = std::vector<Pattern>;

#endif // PATTERN_HPP

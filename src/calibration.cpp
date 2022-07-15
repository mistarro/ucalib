#include <iostream>

#include <opencv2/core.hpp>

#include "pattern.hpp"

Patterns createPatternsFromFile(std::istream & in, cv::Size & imageSize)
{
    std::size_t patternNum;
    std::size_t lineNum;
    std::size_t pointNum;
    cv::Point2d point;

    in >> imageSize.width >> imageSize.height;
    in >> patternNum;

    Patterns patterns;
    patterns.reserve(patternNum);

    for (std::size_t k = 0; k < patternNum; ++k)
    {
        Pattern pattern;

        for (std::size_t d = 0; d < 2; ++d)
        {
            in >> lineNum;
            pattern[d].reserve(lineNum);
            for (std::size_t l = 0; l < lineNum; ++l)
            {
                in >> pointNum;
                Line line;
                line.reserve(pointNum);
                for (std::size_t i = 0; i < pointNum; ++i)
                {
                    in >> point.x >> point.y;
                    line.push_back(point);
                }
                pattern[d].push_back(std::move(line));
            }

        }

        // append to patterns
        patterns.emplace_back(std::move(pattern));
    }

    return patterns;
}

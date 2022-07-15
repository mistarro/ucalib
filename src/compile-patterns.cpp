#include <iostream>
#include <iomanip>
#include <string>
#include <fstream>

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>

#include "pattern.hpp"

bool findChessboardCorners(cv::Mat const & image, cv::Size patternSize, std::vector<cv::Point2f> & corners)
{
    bool patternFound = cv::findChessboardCorners(image, patternSize, corners,
            cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE);

    if (patternFound)
    {
        cv::cornerSubPix(image, corners, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));
        return true;
    }
    else
    {
        return false;
    }
}

Patterns createPatternsFromImages(std::string const & namePrefix, cv::Size patternSize, cv::Size & imageSize, bool const visualCheck = false)
{
    Patterns patterns;
    imageSize = cv::Size(-1, -1);

    for (std::size_t k = 0; ; ++k)
    {
        std::stringstream ss;
        ss << namePrefix << std::setfill('0') << std::setw(3) << k << ".png";
        std::string fname = ss.str();
        std::cerr << "Searching for file " << fname << "... ";
        cv::Mat image = cv::imread(fname, cv::IMREAD_GRAYSCALE);
        if (image.empty())
        {
            std::cerr << "not found\n";
            break;
        }

        std::cerr << "found" << std::endl;

        if (imageSize.width > 0 && imageSize != image.size())
        {
            std::cerr << "  Image size does not match, skipping.\n";
            continue;
        }
        else
            imageSize = image.size();

        std::cerr << "  Detecting chessboard... ";

        // process image
        std::vector<cv::Point2f> corners;
        bool patternFound = findChessboardCorners(image, patternSize, corners);

        if (patternFound)
        {
            std::cerr << "ok.\n";
        }
        else
        {
            std::cerr << "failed, skipping.\n";
            continue;
        }

#if 0 // allows for visual inspection
        cv::Mat imageRGB;
        cv::cvtColor(image, imageRGB, CV_GRAY2BGR);
        cv::drawChessboardCorners(imageRGB, patternSize, cv::Mat(corners), patternfound);
        cv::imshow("Corners", imageRGB);
        int key = cv::waitKey(0);
        if (key == 27) // exit
            break;
        if (key == 's') // skip
            continue;
#endif

        std::cerr << "  Image processed successfully.\n";
        Pattern pattern;
        pattern[0].resize(patternSize.height);
        for (int j = 0; j < patternSize.height; ++j)
            pattern[0][j].resize(patternSize.width);
        pattern[1].resize(patternSize.width);
        for (int i = 0; i < patternSize.width; ++i)
            pattern[1][i].resize(patternSize.height);
        // fill the pattern
        for (int i = 0; i < patternSize.width; ++i)
        for (int j = 0; j < patternSize.height; ++j)
        {
            pattern[0][j][i] = corners[i + j*patternSize.width];
            pattern[1][i][j] = corners[i + j*patternSize.width];
        }

        // append to patterns
        patterns.emplace_back(std::move(pattern));
    }

    return patterns;
}

void outputPatterns(Patterns const & patterns, cv::Size const & imageSize, std::ostream & out = std::cout)
{
    // image size: W H
    out << imageSize.width << ' ' << imageSize.height << "\n\n";

    // number of patterns
    out << patterns.size() << "\n\n";

    // for each pattern
    for (auto const & pattern : patterns)
    {
        // 'horizontal'/'vertical' bundle
        for (std::size_t d = 0; d < 2; ++d)
        {
            // number of lines in the bundle
            out << "  " << pattern[d].size() << "\n\n";
            // for each line in the bundle
            for (auto const & line : pattern[d])
            {
                // number of points in the line
                out << "    " << line.size() << "\n";
                // the points
                for (auto const & point : line)
                    out << std::setprecision(10) << std::setw(10) << "    " << point.x << ' ' << point.y << std::endl;
                out << std::endl;
            }
        }
    }
}

int main(int argc, char ** argv)
{
    if (argc != 4)
    {
        std::cerr << "Parameters: <fielname-prefix> <pattern-width> <pattern-height>" << std::endl;
        return 0;
    }

    // create data
    int pw = std::atoi(argv[2]);
    int ph = std::atoi(argv[3]);
    cv::Size imageSize;
    Patterns patterns = createPatternsFromImages(argv[1], cv::Size(pw, ph), imageSize);

    if (patterns.size() == 0)
    {
        std::cerr << "No patterns detected.\n";
        return 0;
    }

    std::cerr << "Finished analyzing images.\nWriting to file.\n";

    // output to stdout
    outputPatterns(patterns, imageSize);

    std::cerr << "Done.\n";

    return 0;
}

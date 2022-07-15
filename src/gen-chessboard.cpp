#include <iostream>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

uchar getColor(int row, int col, int marginSize, int SquareSize, int chessboardX, int chessboardY)
{
    col -= marginSize;
    row -= marginSize;
    if (col < 0 || col >= chessboardX || row < 0 || row >= chessboardY)
        return 255;
    col /= SquareSize;
    row /= SquareSize;
    return 255*((col + row) & 1);
}

int main(int argc, char ** argv)
{
    if (argc != 5)
    {
        std::cerr << "Parameters: <margin-size-px> <square-size-px> <corners-x> <corners-y>" << std::endl;
        return 0;
    }

    // create data
    int marginSize = std::atoi(argv[1]);
    int squareSize = std::atoi(argv[2]);
    int cx = std::atoi(argv[3]);
    int cy = std::atoi(argv[4]);

    int chessboardX = (cx + 1)*squareSize;
    int chessboardY = (cy + 1)*squareSize;

    int imgW = 2*marginSize + chessboardX;
    int imgH = 2*marginSize + chessboardY;

    cv::Mat image(imgH, imgW, CV_8UC1);

    for (int row = 0; row < imgH; ++row)
        for (int col = 0; col < imgW; ++col)
            image.at<uchar>(row, col) = getColor(row, col, marginSize, squareSize, chessboardX, chessboardY);

    cv::imshow("chessboard", image);
    cv::waitKey(0);

    return 0;
}

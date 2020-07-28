#ifndef HELPER_H
#define HELPER_H

#include<opencv2/core/core.hpp>

namespace ORB_SLAM2
{

class Helper
{
public:
    std::vector<int> MatUnique(const cv::Mat &input, bool sorted);
    std::string type2str(int type);
    cv::Mat RowSliceMat(const cv::Mat &input, std::vector<int> &rows);

};

}// namespace ORB_SLAM

#endif // HELPER_H

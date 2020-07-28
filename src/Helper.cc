#include "Helper.h"

using namespace std;

namespace ORB_SLAM2
{
vector<int> Helper::MatUnique(const cv::Mat &input, bool sorted)
{

    vector<int> out;
    for(int y = 0; y < input.rows; y++)
    {
        for(int x = 0; x < input.cols; x++)
        {
            int val = input.at<uchar>(y,x);
            if(find(out.begin(), out.end(), val) == out.end() )
            {
                if(val <= 255)
                {
                    out.push_back(val);
                }
            }
        }
    }
    if(sorted)
        sort(out.begin(),out.end());
    return out;
}

cv::Mat Helper::RowSliceMat(const cv::Mat &input, vector<int> &rows)
{
    int nrows = rows.size();
    int ncols = input.cols;

    cv::Mat output(nrows,ncols,input.type());
    for(int i = 0; i < nrows; i++)
    {
        int rowIndex = rows[i];
        cv::Mat desiredRow = input.row(rowIndex);
        input.row(rowIndex).copyTo(output.row(i));
    }
    return output;
}


}
#include "inpainting.hpp"

cv::Rect getWindowRoi(const cv::Size map_size, const cv::Size window_size, const cv::Size source_image_size, cv::Point roi_position)
{
    assert(0 <= roi_position.x && roi_position.x < map_size.width);
    assert(0 <= roi_position.y && roi_position.y < map_size.height);

    assert(map_size.width * window_size.width <= source_image_size.width);
    assert(map_size.height * window_size.height <= source_image_size.height);

    double col_step = (double)(source_image_size.width - window_size.width)/(double)(map_size.width-1);
    double row_step = (double)(source_image_size.height- window_size.height)/(double)(map_size.height-1);
    cv::Rect2i roi(col_step * roi_position.x, row_step * roi_position.y, window_size.width, window_size.height);
    return roi;
}

cv::Mat generateInpaintingMap(const cv::Size map_size, const cv::Size window_size, const cv::Mat source, const cv::Mat target)
{
    cv::Mat map(map_size,CV_32FC2);
    for(int row=0;row<map_size.height;++row)
    {
        for(int col=0;col<map_size.width;++col)
        {
            cv::Rect roi = getWindowRoi(map_size,window_size,source.size(),cv::Point(col,row));
            cv::Point2d shift = cv::phaseCorrelate(source(roi),target(roi));
            map.at<cv::Point2f>(row,col) = shift;            
        }
    }
    return map;
}

void visualizeInpaintingMap(cv::Mat &source, const cv::Size window_size, const cv::Mat inpainting_map)
{
    cv::Size map_size = inpainting_map.size();
    for(int row=0;row<map_size.height;++row)
    {
        for(int col=0;col<map_size.width;++col)
        {
            cv::Rect roi = getWindowRoi(map_size,window_size,source.size(),cv::Point(col,row));
            cv::Point start(roi.x+roi.width/2,roi.y+roi.height/2);
            cv::Point2f shift = inpainting_map.at<cv::Point2f>(row,col);
            cv::Point end = cv::Point(shift) + start;
            std::cout << "start:" << start << " end:" << end << std::endl;
            cv::line(source,start,end,cv::Scalar(0,0,255),2);    
            cv::rectangle(source,roi,cv::Scalar(0,0,255),2);
            cv::rectangle(source,cv::Rect(roi.x+shift.x,roi.y+shift.y,roi.width,roi.height),cv::Scalar(127,cv::saturate_cast<uint8_t>(shift.y/roi.height*1000.f+127),cv::saturate_cast<uint8_t>(shift.x/roi.width*1000.f+127)),2);        
        }
    }
}
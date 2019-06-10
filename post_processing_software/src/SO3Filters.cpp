#include "SO3Filters.h"


void gradientLimit(Eigen::VectorXd &input){
    if(input.rows()<2) return;
    double limited_value = input.head(1)[0];
    for(int i=1,e=input.rows();i<e;++i){
        if(input(i) > limited_value - maximum_gradient_){
            limited_value = input(i);
        }else{
            limited_value -= maximum_gradient_;
            input(i) = limited_value;
        }
    }
    limited_value = input.tail(1)[0];
    for(int i=input.rows()-2;i>=0;--i){
        if(input(i) > limited_value - maximum_gradient_){
            limited_value = input(i);
        }else{
            limited_value -= maximum_gradient_;
            input(i) = limited_value;
        }
    }
}

/**
     * @brief ワープした時に欠けがないかチェックします
     * @retval false:欠けあり true:ワープが良好
     **/
template <typename _Tp>
static bool isPerfectWarp(vector<_Tp> &contour)
{
    for (int i = 0; i < contour.size(); i += 2)
    {
        if ((abs(contour[i]) < 1.0) && (abs(contour[i + 1]) < 1.0))
        {
            return false;
        }
    }
    return true;
}

bool hasBlackSpace(int32_t filter_strength, int32_t frame)
{
    std::vector<float> vecPorigonn_uv;
    Eigen::Quaternion<double> prevQ;
    Eigen::Quaternion<double> currQ;
    Eigen::Quaternion<double> nextQ;
    if (0 == frame)
    {
        currQ = toDiffQuaternion2(filter_strength, frame);
        prevQ = currQ;
        nextQ = toDiffQuaternion2(filter_strength, frame + 1);
    }
    else if ((raw_quaternion.rows() - 1) == frame)
    {
        prevQ = toDiffQuaternion2(filter_strength, frame - 1);
        currQ = toDiffQuaternion2(filter_strength, frame);
        nextQ = currQ;
    }
    else
    {
        prevQ = toDiffQuaternion2(filter_strength, frame - 1);
        currQ = toDiffQuaternion2(filter_strength, frame);
        nextQ = toDiffQuaternion2(filter_strength, frame + 1);
    }
    getDistortUnrollingContour(
        prevQ,
        currQ,
        nextQ,
        vecPorigonn_uv);
    return !isPerfectWarp(vecPorigonn_uv);
}

uint32_t bisectionMethod(int32_t frame, int32_t minimum_filter_strength, int32_t maximum_filter_strength, int max_iteration, uint32_t eps)
{
    int32_t a = minimum_filter_strength;
    int32_t b = maximum_filter_strength;
    int count = 0;
    int32_t m;
    //    while(hasBlackSpace(maximum_filter_strength,frame)){
    //        minimum_filter_strength = maximum_filter_strength;
    //        maximum_filter_strength *= 2;
    //    }
    while ((abs(a - b) > eps) && (count++ < max_iteration))
    {
        m = (a + b) * 0.5;
        if (hasBlackSpace(a, frame) ^ hasBlackSpace(m, frame))
        {
            b = m;
        }
        else
        {
            a = m;
        }
        if (count == max_iteration)
        {
            std::cout << "max_iteration" << std::endl;
        }
    }
    return m;
}

Eigen::VectorXd getFilterCoefficients(int32_t minimum_filter_strength, int32_t maximum_filter_strength)
{
    Eigen::VectorXd filter_strength(raw_quaternion.rows());
    //Calcurate in all frame
    for (int frame = 0, e = filter_strength.rows(); frame < e; ++frame)
    {
        if (hasBlackSpace(maximum_filter_strength, frame))
        {
            filter_strength[frame] = maximum_filter_strength;
        }
        else if (!hasBlackSpace(minimum_filter_strength, frame))
        {
            filter_strength[frame] = minimum_filter_strength;
        }
        else
        {
            filter_strength[frame] = bisectionMethod(frame, minimum_filter_strength, maximum_filter_strength);
        }
    }
    //    std::cout << filter_strength << std::endl;
    gradientLimit(filter_strength);

    return (filter_strength);
}
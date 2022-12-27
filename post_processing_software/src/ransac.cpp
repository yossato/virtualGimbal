/*************************************************************************
*  Software License Agreement (BSD 3-Clause License)
*  
*  Copyright (c) 2022, Yoshiaki Sato
*  All rights reserved.
*  
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions are met:
*  
*  1. Redistributions of source code must retain the above copyright notice, this
*     list of conditions and the following disclaimer.
*  
*  2. Redistributions in binary form must reproduce the above copyright notice,
*     this list of conditions and the following disclaimer in the documentation
*     and/or other materials provided with the distribution.
*  
*  3. Neither the name of the copyright holder nor the names of its
*     contributors may be used to endorse or promote products derived from
*     this software without specific prior written permission.
*  
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
*  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
*  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
*  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
*  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
*  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
*  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*************************************************************************/

#include <random>
#include <Eigen/Dense>

Eigen::VectorXd least_squares_method(Eigen::VectorXd &x, Eigen::VectorXd &y, int degree)
{
    int N = degree + 1;

    assert(x.size() == y.size());
    
    Eigen::MatrixXd A(N, N);

    std::vector<double> sum;
    for (int order = 0,e=2 * degree + 1; order < e; ++order)
    {
        sum.push_back(x.array().pow((double)order).sum());
    }

    for (int col = 0; col < N; ++col)
    {
        for (int row = 0; row < N; ++row)
        {
            A(row, col) = sum[row + col];
        }
    }
    Eigen::VectorXd B(N);
    for (int i = 0; i < N; ++i)
    {
        B(i) = (x.array().pow((double)i) * y.array()).sum();
    }

    return A.inverse() * B;
}

Eigen::VectorXd calculateLinearEquationCoefficientsRansac(std::vector<double> x, std::vector<double> y, int32_t iteration, double max_error_ )
    {
        int max_inlier = 0;
        double a_best, b_best;

        std::random_device seed_gen;
        std::mt19937 engine(seed_gen());
        std::uniform_int_distribution<uint64_t> get_rand_uni_int(0, y.size() - 1);
        for (int n = 0; n < iteration; ++n)
        {
            int i0 = get_rand_uni_int(engine);
            int i1 = get_rand_uni_int(engine);
            if(i0==i1)
            {
                continue;
            }

            double a = (y[i1] - y[i0]) / (x[i1] - x[i0]);
            double b = y[i1] - a * x[i1];

            // Count a number of inlier
            int inlier = 0;
            for (size_t i = 0; i < y.size(); ++i)
            {
                double diff = std::fabs(a * x[i] + b - y[i]);
                if (diff <= max_error_)
                {
                    ++inlier;
                }
            }
            if (inlier > max_inlier)
            {
                max_inlier = inlier;
                a_best = a;
                b_best = b;
            }
        }

        if (max_inlier == 0)
        {
            printf("Maximum inlier is zero. @ %s %d", __FILE__, __LINE__);
            Eigen::VectorXd retval(2);
            retval << std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN();
            return retval;
        }

        // Extract inlers
        Eigen::VectorXd x_in(max_inlier), y_in(max_inlier);
        size_t inlier_index = 0;
        for (size_t i = 0; i < y.size(); ++i)
        {
            double diff = std::fabs(a_best * x[i] + b_best - y[i]);
            if (diff <= max_error_)
            {
                x_in[inlier_index] = (x[i]);
                y_in[inlier_index] = (y[i]);
                ++inlier_index;
            }
        }

        // Get the best coeffs
        Eigen::VectorXd coeffs = least_squares_method(x_in, y_in, 1);
        printf("RANSAC - Inlier:%d / %lu coeffs[0]:%f coeffs[1]%f\r\n", max_inlier, y.size(), coeffs[0], coeffs[1]);
        return coeffs;
    }
#ifndef FREQUENCY_DOMAIN_OPTIMIZATION_HPP
#define FREQUENCY_DOMAIN_OPTIMIZATION_HPP

#include "Eigen/Dense"
#include "unsupported/Eigen/NonLinearOptimization"
#include "unsupported/Eigen/NumericalDiff"
#include "vsp.h"

// Generic functor
template<typename _Scalar, int NX=Dynamic, int NY=Dynamic>
struct Functor
{
  typedef _Scalar Scalar;
  enum {
    InputsAtCompileTime = NX,
    ValuesAtCompileTime = NY
  };
  typedef Matrix<Scalar,InputsAtCompileTime,1> InputType;
  typedef Matrix<Scalar,ValuesAtCompileTime,1> ValueType;
  typedef Matrix<Scalar,ValuesAtCompileTime,InputsAtCompileTime> JacobianType;

  Functor() : inputs_(InputsAtCompileTime), values_(ValuesAtCompileTime){}
  Functor(int inputs, int values) : inputs_(inputs), values_(values) {}

  int inputs() const { return inputs_; }
  int values() const { return values_; }

  const int inputs_;
  const int values_;


};

/**
 * @brief 平滑化済み波形を周波数領域で調整することで、なめらか、かつ、画面が欠けないように、時間波形調整します\
 *        x,y,zの軸全てに関して最適化します
 */
template <typename _Tp> struct FrequencyDomainOptimizer : Functor<double>
{
    /**
      * @brief コンストラクタ。
      * @param [in] inputs 最適化の対象となるパラメータの数、周波数軸での係数の個数
      * @param [in] values 最適化に使用できるサンプルデータの数、おそらく、時間軸での係数の個数
      * @param [in] angle 角度。raw_angleをリファレンスとして用いる
      **/
    FrequencyDomainOptimizer(int inputs, int values, vsp &angle)
        : angle_(angle),Functor(inputs, values) {
        vsp::Angle2CLerpedFrequency(angle_.fs,angle_.fc,angle_.filteredDataDFT(),frequency_vector_);
    }

    vsp &angle_;
    Eigen::MatrixXcd frequency_vector_;

    std::complex<double> a;
//    VectorXd getInitial

    int operator()(const VectorXd& complex_frequency_coefficients, VectorXd& fvec) const
    {

        assert(complex_frequency_coefficients.rows() < frequency_vector_.rows());
        //complex_frequency_coefficientsを受け取り、frequency_vector_の一部に詰め替える
        //x,y,zの3chあるからちゃんと扱おう！
        for(int32_t i=0,e=complex_frequency_coefficients.rows()/2;i<e;i+=2){
            frequency_vector_[i].real() = complex_frequency_coefficients[i];
            frequency_vector_[i].imag() = complex_frequency_coefficients[i+1];
        }
        //周波数領域 -> 時間波形に変換、angle_に時間波形の情報を戻す
        vsp::Frequency2Angle(frequency_vector_,angle_.filteredDataDFT());

        //末尾の余白を削除
        Eigen::MatrixXd buf = angle_.filteredDataDFT().block(0,0,angle_.data().rows(),angle_.data().cols());
        angle_.filteredDataDFT() = buf;

        //getRollingVectorErrorでエラーを取得する、fvecに詰め込む
        fvec = angle_.getRollingVectorError();

    }
}

#endif // FREQUENCY_DOMAIN_OPTIMIZATION_HPP

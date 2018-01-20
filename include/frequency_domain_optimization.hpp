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
 * @brief 平滑化済み波形を周波数領域で調整することで、なめらか、かつ、画面が欠けないように、時間波形調整します
 */
template <typename _Tp> struct FrequencyDomainOptimizer : Functor<double>
{
    /**
      * @brief コンストラクタ。
      * @param [in] inputs 最適化の対象となるパラメータの数、周波数軸での係数の個数
      * @param [in] values 最適化に使用できるサンプルデータの数、おそらく、時間軸での係数の個数
      * @param [in]
      **/
    FrequencyDomainOptimizer(int inputs, int values, vsp &angles)
}

#endif // FREQUENCY_DOMAIN_OPTIMIZATION_HPP

/**
 * @file NoiseModelFactorCustom.h
 * @authors Prashanth Ramadoss
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef KINDYNVIO_FACTORS_GTSAM_NOISE_MODEL_FACTOR_CUSTOM_H
#define KINDYNVIO_FACTORS_GTSAM_NOISE_MODEL_FACTOR_CUSTOM_H

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>

namespace gtsam
{

/* ************************************************************************* */
/** A convenient base class for creating your own NoiseModelFactor with 10
 * variables.  To derive from this class, implement evaluateError(). */
template<class VALUE1, class VALUE2, class VALUE3, class VALUE4, class VALUE5,
         class VALUE6, class VALUE7, class VALUE8, class VALUE9, class VALUE10>
class NoiseModelFactor10: public NoiseModelFactor {

public:

  // typedefs for value types pulled from keys
  using X1  = VALUE1;
  using X2  = VALUE2;
  using X3  = VALUE3;
  using X4  = VALUE4;
  using X5  = VALUE5;
  using X6  = VALUE6;
  using X7  = VALUE7;
  using X8  = VALUE8;
  using X9  = VALUE9;
  using X10 = VALUE10;

protected:
  using Base = NoiseModelFactor;
  using This = NoiseModelFactor10<VALUE1, VALUE2, VALUE3, VALUE4, VALUE5,
                                  VALUE6, VALUE7, VALUE8, VALUE9, VALUE10>;

public:
  NoiseModelFactor10() {}
  NoiseModelFactor10(const SharedNoiseModel& noiseModel, Key j1, Key j2, Key j3, Key j4, Key j5,
                     Key j6, Key j7, Key j8, Key j9, Key j10) :
    Base(noiseModel, cref_list_of<10>(j1)(j2)(j3)(j4)(j5)(j6)(j7)(j8)(j9)(j10)) {}

  ~NoiseModelFactor10() override {}

  /** methods to retrieve keys */
  inline Key key1() const { return keys_[0]; }
  inline Key key2() const { return keys_[1]; }
  inline Key key3() const { return keys_[2]; }
  inline Key key4() const { return keys_[3]; }
  inline Key key5() const { return keys_[4]; }
  inline Key key6() const { return keys_[5]; }
  inline Key key7() const { return keys_[6]; }
  inline Key key8() const { return keys_[7]; }
  inline Key key9() const { return keys_[8]; }
  inline Key key10() const { return keys_[9]; }

  /** Calls the 10-key specific version of evaluateError, which is pure virtual
   * so must be implemented in the derived class. */
  gtsam::Vector unwhitenedError(const Values& x, boost::optional<std::vector<gtsam::Matrix>&> H = boost::none) const override {
    if(this->active(x)) {
      if(H)
        return evaluateError(x.at<X1>(keys_[0]), x.at<X2>(keys_[1]), x.at<X3>(keys_[2]), x.at<X4>(keys_[3]), x.at<X5>(keys_[4]),
                             x.at<X6>(keys_[5]), x.at<X7>(keys_[6]), x.at<X8>(keys_[7]), x.at<X9>(keys_[8]), x.at<X10>(keys_[9]),
                             (*H)[0], (*H)[1], (*H)[2], (*H)[3], (*H)[4],
                             (*H)[5], (*H)[6], (*H)[7], (*H)[8], (*H)[9]);
      else
        return evaluateError(x.at<X1>(keys_[0]), x.at<X2>(keys_[1]), x.at<X3>(keys_[2]), x.at<X4>(keys_[3]), x.at<X5>(keys_[4]),
                             x.at<X6>(keys_[5]), x.at<X7>(keys_[6]), x.at<X8>(keys_[7]), x.at<X9>(keys_[8]), x.at<X10>(keys_[9]));
    } else {
      return gtsam::Vector::Zero(this->dim());
    }
  }

  /**
   *  Override this method to finish implementing a 10-way factor.
   *  If any of the optional Matrix reference arguments are specified, it should compute
   *  both the function evaluation and its derivative(s) in X1 (and/or X2, X3).
   */
  virtual gtsam::Vector
  evaluateError(const X1&, const X2&, const X3&, const X4&, const X5&,
                const X6&, const X7&, const X8&, const X9&, const X10&,
      boost::optional<gtsam::Matrix&> H1 = boost::none,
      boost::optional<gtsam::Matrix&> H2 = boost::none,
      boost::optional<gtsam::Matrix&> H3 = boost::none,
      boost::optional<gtsam::Matrix&> H4 = boost::none,
      boost::optional<gtsam::Matrix&> H5 = boost::none,
      boost::optional<gtsam::Matrix&> H6 = boost::none,
      boost::optional<gtsam::Matrix&> H7 = boost::none,
      boost::optional<gtsam::Matrix&> H8 = boost::none,
      boost::optional<gtsam::Matrix&> H9 = boost::none,
      boost::optional<gtsam::Matrix&> H10 = boost::none) const = 0;

private:

  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
    ar & boost::serialization::make_nvp("NoiseModelFactor",
        boost::serialization::base_object<Base>(*this));
  }
}; // \class NoiseModelFactor10


/* ************************************************************************* */
/** A convenient base class for creating your own NoiseModelFactor with 10
 * variables.  To derive from this class, implement evaluateError(). */
template<class VALUE1, class VALUE2, class VALUE3, class VALUE4, class VALUE5,
         class VALUE6, class VALUE7, class VALUE8, class VALUE9, class VALUE10,
         class VALUE11, class VALUE12>
class NoiseModelFactor12: public NoiseModelFactor {

public:

  // typedefs for value types pulled from keys
  using X1  = VALUE1;
  using X2  = VALUE2;
  using X3  = VALUE3;
  using X4  = VALUE4;
  using X5  = VALUE5;
  using X6  = VALUE6;
  using X7  = VALUE7;
  using X8  = VALUE8;
  using X9  = VALUE9;
  using X10 = VALUE10;
  using X11 = VALUE11;
  using X12 = VALUE12;

protected:
  using Base = NoiseModelFactor;
  using This = NoiseModelFactor12<VALUE1, VALUE2, VALUE3, VALUE4, VALUE5,
                                  VALUE6, VALUE7, VALUE8, VALUE9, VALUE10,
                                  VALUE11, VALUE12>;

public:
  NoiseModelFactor12() {}
  NoiseModelFactor12(const SharedNoiseModel& noiseModel, Key j1, Key j2, Key j3, Key j4, Key j5,
                     Key j6, Key j7, Key j8, Key j9, Key j10, Key j11, Key j12) :
    Base(noiseModel, cref_list_of<12>(j1)(j2)(j3)(j4)(j5)(j6)(j7)(j8)(j9)(j10)(j11)(j12)) {}

  ~NoiseModelFactor12() override {}

  /** methods to retrieve keys */
  inline Key key1() const { return keys_[0]; }
  inline Key key2() const { return keys_[1]; }
  inline Key key3() const { return keys_[2]; }
  inline Key key4() const { return keys_[3]; }
  inline Key key5() const { return keys_[4]; }
  inline Key key6() const { return keys_[5]; }
  inline Key key7() const { return keys_[6]; }
  inline Key key8() const { return keys_[7]; }
  inline Key key9() const { return keys_[8]; }
  inline Key key10() const { return keys_[9]; }
  inline Key key11() const { return keys_[10]; }
  inline Key key12() const { return keys_[11]; }

  /** Calls the 12-key specific version of evaluateError, which is pure virtual
   * so must be implemented in the derived class. */
  gtsam::Vector unwhitenedError(const Values& x, boost::optional<std::vector<gtsam::Matrix>&> H = boost::none) const override {
    if(this->active(x)) {
      if(H)
        return evaluateError(x.at<X1>(keys_[0]), x.at<X2>(keys_[1]), x.at<X3>(keys_[2]), x.at<X4>(keys_[3]), x.at<X5>(keys_[4]),
                             x.at<X6>(keys_[5]), x.at<X7>(keys_[6]), x.at<X8>(keys_[7]), x.at<X9>(keys_[8]), x.at<X10>(keys_[9]),
                             x.at<X11>(keys_[10]), x.at<X12>(keys_[11]),
                             (*H)[0], (*H)[1], (*H)[2], (*H)[3], (*H)[4],
                             (*H)[5], (*H)[6], (*H)[7], (*H)[8], (*H)[9],
                             (*H)[10], (*H)[11]);
      else
        return evaluateError(x.at<X1>(keys_[0]), x.at<X2>(keys_[1]), x.at<X3>(keys_[2]), x.at<X4>(keys_[3]), x.at<X5>(keys_[4]),
                             x.at<X6>(keys_[5]), x.at<X7>(keys_[6]), x.at<X8>(keys_[7]), x.at<X9>(keys_[8]), x.at<X10>(keys_[9]),
                             x.at<X11>(keys_[10]), x.at<X12>(keys_[11]));
    } else {
      return gtsam::Vector::Zero(this->dim());
    }
  }

  /**
   *  Override this method to finish implementing a 10-way factor.
   *  If any of the optional Matrix reference arguments are specified, it should compute
   *  both the function evaluation and its derivative(s) in X1 (and/or X2, X3).
   */
  virtual gtsam::Vector
  evaluateError(const X1&, const X2&, const X3&, const X4&, const X5&,
                const X6&, const X7&, const X8&, const X9&, const X10&,
                const X11&, const X12&,
      boost::optional<gtsam::Matrix&> H1 = boost::none,
      boost::optional<gtsam::Matrix&> H2 = boost::none,
      boost::optional<gtsam::Matrix&> H3 = boost::none,
      boost::optional<gtsam::Matrix&> H4 = boost::none,
      boost::optional<gtsam::Matrix&> H5 = boost::none,
      boost::optional<gtsam::Matrix&> H6 = boost::none,
      boost::optional<gtsam::Matrix&> H7 = boost::none,
      boost::optional<gtsam::Matrix&> H8 = boost::none,
      boost::optional<gtsam::Matrix&> H9 = boost::none,
      boost::optional<gtsam::Matrix&> H10 = boost::none,
      boost::optional<gtsam::Matrix&> H11 = boost::none,
      boost::optional<gtsam::Matrix&> H12 = boost::none) const = 0;

private:

  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
    ar & boost::serialization::make_nvp("NoiseModelFactor",
        boost::serialization::base_object<Base>(*this));
  }
}; // \class NoiseModelFactor12


} // namespace gtsam


#endif  // KINDYNVIO_FACTORS_GTSAM_NOISE_MODEL_FACTOR_CUSTOM_H


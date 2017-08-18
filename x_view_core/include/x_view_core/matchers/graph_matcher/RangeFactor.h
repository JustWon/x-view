/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  RangeFactor.h
 *  @author Frank Dellaert
 **/

#pragma once

#include <gtsam/geometry/Point3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/concepts.h>
#include <boost/lexical_cast.hpp>
#include <kindr/minimal/quat-transformation-gtsam.h>

#include <x_view_core/x_view_types.h>

namespace gtsam {

/**
 * Binary factor for a range measurement
 * @addtogroup SLAM
 */
template<class T1, class T2 = T1>
class RangeFactor: public NoiseModelFactor2<T1, T2> {
private:

  double measured_; /** measurement */

  typedef RangeFactor<T1, T2> This;
  typedef NoiseModelFactor2<T1, T2> Base;

  // Concept requirements for this factor
  GTSAM_CONCEPT_RANGE_MEASUREMENT_TYPE(T1, T2)

public:

  RangeFactor() {
  } /* Default constructor */

  RangeFactor(Key key1, Key key2, double measured,
      const SharedNoiseModel& model) :
      Base(model, key1, key2), measured_(measured) {
  }

  virtual ~RangeFactor() {
  }

  /// @return a deep copy of this factor
  virtual gtsam::NonlinearFactor::shared_ptr clone() const {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  gtsam::Point3 transform_to(const x_view::SE3& pose, const gtsam::Point3& p, gtsam::OptionalJacobian<3,6> Dpose = boost::none,
      gtsam::OptionalJacobian<3,3> Dpoint = boost::none) const {
    // Only get transpose once, to avoid multiple allocations,
    // as well as multiple conversions in the Quaternion case
    const Eigen::Matrix3d Rt = pose.getRotation().inverse().getRotationMatrix();
    const gtsam::Point3 q(Rt*(p.vector() - pose.getPosition()));
    if (Dpose) {
      const double wx = q.x(), wy = q.y(), wz = q.z();
      (*Dpose) <<
          0.0, -wz, +wy,-1.0, 0.0, 0.0,
          +wz, 0.0, -wx, 0.0,-1.0, 0.0,
          -wy, +wx, 0.0, 0.0, 0.0,-1.0;
    }
    if (Dpoint) {
      *Dpoint = Rt;
    }
    return q;
  }

  gtsam::Point3 transform_to(const gtsam::Point3& point_a, const gtsam::Point3& point_b, gtsam::OptionalJacobian<3,3> Dpoint_a = boost::none,
                             gtsam::OptionalJacobian<3,3> Dpoint_b = boost::none) const {
    // Only get transpose once, to avoid multiple allocations,
    // as well as multiple conversions in the Quaternion case
    const Eigen::Matrix3d Rt = Eigen::Matrix3d::Identity().inverse();
    const gtsam::Point3 q(Rt*(point_b - point_a).vector());
    if (Dpoint_a) {
      const double wx = q.x(), wy = q.y(), wz = q.z();
      (*Dpoint_a) <<
          0.0, -wz, +wy,
          +wz, 0.0, -wx,
          -wy, +wx, 0.0;
    }
    if (Dpoint_b) {
      *Dpoint_b = Rt;
    }
    return q;
  }

  /* ************************************************************************* */
  double range(const x_view::SE3& pose, const gtsam::Point3& point, gtsam::OptionalJacobian<1, 6> H1 = boost::none,
      gtsam::OptionalJacobian<1, 3> H2 = boost::none) const {
    if (!H1 && !H2) {
      return transform_to(pose, point).norm();
    } else {
      Eigen::Matrix<double, 3, 6> D1;
      Eigen::Matrix3d D2;
      gtsam::Point3 d = transform_to(pose, point, H1 ? &D1 : 0, H2 ? &D2 : 0);
      const double x = d.x(), y = d.y(), z = d.z(), d2 = x * x + y * y + z * z,
          n = sqrt(d2);
      Eigen::Matrix<double, 1, 3> D_result_d;
      D_result_d << x / n, y / n, z / n;
      if (H1) *H1 = D_result_d * D1;
      if (H2) *H2 = D_result_d * D2;
      return n;
    }
  }

  double range(const gtsam::Point3& point_a, const gtsam::Point3& point_b, gtsam::OptionalJacobian<1, 3> H1 = boost::none,
               gtsam::OptionalJacobian<1, 3> H2 = boost::none) const {
    if (!H1 && !H2) {
      return transform_to(point_a, point_a).norm();
    } else {
      Eigen::Matrix3d D1;
      Eigen::Matrix3d D2;
      gtsam::Point3 d = transform_to(point_a, point_b, H1 ? &D1 : 0, H2 ? &D2 : 0);
      const double x = d.x(), y = d.y(), z = d.z(), d2 = x * x + y * y + z * z,
          n = sqrt(d2);
      Eigen::Matrix<double, 1, 3> D_result_d;
      D_result_d << x / n, y / n, z / n;
      if (H1) *H1 = D_result_d * D1;
      if (H2) *H2 = D_result_d * D2;
      return n;
    }
  }

  /** h(x)-z */
  Vector evaluateError(const T1& t1, const T2& t2, boost::optional<Matrix&> H1 =
      boost::none, boost::optional<Matrix&> H2 = boost::none) const {
    double hx;
    hx = range(t1, t2, H1, H2);
    return (Vector(1) << hx - measured_).finished();
  }

  /** return the measured */
  double measured() const {
    return measured_;
  }

  /** equals specialized to this factor */
  virtual bool equals(const NonlinearFactor& expected,
      double tol = 1e-9) const {
    const This *e = dynamic_cast<const This*>(&expected);
    return e != NULL && Base::equals(*e, tol)
        && fabs(this->measured_ - e->measured_) < tol;
  }

  /** print contents */
  void print(const std::string& s = "", const KeyFormatter& keyFormatter =
      DefaultKeyFormatter) const {
    std::cout << s << "RangeFactor, range = " << measured_ << std::endl;
    Base::print("", keyFormatter);
  }

private:

  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
    ar
        & boost::serialization::make_nvp("NoiseModelFactor2",
            boost::serialization::base_object<Base>(*this));
    ar & BOOST_SERIALIZATION_NVP(measured_);
  }
}; // \ RangeFactor

/// traits
template<class T1, class T2>
struct traits<RangeFactor<T1,T2> > : public Testable<RangeFactor<T1,T2> > {};

/**
 * Binary factor for a range measurement, with a transform applied
 * @addtogroup SLAM
 */
template<class POSE, class T2 = POSE>
class RangeFactorWithTransform: public NoiseModelFactor2<POSE, T2> {
private:

  double measured_; /** measurement */
  POSE body_P_sensor_; ///< The pose of the sensor in the body frame

  typedef RangeFactorWithTransform<POSE, T2> This;
  typedef NoiseModelFactor2<POSE, T2> Base;

  // Concept requirements for this factor
  GTSAM_CONCEPT_RANGE_MEASUREMENT_TYPE(POSE, T2)

public:

  RangeFactorWithTransform() {
  } /* Default constructor */

  RangeFactorWithTransform(Key key1, Key key2, double measured,
      const SharedNoiseModel& model, const POSE& body_P_sensor) :
      Base(model, key1, key2), measured_(measured), body_P_sensor_(
          body_P_sensor) {
  }

  virtual ~RangeFactorWithTransform() {
  }

  /// @return a deep copy of this factor
  virtual gtsam::NonlinearFactor::shared_ptr clone() const {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /** h(x)-z */
  Vector evaluateError(const POSE& t1, const T2& t2,
      boost::optional<Matrix&> H1 = boost::none, boost::optional<Matrix&> H2 =
          boost::none) const {
    double hx;
    if (H1) {
      Matrix H0;
      hx = t1.compose(body_P_sensor_, H0).range(t2, H1, H2);
      *H1 = *H1 * H0;
    } else {
      hx = t1.compose(body_P_sensor_).range(t2, H1, H2);
    }
    return (Vector(1) << hx - measured_).finished();
  }

  /** return the measured */
  double measured() const {
    return measured_;
  }

  /** equals specialized to this factor */
  virtual bool equals(const NonlinearFactor& expected,
      double tol = 1e-9) const {
    const This *e = dynamic_cast<const This*>(&expected);
    return e != NULL && Base::equals(*e, tol)
        && fabs(this->measured_ - e->measured_) < tol
        && body_P_sensor_.equals(e->body_P_sensor_);
  }

  /** print contents */
  void print(const std::string& s = "", const KeyFormatter& keyFormatter =
      DefaultKeyFormatter) const {
    std::cout << s << "RangeFactor, range = " << measured_ << std::endl;
    this->body_P_sensor_.print("  sensor pose in body frame: ");
    Base::print("", keyFormatter);
  }

private:

  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
    ar
        & boost::serialization::make_nvp("NoiseModelFactor2",
            boost::serialization::base_object<Base>(*this));
    ar & BOOST_SERIALIZATION_NVP(measured_);
    ar & BOOST_SERIALIZATION_NVP(body_P_sensor_);
  }
}; // \ RangeFactorWithTransform

/// traits
template<class T1, class T2>
struct traits<RangeFactorWithTransform<T1, T2> > : public Testable<RangeFactorWithTransform<T1, T2> > {};

} // \ namespace gtsam

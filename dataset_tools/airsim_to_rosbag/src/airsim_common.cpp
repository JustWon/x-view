#include "airsim_to_rosbag/airsim_common.h"

namespace airsim {

Transformation interpolateTransformations(const Transformation& left,
                                          const Transformation& right,
                                          double t) {
  Transformation output;
  // Linearly interpolate the position between the two.
  output.getPosition() = left.getPosition() * (1 - t) + right.getPosition() * t;

  // slerp the rotation between the two.
  output.getRotation().toImplementation() =
      left.getRotation().toImplementation().slerp(
          t, right.getRotation().toImplementation());

  return output;
}

}  // namespace airsim

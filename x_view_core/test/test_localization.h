#ifndef X_VIEW_TEST_LOCALIZATION_H
#define X_VIEW_TEST_LOCALIZATION_H

namespace x_view_test {

/**
 * \brief This function tests the localization accuracy of the
 * x_view::GraphLocalizer class for localizing a robot in 3D space given a
 * list of observations (coordiantes in camera frame) with associated 3D
 * position.
 * \param num_tests Number of tests to run.
 * \param num_observations Number of observations used in each test.
 * \param seed Seed used for random number generation.
 */
void testLocalization(const int num_tests, const int num_observations,
                      const int seed);

void testLocalizationUnderRotation();

}

#endif //X_VIEW_TEST_LOCALIZATION_H

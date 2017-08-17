#ifndef X_VIEW_EVALUATION_H
#define X_VIEW_EVALUATION_H

#include <x_view_core/timer/null_timer.h>
#include <x_view_core/timer/timer.h>

namespace x_view_evaluation {

/**
 * \brief Parameters to be used when evaluating X-View.
 */
struct EvaluationParameters {

  /// \brief Enum defining which type of timer should be used when running
  /// X-View.
  enum TIMER_TYPE {
    /// \brief A timer that does nothing.
    NULL_TIMER = 0,
    /// \brief A real timer that measures execution time.
    TIMER
  };

  /// \brief Default constructor which initializes the variables on their
  /// default value.
  EvaluationParameters();


  /// \brief Type of timer to be used when running X-View.
  TIMER_TYPE timer_type;

};

class Evaluation {
 public:

  typedef EvaluationParameters::TIMER_TYPE TIMER_TYPE;

  Evaluation(const EvaluationParameters& params);

  // ============================ Timer ============================ //

  /**
   * \brief Generates a string containing the measurements performed by the
   * current active timer (i.e. timer located at x_view::Locator::timer_).
   * \return The table containing timing measurements under the form of a
   * human readable string.
   */
  const std::string getTimingsTable() const;

  const std::string getTimingsTree() const;

  /**
   * \brief Gets the timings performed by the current active timer (i.e.
   * timer located at x_view::Locator::timer_).
   * \return An unordered map keyed by the timer names corresponding to a
   * vector of measurements expressed in seconds.
   */
  const std::unordered_map<std::string, std::vector<x_view::real_t>>
      getAllTimings() const;

  /**
   * \brief This function allows to retrieve and store locally the current
   * active timer. A new empty timer of type params_.timer_type is registered
   * at his place under x_view::Locator::timer_.
   * \param timer A pointer to a pointer to an x_view::AbstractTimer object.
   * \code{cpp}
   * Evaluation evaluation(...);
   * { // Perform some measurements }
   * x_view::AbstractTimer* first_timer;
   * evaluation.storeTimer(&first_timer);
   * { // Perform new measurements }
   * x_view::AbstractTimer* second_timer;
   * evaluation.storeTimer(&second_timer);
   * \endcode
   */
  void storeTimer(x_view::AbstractTimer** timer) const;

  // ========================== Estimation ======================== //
  // TODO: create functions to query estimation properties.

 private:
  /// \brief Function called by the Evaluation constructor which makes sure
  /// x_view::Locator has a valid timer registered.
  static void initializeTimer(const TIMER_TYPE timer_type);

  /// \brief Parameters used for this evaluation.
  const EvaluationParameters params_;
};

}

#endif //X_VIEW_EVALUATION_H

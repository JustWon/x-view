#ifndef X_VIEW_EVALUATION_H
#define X_VIEW_EVALUATION_H

#include <x_view_evaluation/statistics.h>
#include <x_view_core/timer/null_timer.h>
#include <x_view_core/timer/timer.h>

#include <fstream>

namespace x_view_evaluation {

/**
 * \brief Parameters to be used when evaluating X-View.
 */
struct EvaluationParameters {

  /// \brief Default constructor which initializes the variables on their
  /// default value.
  EvaluationParameters();

  /// \brief Enum defining which type of timer should be used when running
  /// X-View.
  enum TIMER_TYPE {
    /// \brief A timer that does nothing.
    NULL_TIMER = 0,
    /// \brief A real timer that measures execution time.
    TIMER
  };

  /// \brief Type of timer to be used when running X-View.
  TIMER_TYPE timer_type;

};

class Evaluation {
 public:

  typedef EvaluationParameters::TIMER_TYPE TIMER_TYPE;

  Evaluation(const EvaluationParameters& params);
  
  /**
   * \brief Writes all data collected by the evaluation into the directory
   * specified by the passed argument.
   * \param folder_name Path to the folder where to write all results
   * contained in this Evaluation instance.
   * \return Boolen flag indicating writing success.
   */
  bool writeToFolder(const std::string& folder_name) const;

  /**
   * \brief Evaluation of timings.
   */
  class TimerEvaluation {

    friend class Evaluation;

   public:

    bool writeToFile(const std::string& filename) const;

    /**
     * \brief Generates a string containing the measurements performed by the
     * current active timer (i.e. timer located at x_view::Locator::timer_).
     * \return The table containing timing measurements under the form of a
     * human readable string.
     */
    const std::string getTimingsTable() const;

    /**
     * \brief Generates a string containing the measurements performed by the
     * current active timer (i.e. timer located at x_view::Locator::timer_).
     * \return The tree structure containing timing measurements under the form
     * of a human readable string.
     */
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
     * evaluation.time.storeTimer(&first_timer);
     * { // Perform new measurements }
     * x_view::AbstractTimer* second_timer;
     * evaluation.time.storeTimer(&second_timer);
     * \endcode
     */
    void storeTimer(x_view::AbstractTimer** timer) const;

   private:
    TimerEvaluation(const EvaluationParameters* params)
        : params_(params) {}

    /// \brief Function called by the Evaluation constructor which makes sure
    /// x_view::Locator has a valid timer registered.
    static void initializeTimer(const TIMER_TYPE timer_type);

    const EvaluationParameters* params_;
  };

  /// \brief Instance of TimerEvaluation accessible publicly.
  TimerEvaluation time;

  class LocalizationEvaluation {

    friend class Evaluation;

   public:

    bool writeToFile(const std::string& filename) const;

    /**
     * \brief Adds the pair of positions to the statistic object referred to
     * by the string passed as argument. This allows to compute different
     * statistics simultaneously without mixing the samples.
     * \param statistics_name Name used as key to refer to the statistics
     * where to store the samples passed as argument.
     * \param true_position Ground true position of the robot.
     * \param estimated_position Estimated position of the robot.
     */
    void addLocalization(const std::string& statistics_name,
                         const x_view::Vector3r& true_position,
                         const x_view::Vector3r& estimated_position);


    /**
     * \brief Computes the mean squared distance of the samples added to the
     * statistics associated with the statistics name passed as argument.
     * \param statistics_name Name used as key to refer to the statistics
     * for which the mean squared distance to the ground truths is to be
     * computed.
     * \return The mean distance squared of the measurements to the
     * corresponding ground truths.
     */
    const x_view::real_t MSD(const std::string& statistics_name) const;

    /**
     * \brief Generates a table of statistics containing information
     * associated with the computed statistics.
     * \return A string representing a table of statistic and associated
     * information in a human readable way.
     */
    const std::string getStatisticsTable() const;


   private:
    LocalizationEvaluation(const EvaluationParameters* params)
        : params_(params) {}

    std::unordered_map<std::string, Statistics> statistics_map_;

    const EvaluationParameters* params_;
  };

  /// \brief Instance of LocalizationEvaluation accessible publicly.
  LocalizationEvaluation localization;

 private:

  /// \brief Parameters used for this evaluation.
  const EvaluationParameters params_;
};

}

#endif //X_VIEW_EVALUATION_H

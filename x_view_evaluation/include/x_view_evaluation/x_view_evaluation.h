#ifndef X_VIEW_EVALUATION_H
#define X_VIEW_EVALUATION_H

#include <x_view_core/features/graph.h>
#include <x_view_core/matchers/graph_matcher.h>
#include <x_view_core/timer/null_timer.h>
#include <x_view_core/timer/timer.h>

#include <fstream>
#include <unordered_map>
#include <vector>

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
   * \return Boolean flag indicating writing success.
   */
  bool writeToFolder(const std::string& folder_name) const;

  /**
   * \brief Evaluation of timings.
   */
  class TimerEvaluation {

    friend class Evaluation;

   public:

    /**
     * \brief Writes the data collected by the TimerEvaluation into the
     * folder specified by the path passed as argument.
     * \param folder_name Absolute path of the folder where to write the time
     * measurements.
     * \param suffix Suffix string to add to all generated files.
     * \return Success flag.
     */
    bool writeToFolder(const std::string& folder_name,
                       const std::string& suffix = "") const;

    /**
     * \brief Generates a string containing the measurements performed by the
     * current active timer (i.e. timer located at x_view::Locator::timer_).
     * \param use_colors Boolean flag to indicate if the generated string
     * should be color-coded (nicer for terminal output).
     * \return The table containing timing measurements under the form of a
     * human readable string.
     */
    const std::string getTimingsTable(const bool use_colors = false) const;

    /**
     * \brief Generates a string containing the measurements performed by the
     * current active timer (i.e. timer located at x_view::Locator::timer_).
     * \param use_colors Boolean flag to indicate if the generated string
     * should be color-coded (nicer for terminal output).
     * \return The tree structure containing timing measurements under the form
     * of a human readable string.
     */
    const std::string getTimingsTree(const bool use_colors = false) const;

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

    /**
     * \brief Writes the data collected by the LocalizationEvaluation into the
     * folder specified by the path passed as argument.
     * \param folder_name Absolute path of the folder where to write the time
     * measurements.
     * \param suffix Suffix string to add to all generated files.
     * \return Success flag.
     * \note All localizations are written to a file in the following format:
     * "
     * x_gt y_gt z_gt r11_gt r12_gt r13_gt ... r33_gt
     * x_es y_es z_es r11_es r12_es r13_es ... r33_es
     * localization_error
     * ...
     * x_gt y_gt z_gt r11_gt r12_gt r13_gt ... r33_gt
     * x_es y_es z_es r11_es r12_es r13_es ... r33_es
     * localization_error
     * "
     * where each triplet of consecutive lines corresponds to the ground truth
     * pose followed by the estimated pose, followed by the localization error.
     * pose followed by the estimated pose, followed by the localization error.
     */
    bool writeToFolder(const std::string& folder_name,
                       const std::string& suffix = "") const;

    /**
     * \brief Adds the pair of poses to the statistic object referred to
     * by the string passed as argument. This allows to compute different
     * statistics simultaneously without mixing the samples.
     * \param localization_pair Localization pair resulting from a
     * localization query to X-View.
     * \param error Error associated to the localization estimate.
     */
    void addLocalization(const x_view::LocalizationPair& localization_pair,
                         const x_view::real_t error);

    /**
     * \brief Computes the mean squared distance of the samples added to the
     * statistics associated with the statistics name passed as argument.
     * \return The mean distance squared of the measurements to the
     * corresponding ground truths.
     */
    const x_view::real_t MSD() const;

    /**
     * \brief Computes the mean distance of the samples added to the
     * statistics associated with the statistics name passed as argument.
     * \return The mean distance of the measurements to the corresponding
     * ground truths.
     */
    const x_view::real_t MD() const;

    /**
     * \brief Computes the mean squared angle of the samples added to the
     * statistics associated with the statistics name passed as argument.
     * \return The mean angle of the measurements to the corresponding
     * ground truths.
     */
    const x_view::real_t MSA() const;

    /**
     * \brief Computes the empirical standard deviation on the squared
     * distances.
     * \return The empirical standard deviation of the squared distances.
     */
    const x_view::real_t standardDeviationMeanSquaredDistance() const;

    /**
     * \brief Computes the empirical standard deviation on the distances.
     * \return The empirical standard deviation of the distances.
     */
    const x_view::real_t standardDeviationMeanDistance() const;

    /**
     * \brief Computes the empirical standard deviation on the squared angle
     * deviations.
     * \return The empirical standard deviation of the squared angles.
     */
    const x_view::real_t standardDeviationMeanSquaredAngles() const;

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

    struct LocalizationSample {
      LocalizationSample() {}
      LocalizationSample(
          const x_view::LocalizationPair& localization_pair,
          const x_view::real_t error)
          : localization_pair(localization_pair),
            error(error) {
      }
      x_view::LocalizationPair localization_pair;
      x_view::real_t error;
    };
    std::vector<LocalizationSample> localizations_vector_;

    const EvaluationParameters* params_;
  };

  /// \brief Instance of LocalizationEvaluation accessible publicly.
  LocalizationEvaluation localization;


  class SimilarityEvaluation {

    friend class Evaluation;

   public:

    bool writeToFolder(const std::string& folder_name,
                       const std::string& suffix = "") const;

    void addSimilarities(
        const x_view::Graph& database_graph, const x_view::Graph& query_graph,
        const x_view::GraphMatcher::IndexMatrixType& candidate_matches);

    void addSimilarities(
        const x_view::Graph& database_graph, const x_view::Graph& query_graph,
        const x_view::GraphMatcher::SimilarityMatrixType& similarity_matrix,
        const x_view::GraphMatcher::IndexMatrixType& candidate_matches);

   private:
    SimilarityEvaluation(const EvaluationParameters* params)
        : params_(params) {}

    struct SimilaritySample {
      SimilaritySample() {}
      SimilaritySample(const x_view::Vector3r& db_position,
                       const x_view::Vector3r& query_position,
                       const x_view::real_t similarity)
          : db_position(db_position),
            query_position(query_position),
            similarity(similarity) {
      }

      x_view::Vector3r db_position;
      x_view::Vector3r query_position;
      x_view::real_t similarity;
    };

    std::vector<SimilaritySample> similarities_vector_;

    const EvaluationParameters* params_;
  };

  /// \brief Instance of SimilarityEvaluation accessible publicly.
  SimilarityEvaluation similarity;

 private:

  /// \brief Parameters used for this evaluation.
  const EvaluationParameters params_;
};

}

#endif //X_VIEW_EVALUATION_H

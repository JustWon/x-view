#include <x_view_evaluation/x_view_evaluation.h>

#include <x_view_core/timer/timer_printer.h>
#include <x_view_core/x_view_locator.h>
#include <x_view_core/x_view_tools.h>

namespace x_view_evaluation {

EvaluationParameters::EvaluationParameters()
    : timer_type(TIMER_TYPE::NULL_TIMER) {
}

Evaluation::Evaluation(const EvaluationParameters& params)
: params_(params),
  time(&params_),
  localization(&params_),
  similarity(&params_) {

  Evaluation::TimerEvaluation::initializeTimer(params_.timer_type);
}

bool Evaluation::writeToFolder(const std::string& folder_name) const {

  CHECK(folder_name.back() == '/')
        << "Folder name <" << folder_name << "> passed to "
        << __FUNCTION__ << " does not end with '/'.";

  LOG(INFO) << "Writing evaluation results to folder <" << folder_name << ">.";

  bool success = true;
  success &= time.writeToFolder(folder_name);
  success &= localization.writeToFolder(folder_name);
  success &= similarity.writeToFolder(folder_name);

  return success;
}

bool Evaluation::TimerEvaluation::writeToFolder(
    const std::string& folder_name, const std::string& suffix) const {

  // Create the new folder.
  system(("mkdir -p " + folder_name).c_str());
  LOG(INFO) << "Creating directory <" << folder_name << ">.";

  // Disable color coding for plain output text.
  const bool use_colors = false;

  // Write the time table to file.
  const std::string table_file_name =
      folder_name + "time_table" +
          (suffix == "" ? "" : "_" + suffix + "_") + ".dat";
  std::ofstream out_table(table_file_name.c_str());
  if(!out_table.is_open()) {
    LOG(ERROR) << "Could not open file <" << folder_name << "time_table.dat>.";
    return false;
  }
  out_table << getTimingsTable(use_colors);

  // Write the time tree to file.
  const std::string tree_file_name =
      folder_name + "time_tree" +
          (suffix == "" ? "" : "_" + suffix + "_") + ".dat";
  std::ofstream out_tree(tree_file_name.c_str());
  if(!out_table.is_open()) {
    LOG(ERROR) << "Could not open file <" << folder_name << "time_tree.dat>.";
    return false;
  }
  out_tree << getTimingsTree(use_colors);

  // Write all measurements to file.
  const std::string all_timings_file_name =
      folder_name + "all_timings" +
          (suffix == "" ? "" : "_" + suffix + "_") + ".dat";
  std::ofstream out_all_times(all_timings_file_name.c_str());
  if(!out_table.is_open()) {
    LOG(ERROR) << "Could not open file <" << folder_name << "time_tree.dat>.";
    return false;
  }

  const auto all_timings = getAllTimings();
  for(const auto& p : all_timings) {
    out_all_times <<  p.first << " \t ";
    for(const x_view::real_t t : p.second) {
      out_all_times << t << " \t ";
    }
    out_all_times << std::endl;
  }

  return true;
}

const std::string Evaluation::TimerEvaluation::getTimingsTable(
    const bool use_colors) const {
  const auto& timer = x_view::Locator::getTimer();
  if(params_->timer_type == EvaluationParameters::TIMER_TYPE::NULL_TIMER) {
    return "Registered timer is of <nonMeasuring> type, this means that the "
        "registered instance does not measure any time.";
  }

  x_view::Timer* real_timer = dynamic_cast<x_view::Timer*>(timer.get());
  CHECK(real_timer != nullptr)
        << "Function " << __FUNCTION__ << " is only supported for timers of "
            "type <x_view::Timer>";

  x_view::TimerPrinter::USE_COLORS = use_colors;
  return x_view::TimerPrinter::getTimingsTable(*real_timer);
}

const std::string Evaluation::TimerEvaluation::getTimingsTree(
    const bool use_colors) const {
  const auto& timer = x_view::Locator::getTimer();
  if(params_->timer_type == EvaluationParameters::TIMER_TYPE::NULL_TIMER) {
    return "Registered timer is of <nonMeasuring> type, this means that the "
        "registered instance does not measure any time.";
  }

  x_view::Timer* real_timer = dynamic_cast<x_view::Timer*>(timer.get());
  CHECK(real_timer != nullptr)
  << "Function " << __FUNCTION__ << " is only supported for timers of "
      "type <x_view::Timer>";

  x_view::TimerPrinter::USE_COLORS = use_colors;
  return x_view::TimerPrinter::getTimingsTree(*real_timer);
}

const std::unordered_map<std::string, std::vector<x_view::real_t>>
Evaluation::TimerEvaluation::getAllTimings() const {
  const auto& timer = x_view::Locator::getTimer();
  if(params_->timer_type == EvaluationParameters::TIMER_TYPE::NULL_TIMER) {
    return {{"No measurements where made", {}}};
  }

  x_view::Timer* real_timer = dynamic_cast<x_view::Timer*>(timer.get());
  CHECK(real_timer != nullptr)
  << "Function " << __FUNCTION__ << " is only supported for timers of "
      "type <x_view::Timer>";

  return real_timer->getAllTimings();
};

void Evaluation::TimerEvaluation::storeTimer(x_view::AbstractTimer** timer) const {
  *timer = x_view::Locator::removeTimer();

  initializeTimer(params_->timer_type);
}


void Evaluation::TimerEvaluation::initializeTimer(const TIMER_TYPE timer_type) {

  switch(timer_type) {
    case TIMER_TYPE::NULL_TIMER: {
      std::unique_ptr<x_view::AbstractTimer> timer(new x_view::NullTimer());
      x_view::Locator::registerTimer(std::move(timer));
      return;
    }
    case TIMER_TYPE::TIMER: {
      std::unique_ptr<x_view::AbstractTimer> timer(new x_view::Timer());
      x_view::Locator::registerTimer(std::move(timer));
      return;
    }
    default:
      LOG(ERROR) << "Undefined timer type passed to " << __FUNCTION__ << ".";
  }
}

bool Evaluation::LocalizationEvaluation::writeToFolder(
    const std::string& folder_name,  const std::string& suffix) const {

  // Create the new folder.
  system(("mkdir -p " + folder_name).c_str());

  const std::string localization_table_file_name =
      folder_name + "localization_table" +
          (suffix == "" ? "" : "_" + suffix + "_") + ".dat";
  std::ofstream out(localization_table_file_name.c_str());
  if(!out.is_open()) {
    LOG(ERROR) << "Could not open file <"
               << localization_table_file_name << ">.";
    return false;
  }

  // If there are no localization info, then write an empty file.
  if(localizations_vector_.size() == 0)
    out << "No statistics on localization, as no localization data was "
        "registered.";
  else
    out << getStatisticsTable();


  // Write all measurements to file.
  const std::string all_localization_file_name =
      folder_name + "all_localizations" +
          (suffix == "" ? "" : "_" + suffix + "_") + ".dat";
  std::ofstream out_all_localizations(all_localization_file_name.c_str());
  if(!out_all_localizations.is_open()) {
    LOG(ERROR) << "Could not open file <" << all_localization_file_name << ">";
    return false;
  }

  // Converts a pose to a string of the form  "x y z r11 r12 r13 r21 .... r33"
  auto poseToString = [](const x_view::SE3& pose) -> std::string {
    const auto position = pose.getPosition();
    const auto rotation = pose.getRotationMatrix();

    std::stringstream ss;
    Eigen::IOFormat plain_format(Eigen::StreamPrecision, Eigen::DontAlignCols,
                                 " ", " ", "", "", "", "");
    ss << position.format(plain_format) << " " << rotation.format(plain_format);
    return ss.str();
  };

  for(const LocalizationSample& p : localizations_vector_) {
    const x_view::LocalizationPair& l = p.localization_pair;
    const x_view::SE3& ground_truth = l.true_pose;
    const x_view::SE3& estimation = l.estimated_pose;
    const x_view::real_t error = p.error;

    out_all_localizations << poseToString(ground_truth) << "\n"
                          << poseToString(estimation) << "\n"
                          << error << "\n";
  }

  return true;
}

void Evaluation::LocalizationEvaluation::addLocalization(
    const x_view::LocalizationPair& localization_pair,
    const x_view::real_t error) {
  localizations_vector_.push_back(LocalizationSample(localization_pair, error));
}

const x_view::real_t Evaluation::LocalizationEvaluation::MSD() const {

  x_view::real_t mean_squared_distance = static_cast<x_view::real_t>(0.0);
  for(const LocalizationSample& p : localizations_vector_) {
    const x_view::LocalizationPair& l = p.localization_pair;
    mean_squared_distance += x_view::distSquared(l.estimated_pose.getPosition(),
                                                 l.true_pose.getPosition());
  }

  const uint64_t num_obs = localizations_vector_.size();
  return mean_squared_distance / num_obs;
}

const x_view::real_t Evaluation::LocalizationEvaluation::MD() const {

  x_view::real_t mean_distance = static_cast<x_view::real_t>(0.0);
  for(const LocalizationSample& p : localizations_vector_) {
    const x_view::LocalizationPair& l = p.localization_pair;
    mean_distance += x_view::dist(l.estimated_pose.getPosition(),
                                  l.true_pose.getPosition());
  }

  const uint64_t num_obs = localizations_vector_.size();
  return mean_distance / num_obs;
}

const x_view::real_t Evaluation::LocalizationEvaluation::MSA() const {

  x_view::real_t mean_squared_angle = static_cast<x_view::real_t>(0.0);
  for(const LocalizationSample& p : localizations_vector_) {
    const x_view::LocalizationPair& l = p.localization_pair;
    const x_view::real_t angle = x_view::angle(l.estimated_pose, l.true_pose);
    mean_squared_angle += angle * angle;
  }

  const uint64_t num_obs = localizations_vector_.size();
  return mean_squared_angle / num_obs;
}


const x_view::real_t Evaluation::LocalizationEvaluation::
standardDeviationMeanSquaredDistance() const {
  const x_view::real_t msd = MSD();
  x_view::real_t s = static_cast<x_view::real_t>(0.0);
  for(const LocalizationSample& p : localizations_vector_) {
    const x_view::LocalizationPair& l = p.localization_pair;
    const x_view::real_t squared_distance =
        x_view::distSquared(l.estimated_pose.getPosition(),
                            l.true_pose.getPosition());
    s += (squared_distance - msd) * (squared_distance - msd);
  }
  const uint64_t num_obs = localizations_vector_.size();
  return static_cast<x_view::real_t>(1.0 / (num_obs - 1) * std::sqrt(s));
}

const x_view::real_t Evaluation::LocalizationEvaluation::
standardDeviationMeanDistance() const {
  const x_view::real_t md = MD();
  x_view::real_t s = static_cast<x_view::real_t>(0.0);
  for(const LocalizationSample& p : localizations_vector_) {
    const x_view::LocalizationPair& l = p.localization_pair;
    const x_view::real_t distance = x_view::dist(l.estimated_pose.getPosition(),
                                                 l.true_pose.getPosition());
    s += (distance - md) * (distance - md);
  }
  const uint64_t num_obs = localizations_vector_.size();
  return static_cast<x_view::real_t>(1.0 / (num_obs - 1) * std::sqrt(s));
}

const x_view::real_t Evaluation::LocalizationEvaluation::
standardDeviationMeanSquaredAngles() const {
  const x_view::real_t msa = MSA();
  x_view::real_t s = static_cast<x_view::real_t>(0.0);
  for(const LocalizationSample& p : localizations_vector_) {
    const x_view::LocalizationPair& l = p.localization_pair;
    const x_view::real_t angle = x_view::angle(l.estimated_pose, l.true_pose);
    const x_view::real_t squared_angle = angle * angle;
    s += (squared_angle - msa) * (squared_angle - msa);
  }
  const uint64_t num_obs = localizations_vector_.size();
  return static_cast<x_view::real_t>(1.0 / (num_obs - 1) * std::sqrt(s));
}

const std::string Evaluation::LocalizationEvaluation::getStatisticsTable()
const {
  const uint64_t statistics_name_length = 15;
  const uint64_t col_width = 7;
  const std::string col_sep = " | ";
  std::stringstream ss;

  auto getRightString = [&](const std::string& s) -> std::string {
    const uint64_t string_length = s.length();
    if (string_length > statistics_name_length)
      return s.substr(0, statistics_name_length - 1) + ".";

    const uint64_t remaining_space = statistics_name_length - string_length;
    std::string center_string(remaining_space, ' ');
    return center_string + s;
  };

  auto getLeftString = [&](const std::string& s) -> std::string {
    const uint64_t string_length = s.length();
    if (string_length > col_width)
      return s.substr(0, col_width - 1) + ".";

    const uint64_t remaining_space = col_width - string_length;
    std::string center_string(remaining_space, ' ');
    return s + center_string;
  };

  ss << getRightString("Statistics") << col_sep;
  ss << getLeftString("MSD") << col_sep;
  ss << getLeftString("std") << col_sep;
  ss << getLeftString("MD") << col_sep;
  ss << getLeftString("std") << col_sep;
  ss << getLeftString("MSA") << col_sep;
  ss << getLeftString("std") << col_sep;
  ss << getLeftString("num");
  ss << "\n";

  const uint64_t line_width = ss.str().length();
  ss << std::setfill('=') << std::setw(line_width);
  ss << "\n";


  ss << getRightString(" ") << col_sep;
  ss << getLeftString(std::to_string(MSD())) << col_sep;
  ss << getLeftString(std::to_string(standardDeviationMeanSquaredDistance())) << col_sep;
  ss << getLeftString(std::to_string(MD())) << col_sep;
  ss << getLeftString(std::to_string(standardDeviationMeanDistance())) << col_sep;
  ss << getLeftString(std::to_string(MSA())) << col_sep;
  ss << getLeftString(std::to_string(standardDeviationMeanSquaredAngles())) << col_sep;
  ss << getLeftString(std::to_string(localizations_vector_.size()));
  ss << "\n";

  std::string s = "======== STATISTICS TABLE ========";
  s += "\n\n";
  return s + ss.str();

}

bool Evaluation::SimilarityEvaluation::writeToFolder(
    const std::string& folder_name,  const std::string& suffix) const {

  // Create the new folder.
  system(("mkdir -p " + folder_name).c_str());

  const std::string similarities_file_name =
      folder_name + "similarities" +
          (suffix == "" ? "" : "_" + suffix + "_") + ".dat";
  std::ofstream out(similarities_file_name.c_str());
  if(!out.is_open()) {
    LOG(ERROR) << "Could not open file <"
               << similarities_file_name << ">.";
    return false;
  }

  // If there are no similarity info, then write an empty file.
  if(similarities_vector_.size() == 0)
    out << "No similarities available.";
  else {

    // Converts a SimilaritySample to a string of the form
    // "x_db y_db z_db x_q y_q z_q s"
    auto similarityToString = [](const SimilaritySample& sample) ->
        std::string {

      std::stringstream ss;
      Eigen::IOFormat plain_format(Eigen::StreamPrecision, Eigen::DontAlignCols,
                                   " ", " ", "", "", "", "");
      ss << sample.db_position.format(plain_format) << " "
         << sample.query_position.format(plain_format) << " "
         << sample.similarity;
      return ss.str();
    };

    for (const SimilaritySample& s : similarities_vector_) {
      out << similarityToString(s) << "\n";
    }
  }

  return true;
}

void Evaluation::SimilarityEvaluation::addSimilarities(
    const x_view::Graph& database_graph, const x_view::Graph& query_graph,
    const x_view::GraphMatcher::SimilarityMatrixType& similarity_matrix,
    const x_view::GraphMatcher::IndexMatrixType& candidate_matches) {

  const int INVALID_MATCH_INDEX = x_view::GraphMatcher::INVALID_MATCH_INDEX;
  for(uint64_t j = 0; j < candidate_matches.cols(); ++j) {
    for(uint64_t i = 0; i < candidate_matches.rows(); ++i) {

      const int db_index = candidate_matches(i, j);
      if(db_index != INVALID_MATCH_INDEX) {
        const x_view::Vector3r& query_position = query_graph[j].location_3d;
        const x_view::Vector3r& db_position =
            database_graph[db_index].location_3d;
        const x_view::real_t similarity = similarity_matrix(db_index, j);

        similarities_vector_.push_back(SimilaritySample(db_position,
                                                        query_position,
                                                        similarity));

      }
    }
  }
}

}


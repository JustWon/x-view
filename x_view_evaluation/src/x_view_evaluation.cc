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
  localization(&params_) {

  Evaluation::TimerEvaluation::initializeTimer(params_.timer_type);
}

bool Evaluation::writeToFolder(const std::string& folder_name) const {

  CHECK(folder_name.back() == '/')
        << "Folder name <" << folder_name << "> passed to "
        << __FUNCTION__ << " does not end with '/'.";

  // Create and delete all dat file contained the new folder.
  system(("mkdir -p " + folder_name).c_str());
  system(("rm -rf " + folder_name + "*.dat").c_str());
  
  LOG(INFO) << "Writing evaluation results to folder <" << folder_name << ">.";

  bool success = true;
  success &= time.writeToFile(folder_name);
  success &= localization.writeToFile(folder_name);

  return success;
}

bool Evaluation::TimerEvaluation::writeToFile(const std::string& folder_name) const {

  // Disable color coding for plain output text.
  const bool use_colors = false;

  // Write the time table to file.
  std::ofstream out_table((folder_name + "time_table.dat").c_str());
  if(!out_table.is_open()) {
    LOG(ERROR) << "Could not open file <" << folder_name << "time_table.dat>.";
    return false;
  }
  out_table << getTimingsTable(use_colors);

  // Write the time tree to file.
  std::ofstream out_tree((folder_name + "time_tree.dat").c_str());
  if(!out_table.is_open()) {
    LOG(ERROR) << "Could not open file <" << folder_name << "time_tree.dat>.";
    return false;
  }
  out_tree << getTimingsTree(use_colors);

  // Write all measurements to file.
  std::ofstream out_all_times((folder_name + "all_timings.dat").c_str());
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

bool Evaluation::LocalizationEvaluation::writeToFile(
    const std::string& folder_name) const {

  std::ofstream out((folder_name + "localization_table.dat").c_str());
  if(!out.is_open()) {
    LOG(ERROR) << "Could not open file <" << folder_name << "localization_table.dat>.";
    return false;
  }


  // If there are no localization info, then write an empty file.
  if(statistics_map_.size() == 0)
    out << "No statistics on localization, as no localization data was "
        "registered.";
  else
    out << getStatisticsTable();

  return true;
}

void Evaluation::LocalizationEvaluation::addLocalization(
    const std::string& statistics_name,
    const x_view::LocalizationPair& localization_pair) {
  statistics_map_[statistics_name].push_back(localization_pair);
}

const x_view::real_t Evaluation::LocalizationEvaluation::MSD(
    const std::string& statistics_name) const {
  CHECK(statistics_map_.count(statistics_name) > 0)
        << "Requested MSD for statistics <" << statistics_name << "> but no "
            "statistic with that key has been registered.";

  x_view::real_t mean_squared_distance = static_cast<x_view::real_t>(0.0);
  for(const x_view::LocalizationPair& p : statistics_map_.at(statistics_name)) {
    mean_squared_distance += x_view::distSquared(p.estimated_pose.getPosition(),
                                                 p.true_pose.getPosition());
  }

  const uint64_t num_obs = statistics_map_.at(statistics_name).size();
  return mean_squared_distance / num_obs;
}

const x_view::real_t Evaluation::LocalizationEvaluation::MD(
    const std::string& statistics_name) const {
  CHECK(statistics_map_.count(statistics_name) > 0)
  << "Requested MD for statistics <" << statistics_name << "> but no "
      "statistic with that key has been registered.";

  x_view::real_t mean_distance = static_cast<x_view::real_t>(0.0);
  for(const x_view::LocalizationPair& p : statistics_map_.at(statistics_name)) {
    mean_distance += x_view::dist(p.estimated_pose.getPosition(),
                                  p.true_pose.getPosition());
  }

  const uint64_t num_obs = statistics_map_.at(statistics_name).size();
  return mean_distance / num_obs;
}

const x_view::real_t Evaluation::LocalizationEvaluation::MSA(
    const std::string& statistics_name) const {
  CHECK(statistics_map_.count(statistics_name) > 0)
  << "Requested MSA for statistics <" << statistics_name << "> but no "
      "statistic with that key has been registered.";

  x_view::real_t mean_squared_angle = static_cast<x_view::real_t>(0.0);
  for(const x_view::LocalizationPair& p : statistics_map_.at(statistics_name)) {
    const x_view::real_t angle = x_view::angle(p.estimated_pose, p.true_pose);
    mean_squared_angle += angle * angle;
  }

  const uint64_t num_obs = statistics_map_.at(statistics_name).size();
  return mean_squared_angle / num_obs;
}


const x_view::real_t Evaluation::LocalizationEvaluation::
standardDeviationMeanSquaredDistance(const std::string& statistics_name) const {
  const x_view::real_t msd = MSD(statistics_name);
  x_view::real_t s = static_cast<x_view::real_t>(0.0);
  for(const x_view::LocalizationPair& p : statistics_map_.at(statistics_name)) {
    const x_view::real_t squared_distance =
        x_view::distSquared(p.estimated_pose.getPosition(),
                            p.true_pose.getPosition());
    s += (squared_distance - msd) * (squared_distance - msd);
  }
  const uint64_t num_obs = statistics_map_.at(statistics_name).size();
  return static_cast<x_view::real_t>(1.0 / (num_obs - 1) * std::sqrt(s));
}

const x_view::real_t Evaluation::LocalizationEvaluation::
standardDeviationMeanDistance(const std::string& statistics_name) const {
  const x_view::real_t md = MD(statistics_name);
  x_view::real_t s = static_cast<x_view::real_t>(0.0);
  for(const x_view::LocalizationPair& p : statistics_map_.at(statistics_name)) {
    const x_view::real_t distance = x_view::dist(p.estimated_pose.getPosition(),
                                                 p.true_pose.getPosition());
    s += (distance - md) * (distance - md);
  }
  const uint64_t num_obs = statistics_map_.at(statistics_name).size();
  return static_cast<x_view::real_t>(1.0 / (num_obs - 1) * std::sqrt(s));
}

const x_view::real_t Evaluation::LocalizationEvaluation::
standardDeviationMeanSquaredAngles(const std::string& statistics_name) const {
  const x_view::real_t msa = MSA(statistics_name);
  x_view::real_t s = static_cast<x_view::real_t>(0.0);
  for(const x_view::LocalizationPair& p : statistics_map_.at(statistics_name)) {
    const x_view::real_t angle = x_view::angle(p.estimated_pose, p.true_pose);
    const x_view::real_t squared_angle = angle * angle;
    s += (squared_angle - msa) * (squared_angle - msa);
  }
  const uint64_t num_obs = statistics_map_.at(statistics_name).size();
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

  for(const auto& p : statistics_map_) {
    const std::string& statistics_name = p.first;

    ss << getRightString(statistics_name) << col_sep;
    ss << getLeftString(std::to_string(MSD(statistics_name))) << col_sep;
    ss << getLeftString(std::to_string(standardDeviationMeanSquaredDistance(statistics_name))) << col_sep;
    ss << getLeftString(std::to_string(MD(statistics_name))) << col_sep;
    ss << getLeftString(std::to_string(standardDeviationMeanDistance(statistics_name))) << col_sep;
    ss << getLeftString(std::to_string(MSA(statistics_name))) << col_sep;
    ss << getLeftString(std::to_string(standardDeviationMeanSquaredAngles(statistics_name))) << col_sep;
    ss << getLeftString(std::to_string(p.second.size()));
    ss << "\n";
  }

  std::string s = "======== STATISTICS TABLE ========";
  s += "\n\n";
  return s + ss.str();

}
}


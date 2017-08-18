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

  // Create and delete all content of the new folder.
  system(("mkdir -p " + folder_name).c_str());
  system(("rm -rf " + folder_name + "*").c_str());

  bool success = true;
  success &= time.writeToFile(folder_name + "time.dat");
  success &= localization.writeToFile(folder_name + "localization.dat");

  return success;
}

bool Evaluation::TimerEvaluation::writeToFile(const std::string& filename) const {
  std::ofstream out(filename.c_str());
  if(!out.is_open()) {
    LOG(ERROR) << "Could not open file <" << filename << ">.";
    return false;
  }

  out << "Timer evaluation results.";

  return true;
}

const std::string Evaluation::TimerEvaluation::getTimingsTable() const {
  const auto& timer = x_view::Locator::getTimer();
  if(params_->timer_type == EvaluationParameters::TIMER_TYPE::NULL_TIMER) {
    return "Registered timer is of <nonMeasuring> type, this means that the "
        "registered instance does not measure any time.";
  }

  x_view::Timer* real_timer = dynamic_cast<x_view::Timer*>(timer.get());
  CHECK(real_timer != nullptr)
        << "Function " << __FUNCTION__ << " is only supported for timers of "
            "type <x_view::Timer>";

  return x_view::TimerPrinter::getTimingsTable(*real_timer);
}

const std::string Evaluation::TimerEvaluation::getTimingsTree() const {
  const auto& timer = x_view::Locator::getTimer();
  if(params_->timer_type == EvaluationParameters::TIMER_TYPE::NULL_TIMER) {
    return "Registered timer is of <nonMeasuring> type, this means that the "
        "registered instance does not measure any time.";
  }

  x_view::Timer* real_timer = dynamic_cast<x_view::Timer*>(timer.get());
  CHECK(real_timer != nullptr)
  << "Function " << __FUNCTION__ << " is only supported for timers of "
      "type <x_view::Timer>";

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

bool Evaluation::LocalizationEvaluation::writeToFile(const std::string& filename) const {
  std::ofstream out(filename.c_str());
  if(!out.is_open()) {
    LOG(ERROR) << "Could not open file <" << filename << ">.";
    return false;
  }

  out << "Localization evaluation results.";

  return true;
}

void Evaluation::LocalizationEvaluation::addLocalization(
    const std::string& statistics_name,
    const x_view::Vector3r& true_position,
    const x_view::Vector3r& estimated_position) {

  const x_view::real_t distance_squared =
      x_view::distSquared(true_position, estimated_position);
    statistics_map_[statistics_name].insert(distance_squared);
}

const x_view::real_t Evaluation::LocalizationEvaluation::MSD(
    const std::string& statistics_name) const {
  CHECK(statistics_map_.count(statistics_name) > 0)
        << "Requested MSD for statistics <" << statistics_name << "> but no "
            "statistic with that key has been registered.";

  return statistics_map_.at(statistics_name).mean();
}

const std::string Evaluation::LocalizationEvaluation::getStatisticsTable()
const {
  const uint64_t statistics_name_length = 15;
  const uint64_t col_width = 9;
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
  ss << getLeftString("MSQ") << col_sep;
  ss << getLeftString("num");
  ss << "\n";

  const uint64_t line_width = ss.str().length();
  ss << std::setfill('=') << std::setw(line_width);
  ss << "\n";

  for(const auto& p : statistics_map_) {
    const std::string& statistics_name = p.first;
    const Statistics& statistic = p.second;

    ss << getRightString(statistics_name) << col_sep;
    ss << getLeftString(std::to_string(MSD(statistics_name))) << col_sep;
    ss << getLeftString(std::to_string(statistic.numSamples()));
    ss << "\n";
  }

  std::string s = "======== STATISTICS TABLE ========";
  s += "\n\n";
  return s + ss.str();

}
}


from . import getLocalizations
import numpy as np
import os


def getMeanDistance(ground_truths, estimations):
    if len(ground_truths) != len(estimations):
        raise RuntimeError(
            "Estimation has {} poses, while ground truths has {}".format(len(estimations), len(ground_truths)))

    invalid_position = np.array([0, 0, 0])

    num_total = 0
    num_valid = 0
    num_invalid = 0
    distance_sum = 0.0

    for gt, es in zip(ground_truths, estimations):
        gt_pos = gt["position"]
        es_pos = es["position"]
        num_total += 1
        if (es_pos == invalid_position).all():
            num_invalid += 1
            continue
        else:
            distance = np.linalg.norm(gt_pos - es_pos)
            distance_sum += distance
            num_valid += 1

    if num_valid == 0:
        raise RuntimeError("All localizations are invalid")
    return distance_sum / num_valid


def computeSuccessRate(distance_thresholds, ground_truths, estimations):
    if len(ground_truths) != len(estimations):
        raise RuntimeError(
            "Estimation has {} poses, while ground truths has {}".format(len(estimations), len(ground_truths)))

    invalid_position = np.array([0, 0, 0])

    successes = []
    distances = []
    num_accepted = 0
    num_discarded = 0
    for gt, es in zip(ground_truths, estimations):
        dist = np.linalg.norm(gt["position"] - es["position"])
        if (es["position"] == invalid_position).all():
            num_discarded += 1
            continue
        else:
            distances.append(dist)
            num_accepted += 1

    if num_accepted == 0:
        return np.zeros(len(distance_thresholds)), 1.0

    for thresh in distance_thresholds:
        num_success = 0
        for dist in distances:
            if dist < thresh:
                num_success += 1
        successes.append(float(num_success) / num_accepted)

    return np.array(successes), float(num_discarded) / (num_discarded + num_accepted)


class XViewPR:
    def __init__(self, filename, true_threshold=None):

        if not os.path.exists(filename):
            print("File {} does not exist.".format(filename))
            exit(1)

        self._filename = filename

        true_threshold_ = true_threshold
        if true_threshold_ is None:
            true_threshold_ = 0.05
        self._true_threshold = true_threshold_

    def computePR(self, true_threshold=None):

        if true_threshold is None:
            true_threshold = self._true_threshold

        PR = []

        ground_truths, estimations, errors = getLocalizations(localization_file_name=self._filename)

        positive_radius = 0.5
        positive_radius_step = 0.5

        while positive_radius < 20:

            true_positives = 0
            true_negatives = 0
            false_positives = 0
            false_negatives = 0

            for gt, es, err in zip(ground_truths, estimations, errors):
                distance = np.linalg.norm(gt["position"] - es["position"])
                if err < true_threshold:
                    if distance < positive_radius:
                        true_positives += 1
                    else:
                        false_positives += 1
                else:
                    if distance < positive_radius:
                        false_negatives += 1
                    else:
                        true_negatives += 1

            precision = 1
            if true_positives + false_positives > 0:
                precision = (1.0 * true_positives) / (true_positives + false_positives)

            recall = 0
            if true_positives + false_negatives > 0:
                recall = (1.0 * true_positives) / (true_positives + false_negatives)

            PR.append([precision, recall])

            positive_radius += positive_radius_step

        return np.array(PR)

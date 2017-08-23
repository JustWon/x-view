from . import getLocalizations
import numpy as np
import os


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

        recall = 0
        positive_radius = 0.0
        positive_radius_step = 0.01

        while recall < 1:

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
                        true_negatives += 1
                else:
                    if distance < positive_radius:
                        false_positives += 1
                    else:
                        false_negatives += 1

            precision = 0
            if true_positives + false_positives > 0:
                precision = (1.0 * true_positives) / (true_positives + false_positives)

            recall = 0
            if true_positives + false_negatives > 0:
                recall = (1.0 * true_positives) / (true_positives + false_negatives)

            PR.append([precision, recall])

            positive_radius += positive_radius_step

        return np.array(PR)



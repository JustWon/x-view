from . import getLocalizations
import numpy as np
import os


class XViewPR:
    def __init__(self, filenames):
        filenames_ = filenames
        if type(filenames) is not list:
            filenames_ = [filenames]
        for filename in filenames_:
            if not os.path.exists(filename):
                print("File {} does not exist.".format(filename))
                exit(1)

        self._filenames = filenames_

    def computePR(self):

        PRs = []

        for filename in self._filenames:
            with open(filename, 'r') as file:
                ground_truths, estimations = getLocalizations(localization_file_name=filename)
                


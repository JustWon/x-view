from x_view_run import XViewRun, XViewConfig
from x_view_data import getTimes, getLastResultsDir, getLocalizations, getVertices, getEdges, XViewPR
import time
import matplotlib
import matplotlib.pyplot as plt
import csv
import os
import sys
import numpy as np
import math

def computeDBoWSuccessRate(distance_thresholds, dbow_results):

    invalid_position = np.array([0, 0, 0])

    successes = []
    distances = []
    num_accepted = 0
    num_discarded = 0
    final_accepted = 1
    final_discarded = 1
    yield_rejection_rate = 0.8
    error_thresh = 0.0
    # Try to get equal rejection rates for all experiments, e.g., 70%.
    while float(final_discarded) / (final_discarded + final_accepted) < yield_rejection_rate:
        num_discarded = 0
        num_accepted = 0
        distances = []
        for row in dbow_results:
            gt = np.array([row[0],row[1],row[2]])
            es = np.array([row[3],row[4],row[5]])
            similarity = row[6]
            
            dist = math.sqrt((gt[0] - es[0]) * (gt[0] - es[0]) + (gt[1] - es[1]) * (gt[1] - es[1]))
            if similarity < error_thresh:
                num_discarded += 1
                continue
            else:
                distances.append(dist)
                num_accepted += 1
    
        if num_accepted == 0:
            return np.zeros(len(distance_thresholds)), 1.0
        final_accepted = num_accepted
        final_discarded = num_discarded
        error_thresh = error_thresh + 0.01

    for thresh in distance_thresholds:
        num_success = 0
        for dist in distances:
            if dist < thresh:
                num_success += 1
        successes.append(float(num_success) / num_accepted)

    return np.array(successes), float(num_discarded) / (num_discarded + num_accepted)

# Parse the DBoW data, wich has an irregular file format with many spaces.
distance_thres = np.linspace(0, 100, 100)
file_name_dbow_airsim = "/home/johnny/Documents/Submissions/ICRA_2018/icra_2018_xview/results/dbow_airsim.txt"
file_name_dbow_synthia = "/home/johnny/Documents/Submissions/ICRA_2018/icra_2018_xview/results/dbow_synthia.txt"

dbow_airsim = [0,0,0,0,0,0,0]
with open(file_name_dbow_airsim, 'rb') as f:
    for line in f:
        line.strip()
        gtx = ''
        gty = ''
        gtz = ''
        estx = ''
        esty = ''
        estz = ''
        sim = ''
        counter = -1
        prev = False
        for n in line:
            
            if n != ' ':
                prev = False
                if counter == 0:
                    gtx += n
                if counter == 1:
                    gty += n
                if counter == 2:
                    gtz += n   
                if counter == 3:
                    estx += n
                if counter == 4:
                    esty += n
                if counter == 5:
                    estz += n
                if counter == 6:
                    sim += n
            else:
                if prev == False:
                    counter += 1
                    prev = True
        
        dbow_airsim = np.vstack((dbow_airsim, np.array([float(gtx), float(gty), float(gtz), float(estx), float(esty), float(estz), float(sim)])))    

dbow_airsim = np.delete(dbow_airsim, (0), axis=0)

dbow_synthia = [0,0,0,0,0,0,0]
with open(file_name_dbow_synthia, 'rb') as f2:
    for line in f2:
        line.strip()
        gtx = ''
        gty = ''
        gtz = ''
        estx = ''
        esty = ''
        estz = ''
        sim = ''
        counter = -1
        prev = False
        for n in line:
            
            if n != ' ':
                prev = False
                if counter == 0:
                    gtx += n
                if counter == 1:
                    gty += n
                if counter == 2:
                    gtz += n   
                if counter == 3:
                    estx += n
                if counter == 4:
                    esty += n
                if counter == 5:
                    estz += n
                if counter == 6:
                    sim += n
            else:
                if prev == False:
                    counter += 1
                    prev = True
        
        dbow_synthia = np.vstack((dbow_synthia, np.array([float(gtx), float(gty), float(gtz), float(estx), float(esty), float(estz), float(sim)])))    

dbow_synthia = np.delete(dbow_synthia, (0), axis=0)

success_rate_dbow_airsim, filtered = computeDBoWSuccessRate(distance_thres, dbow_airsim)
success_rate_dbow_synthia, filtered = computeDBoWSuccessRate(distance_thres, dbow_synthia)

pgf_with_rc_fonts = {"pgf.texsystem": "pdflatex"}
matplotlib.rcParams.update(pgf_with_rc_fonts)

plt.figure(figsize=(8,4))
plt.plot(success_rate_dbow_airsim, color="black", label="BL Airsim")
plt.plot(success_rate_dbow_synthia, '--', color="black", label="BL SYNTHIA")
plt.xlabel('Localization Accuracy [m]', fontsize=16)
plt.ylabel('Success rate [%]', fontsize=16)
leg = plt.legend(loc = 0)

plt.grid(True)
plt.show()
plt.clf()
plt.close()

# Plot PR
f, ax = plt.subplots(figsize=(16. / 2.5, 9. / 2.5))
x_view_pr_airsim = XViewPR(filename=file_name_dbow_airsim, true_threshold=20.0)
PR_airsim = x_view_pr_airsim.computePRDBoW(dbow_airsim,20.0)
order = PR_airsim[0:, 1].argsort()
ax.plot(PR_airsim[order, 1], PR_airsim[order, 0], label='DBoW Airsim')
x_view_pr_synthia = XViewPR(filename=file_name_dbow_airsim, true_threshold=15.0)
PR_synthia = x_view_pr_synthia.computePRDBoW(dbow_synthia,15.0)
order = PR_synthia[0:, 1].argsort()
ax.plot(PR_synthia[order, 1], PR_synthia[order, 0], label='DBoW SYNTHIA')

plt.title("PR curves")
plt.xlabel("Recall")
plt.ylabel("Precision")
plt.legend()

plt.ylim([-0.1, 1.05])
plt.xlim([-0.1, 1.05])
    
plt.tight_layout()
plt.show()



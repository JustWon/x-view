import numpy as np
import matplotlib.pylab as plt
import seaborn as sns


def accuracy_heatmap(param_1, param_2, accuracy, radii, extraction_radius, title, xlabel, ylabel, outpath="./output/"):
    # Extract unique values for the arrays and the respective number of different elements
    param_1_unique = np.unique(param_1)
    num_param_1 = len(param_1_unique)

    param_2_unique = np.unique(param_2)
    num_param_2 = len(param_2_unique)

    scores = np.zeros(shape=(num_param_1, num_param_2), dtype=object)
    avg_scores = np.zeros(shape=(num_param_1, num_param_2), dtype=float)
    std_dev_scores = np.zeros(shape=(num_param_1, num_param_2), dtype=float)

    labels = np.zeros(shape=(num_param_1, num_param_2), dtype='a50')
    params_per_cell = np.zeros(shape=(num_param_1, num_param_2), dtype=object)

    for i in range(num_param_1):
        current_val_1 = param_1_unique[i]
        for j in range(num_param_2):
            current_val_2 = param_2_unique[j]
            params_per_cell[i, j] = np.array([current_val_1, current_val_2])
            local_accuracy = []
            for k in range(len(accuracy)):
                if radii[k] == extraction_radius:
                    if param_1[k] == current_val_1 and param_2[k] == current_val_2:
                        local_accuracy.append(accuracy[k])

            scores[i, j] = local_accuracy
            if len(scores[i, j]) > 0:
                val = sum(scores[i, j]) / float(len(scores[i, j]))
                avg_scores[i, j] = val
                std_dev_scores[i, j] = np.std(np.array(local_accuracy))
                labels[i, j] = "mean: {0:.2f}\nstd: {1:.2f}".format(avg_scores[i, j], std_dev_scores[i, j])

    # Set up the matplotlib figure
    f, ax = plt.subplots(1, 1, figsize=(12, 9))

    sns.heatmap(avg_scores[::-1, :], square=True, robust=True, annot=labels[::-1, :], fmt='', linewidths=.5,
                vmin=0, vmax=1, xticklabels=param_2_unique, yticklabels=param_1_unique[::-1], cmap='RdYlGn')

    ax.set(title=title)
    ax.set(xlabel=xlabel, ylabel=ylabel)

    f.tight_layout()
    plt.savefig(outpath + title + ".png", dpi=250)

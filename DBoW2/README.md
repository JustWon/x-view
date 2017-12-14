DBoW2
=====

This is a port of the DBoW2 from https://github.com/dorian3d/DBoW2 for running reference place recognition experiments.
In the experiments folder, there are presently 2 expeirments performed on the Airsim and the SYNTHIA raw image data.

## Setting up and running experiments

First, build the package:

    $ cd DBoW2
    $ mkdir build
    $ cd build
    $ cmake ..
    $ make

For running the three experiments, make sure you have the dataset paths correctly set up in `experiments/airsim.cc`, `experiments/synthia_ff.cc`, and `experiments/synthia_fb.cc`.

Then simply run:

    $ ./experiments/airsim
    $ ./experiments/synthia_ff
    $ ./experiments/synthia_fb
    
from the root directory.

The results are written to `/tmp/dbow_airsim.txt`, `/tmp/dbow_synthia_ff.txt`, and `/tmp/dbow_synthia_fb.txt` respectively. The results are represented as a N x M matrix with N, the number of tested images and M = gt_x, gt_y, gt_z, etimation_x, estimation_y, estimation_z, similarity_score, rank=0.

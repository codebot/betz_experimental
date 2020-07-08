#!/usr/bin/env python3

# sudo apt install python3-sklearn

import csv
import math
import matplotlib.pyplot as plt
import numpy as np
from scipy import linalg
from sklearn.neighbors import NearestNeighbors
import statistics
import sys


if __name__=='__main__':
    if len(sys.argv) != 2:
        print('usage: decog.py LOG.csv')
        sys.exit(1)

    log_filename = sys.argv[1]
    print(f'loading {log_filename}')
    num_rows = 0
    #all_t = []
    #all_target = []
    #all_vel = []
    #all_effort = []
    #all_change_idx = []
    #prev_target = 0

    with open(log_filename) as csv_file:
        csv_reader = csv.reader(csv_file)

        # First, let's figure out how big of a numpy array to allocate
        num_rows = sum(1 for row in csv_reader)  # fastest way to do it (?)
        csv_file.seek(0)

        data = np.zeros((num_rows, 9))

        row_idx = 0
        for row in csv_reader:
            for col_idx in range(0, 9):
                data[row_idx, col_idx] = float(row[col_idx])
            row_idx += 1

            """
            all_t.append(float(row[0]))
            all_target.append(float(row[2]))
            all_vel.append(float(row[7]))
            all_effort.append(float(row[8]))

            if num_rows > 1:
                if all_target[-1] != prev_target:
                    all_change_idx.append(num_rows)
            prev_target = all_target[-1]
            """

    t_start = data[0, 0]
    t_end = data[-1, 0]
    elapsed_secs = t_end - t_start
    if elapsed_secs < 0:
        elapsed_secs += 4294967296.0
    elapsed_secs /= 1000000.0
    elapsed_mins = elapsed_secs / 60.0
    print(f'found {num_rows} rows')
    print(f'elapsed time: {elapsed_secs}s = {elapsed_mins:.3f} minutes')

    # compute a function approximation
    start_pos = -math.pi
    end_pos = math.pi
    num_pos = 16384
    approx_pos = np.linspace(start_pos, end_pos, num_pos)
    approx_effort = np.zeros(len(approx_pos))

    sigma = 0.001  # gaussian weights

    p = data[:, 2][::1]  # positions
    e = data[:, 8][::1]  # efforts

    # Currently using the approach given here:
    # https://xavierbourretsicotte.github.io/loess.html
    # Haven't spent any time working on speeding it up beyond using
    # NearestNeighbors to help avoid computing tons of useless weights

    print('computing nearest neighbor tree...')
    nn_tree = NearestNeighbors(1000, 0.2, leaf_size=100).fit(p.reshape(-1, 1))

    for idx, pos in enumerate(approx_pos):
        print(f'{idx}/{len(approx_pos)}')
        nn_dist, nn_idx = nn_tree.kneighbors(pos)
        # print('nn_dist', nn_dist)
        # print('nn_idx', nn_idx)

        nn_p = p[nn_idx]
        nn_e = e[nn_idx]

        w = np.exp(-(nn_p - pos)**2 / (2 * sigma**2))
        b = np.array([np.sum(w * nn_e), np.sum(w * nn_e * nn_p)])
        A = np.array(
            [
                [np.sum(w), np.sum(w * nn_p)],
                [np.sum(w * nn_p), np.sum(w * nn_p * nn_p)]
            ])
        x = linalg.solve(A, b)
        approx_effort[idx] = x[0] + x[1] * approx_pos[idx]

    with open('approximation.txt', 'w') as output_file:
        output_file.write(f'{start_pos:.9f} {end_pos:.9f} {num_pos}\n')
        for effort in approx_effort:
            output_file.write(f'{effort:.9f}\n')

    plt.plot(p[::1], e[::1], linewidth=0, marker='o', markersize=0.5)
    plt.plot(approx_pos, approx_effort, color='red', marker='o', markersize=5)
    plt.show()

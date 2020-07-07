#!/usr/bin/env python3

import csv
import math
import matplotlib.pyplot as plt
import numpy as np
import statistics
import sys

if __name__=='__main__':
    if len(sys.argv) != 2:
        print('usage: decog.py LOG.csv')
        sys.exit(1)

    log_filename = sys.argv[1]
    print(f'loading {log_filename}')
    num_rows = 0
    all_t = []
    all_target = []
    all_vel = []
    all_effort = []
    all_change_idx = []
    prev_target = 0

    with open(log_filename) as csv_file:
        csv_reader = csv.reader(csv_file)
        for row in csv_reader:
            num_rows += 1

            all_t.append(float(row[0]))
            all_target.append(float(row[2]))
            all_vel.append(float(row[7]))
            all_effort.append(float(row[8]))

            if num_rows > 1:
                if all_target[-1] != prev_target:
                    all_change_idx.append(num_rows)
            prev_target = all_target[-1]

    t_start = all_t[0]
    t_end = all_t[-1]
    elapsed_secs = t_end - t_start
    if elapsed_secs < 0:
        elapsed_secs += 4294967296.0
    elapsed_secs /= 1000000.0
    elapsed_mins = elapsed_secs / 60.0
    print(f'found {num_rows} rows')
    print(f'elapsed time: {elapsed_secs}s = {elapsed_mins:.3f} minutes')
    print(f'{len(all_change_idx)} target changes')

    change_t = []
    change_effort = []
    change_target = []
    for change_idx in all_change_idx:
        idx = change_idx - 2
        change_t.append(all_t[idx])
        change_effort.append(statistics.mean(all_effort[idx-5:idx]))
        change_target.append(all_target[idx])

    num_buckets = 16384
    bucket_width = 2 * math.pi / num_buckets
    bucket_centers = []
    bucket_efforts = []
    for i in range(0, num_buckets):
        bucket_centers.append(-math.pi + bucket_width/2 + i * bucket_width)
        bucket_efforts.append([])
    # print(bucket_centers)

    for idx in range(0, len(all_effort), 10):
        target = all_target[idx]
        if target < -math.pi or target > math.pi:
            continue

        min_dist = 1.0e9
        min_bucket_idx = -1
        for bucket_idx in range(0, num_buckets):
            bucket_dist = abs(target - bucket_centers[bucket_idx])
            # print(f'  bucket {bucket_idx} center {bucket_centers[bucket_idx]} dist {bucket_dist}')
            if bucket_dist < min_dist:
                min_dist = bucket_dist
                min_bucket_idx = bucket_idx

        # print(f'using bucket {min_bucket_idx} for target {target}')
        bucket_efforts[min_bucket_idx].append(all_effort[idx])

    # for bucket_effort in bucket_efforts:
    #     print(len(bucket_effort))

    bucket_means = []
    for bucket_effort in bucket_efforts:
        bucket_means.append(statistics.mean(bucket_effort))

    plt.plot(change_target, change_effort, linewidth=1.0)
    plt.plot(bucket_centers, bucket_means, linewidth=5.0, color='red', markersize=10)
    plt.show()

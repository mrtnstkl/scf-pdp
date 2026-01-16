"""
Q4 / Q5 Analysis Script (Python) — Batch Evaluation with Two Algorithm Paths

Assumptions:
- You have TWO base directories, one per algorithm
- Inside each base directory are subfolders per instance size
- Inside each size folder are plain .txt solution files

Example layout:

results_GA/
  n50/*.txt
  n100/*.txt

results_GRASP/
  n50/*.txt
  n100/*.txt
"""

import glob
import os
import pandas as pd
import numpy as np
from scipy import stats
import matplotlib.pyplot as plt

# --------------------------------------------------
# 1. Load a single solution file
# --------------------------------------------------

def load_solution(path: str, algorithm: str, size_label: str) -> dict:
    """
    Expected first line format (semicolon-separated):
    instance ; total_cost ; fairness ; time ; fairness_weight
    Remaining lines are ignored.
    """
    with open(path, "r") as f:
        first_line = f.readline().strip()

    parts = first_line.split(";")
    if len(parts) != 5:
        raise ValueError(f"Unexpected header format in {path}")

    return {
        "instance": parts[0],
        "size": size_label,
        "total_cost": float(parts[1]),
        "fairness": float(parts[2]),
        "time": float(parts[3]),
        "fairness_weight": float(parts[4]),
        "algorithm": algorithm,
    }


# --------------------------------------------------
# 2. Batch loading from two algorithm paths
# --------------------------------------------------

# PATHS
PATH_GA = "results/"
PATH_GRASP = "results_GRASP"

rows = []

for algo, base_path in [("GA", PATH_GA), ("GRASP", PATH_GRASP)]:
    for size_dir in sorted(os.listdir(base_path)):
        size_path = os.path.join(base_path, size_dir)
        if not os.path.isdir(size_path):
            continue

        for file in glob.glob(os.path.join(size_path, "*.txt")):
            rows.append(load_solution(file, algo, size_dir))


df = pd.DataFrame(rows)

print(f"Loaded {len(df)} runs")


# --------------------------------------------------
# 3. Q4 — Descriptive statistics (by size)
# --------------------------------------------------

stats_q4 = (
    df
    .groupby(["size", "algorithm"])
    .agg(
        mean_cost=("total_cost", "mean"),
        std_cost=("total_cost", "std"),
        best_cost=("total_cost", "min"),
        mean_time=("time", "mean"),
        runs=("total_cost", "count"),
    )
    .reset_index()
)

print("\nQ4 summary statistics (by size):\n")
print(stats_q4)


# --------------------------------------------------
# 4. Q5 — Statistical tests (per size)
# --------------------------------------------------

for size in sorted(df.size.unique()):
    print(f"\n=== Instance size: {size} ===")

    ga = df[(df.size == size) & (df.algorithm == "GA")]["total_cost"].values
    gr = df[(df.size == size) & (df.algorithm == "GRASP")]["total_cost"].values

    if len(ga) == 0 or len(gr) == 0:
        print("  Skipped (missing data)")
        continue

    # Welch t-test
    t_res = stats.ttest_ind(ga, gr, equal_var=False)
    print("  t-test (Welch):", t_res)

    # Wilcoxon signed-rank (only if paired)
    if len(ga) == len(gr):
        w_res = stats.wilcoxon(ga, gr)
        print("  Wilcoxon:", w_res)
    else:
        print("  Wilcoxon: skipped (unequal sample sizes)")

    # Mann–Whitney U
    mw_res = stats.mannwhitneyu(ga, gr, alternative="two-sided")
    print("  Mann–Whitney U:", mw_res)


# --------------------------------------------------
# 5. Plots (variance & scalability)
# --------------------------------------------------

# Boxplots per size
for size in sorted(df.size.unique()):
    subset = df[df.size == size]
    plt.figure()
    subset.boxplot(column="total_cost", by="algorithm")
    plt.title(f"Solution quality distribution — {size}")
    plt.suptitle("")
    plt.ylabel("Total cost")
    plt.show()

# Runtime scaling plot
plt.figure()
for algo in ["GA", "GRASP"]:
    means = (
        df[df.algorithm == algo]
        .groupby("size")["time"].mean()
    )
    plt.plot(means.index, means.values, marker="o", label=algo)

plt.xlabel("Instance size")
plt.ylabel("Mean runtime")
plt.legend()
plt.title("Runtime scalability")
plt.show()
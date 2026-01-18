import glob
import os
import pandas as pd
import numpy as np
from scipy import stats
import matplotlib.pyplot as plt
from pathlib import Path

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
SCRIPT_DIR = Path(__file__).resolve().parent

PATH_GA = SCRIPT_DIR / "results" / "ga"
PATH_GRASP = SCRIPT_DIR / "results" / "grasp"

rows = []

for algo, base_path in [("GA", PATH_GA), ("GRASP", PATH_GRASP)]:
    if not base_path.exists():
        raise FileNotFoundError(f"Missing directory: {base_path}")

    for size_dir in sorted(base_path.iterdir()):
        if not size_dir.is_dir():
            continue

        size_label = size_dir.name  # "50", "100", ...

        for file in size_dir.iterdir():
            if file.is_file():
                rows.append(load_solution(str(file), algo, size_label))



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
# 4. Q5 — Best-solution aggregation for statistical tests
# --------------------------------------------------

out_dir = SCRIPT_DIR / "tables"
out_dir.mkdir(exist_ok=True)

# 4.1 Best solution per instance and algorithm
best_per_instance = (
    df
    .groupby(["size", "instance", "algorithm"])["total_cost"]
    .min()
    .reset_index()
)

# 4.2 One CSV per instance size (paired GA vs GRASP)
for size in sorted(best_per_instance["size"].unique()):
    paired = (
        best_per_instance[best_per_instance["size"] == size]
        .pivot(index="instance", columns="algorithm", values="total_cost")
        .dropna()
        .reset_index()
    )

    paired.to_csv(
        out_dir / f"q5_best_solutions_size_{size}.csv",
        sep=";",
        index=False
    )

    print(
        f"Q5 size {size}: wrote {len(paired)} paired instances"
    )

# 4.3 One CSV over all sizes combined
paired_all = (
    best_per_instance
    .pivot(index="instance", columns="algorithm", values="total_cost")
    .dropna()
    .reset_index()
)

paired_all.to_csv(
    out_dir / "q5_best_solutions_all_sizes.csv",
    sep=";",
    index=False
)

print(
    f"Q5 all sizes: wrote {len(paired_all)} paired instances"
)

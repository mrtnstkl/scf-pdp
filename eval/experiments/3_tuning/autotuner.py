#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
(mu + lambda)-ES style tuning

initialise mu parents randomly
evaluate each parent (k repeats)

repeat G generations:
    generate lambda children via Gaussian mutation
    evaluate children
I
    select mu best from parents union children
    adapt mutation strength
"""

import random
import subprocess
from pathlib import Path
import csv
import statistics
from multiprocessing import Pool

# =========================
# GLOBAL CONFIG
# =========================

SEED = 7901
random.seed(SEED)

BASE_DIR = Path.cwd()
SCFPDP_EXE = BASE_DIR / "scfpdp"   # binary next to this script

ALGO = "genetic"          # "lns" or "genetic"
TUNING_MODE = 1           # 0 = random, 1 = adaptive (currently unused)
TIMEOUT_SEC = 300
REPEATS = 3

MAX_GENERATIONS = 2000
MAX_WORKERS = 96

SIZES = [50,100,200,500,1000]
INSTANCE_BASE = BASE_DIR / "instances"
OUTPUT_BASE = BASE_DIR / "results" / f"autotune_{ALGO}_mode{TUNING_MODE}"

# =========================
# PARAMETER RANGES
# =========================

LNS_RANGES = {
    "stagnation": (50, 500),
    "reaction": (0.05, 0.8),
}

GENETIC_RANGES = {
    "population": (300, 500),
    "elite_frac": (0.05, 0.4),
    "mutant_frac": (0.1, 0.5),
    "elite_bias": (0.4, 1.5),
}

# =========================
# PROGRESS
# =========================

class Progress:
    def __init__(self, total: int, every: int = 50):
        self.total = total
        self.every = every
        self.done = 0

    def update(self, n: int = 1):
        self.done += n
        if self.done % self.every == 0 or self.done == self.total:
            print(f"[progress] {self.done}/{self.total}", flush=True)

# =========================
# UTILITIES
# =========================

def random_individual(ranges):
    return {k: random.random() for k in ranges}

def denormalise(vec, ranges):
    out = {}
    for k, (lo, hi) in ranges.items():
        val = lo + vec[k] * (hi - lo)
        out[k] = int(val) if isinstance(lo, int) else round(val, 3)
    return out

def mutate(ind, sigma):
    return {
        k: min(1.0, max(0.0, v + random.gauss(0, sigma)))
        for k, v in ind.items()
    }

def run_instance(instance, params, out_file=None):
    if ALGO == "lns":
        cmd = [
            str(SCFPDP_EXE), "lns", str(instance),
            str(params["stagnation"]), str(params["reaction"])
        ]
    else:
        cmd = [
            str(SCFPDP_EXE), "genetic", str(instance),
            str(params["population"]),
            str(params["elite_frac"]),
            str(params["mutant_frac"]),
            str(params["elite_bias"]),
            str(MAX_GENERATIONS)
        ]

    if out_file is not None:
        cmd.append(str(out_file))

    try:
        result = subprocess.run(
            cmd,
            timeout=TIMEOUT_SEC,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            universal_newlines=True
        )
    except subprocess.TimeoutExpired:
        return None

    if result.returncode != 0:
        return None

    for line in result.stdout.splitlines():
        if "Solution with cost:" in line:
            try:
                return float(line.split(":")[1].strip())
            except ValueError:
                return None

    return None

def _eval_one(args):
    inst, params = args
    return run_instance(inst, params)

def evaluate(ind, instances, progress, ranges):
    params = denormalise(ind, ranges)
    tasks = [(inst, params) for inst in instances for _ in range(REPEATS)]

    costs = []
    with Pool(processes=MAX_WORKERS) as pool:
        for c in pool.imap_unordered(_eval_one, tasks):
            if progress is not None:
                progress.update(1)
            if c is not None:
                costs.append(c)

    return statistics.mean(costs) if costs else float("inf")

# =========================
# TUNING PER SIZE
# =========================

def tune_for_size(size):
    instance_dir = INSTANCE_BASE / str(size) / "train"
    instances = sorted(instance_dir.glob("*.txt"))
    if not instances:
        print(f"[size {size}] no instances found at {instance_dir}", flush=True)
        return

    ranges = LNS_RANGES if ALGO == "lns" else GENETIC_RANGES

    MU = 3
    LAMBDA = 6
    GENERATIONS = 5
    SIGMA = 0.2

    population = [random_individual(ranges) for _ in range(MU)]
    scores = {}

    total_evals = (MU + GENERATIONS * LAMBDA) * len(instances) * REPEATS
    progress = Progress(total=total_evals, every=50)

    for ind in population:
        scores[id(ind)] = evaluate(ind, instances, progress, ranges)

    for gen in range(GENERATIONS):
        offspring = [mutate(random.choice(population), SIGMA)
                     for _ in range(LAMBDA)]

        for ind in offspring:
            scores[id(ind)] = evaluate(ind, instances, progress, ranges)

        combined = population + offspring
        combined.sort(key=lambda ind: scores[id(ind)])
        population = combined[:MU]

        SIGMA *= 0.9
        print(f"[size {size}] gen {gen}: best={scores[id(population[0])]:.2f}", flush=True)

    best = population[0]
    best_params = denormalise(best, ranges)
    best_score = scores[id(best)]

    print(f"\n[size {size}] BEST CONFIG:", flush=True)
    print(best_params, flush=True)
    print(f"Average cost: {best_score:.2f}", flush=True)

    out_dir = OUTPUT_BASE / str(size)
    out_dir.mkdir(parents=True, exist_ok=True)

    report_file = out_dir / f"population_final_{size}.csv"
    with report_file.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["rank", "avg_cost", "params"])
        for rank, ind in enumerate(population):
            writer.writerow([rank, scores[id(ind)], denormalise(ind, ranges)])

    sol_dir = out_dir / "best_solutions"
    sol_dir.mkdir(exist_ok=True)

    for inst in instances:
        out_file = sol_dir / f"{inst.stem}.txt"
        run_instance(inst, best_params, out_file=out_file)

# =========================
# ENTRY POINT
# =========================

if __name__ == "__main__":
    for size in SIZES:
        tune_for_size(size)

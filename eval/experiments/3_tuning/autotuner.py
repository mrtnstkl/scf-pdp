"""
initialise μ parents randomly
evaluate each parent (k repeats)

repeat G generations:
    generate λ children via Gaussian mutation
    evaluate children
    select μ best from parents ∪ children
    adapt mutation strength

"""

import random
import subprocess
from pathlib import Path
import csv
import statistics
from tqdm import tqdm

# =========================
# GLOBAL CONFIG
# =========================

SEED = 7901
random.seed(SEED)

BASE_DIR = Path(__file__).resolve().parents[3]
SCFPDP_EXE = BASE_DIR / "build" / "release" / "scfpdp.exe"

ALGO = "lns"          # "lns" or "genetic"
TUNING_MODE = 1       # 0 = random, 1 = adaptive
TIMEOUT_SEC = 300
REPEATS = 3
ITERATIONS = 30

SIZES = [50,100,200,500,1000]
INSTANCE_BASE = BASE_DIR / "instances"
OUTPUT_BASE = BASE_DIR / "eval" / "experiments" / "3_tuning"

# =========================
# PARAMETER RANGES
# =========================

LNS_RANGES = {
    "stagnation": (50, 500),
    "reaction": (0.05, 0.8),
}

GENETIC_RANGES = {
    "population": (20, 200),
    "elite_frac": (0.05, 0.3),
    "mutant_frac": (0.05, 0.5),
    "elite_bias": (0.5, 2.0),
    "generations": (50, 500),
}

# =========================
# UTILITIES
# =========================

def normalise(params, ranges):
    return {
        k: (params[k] - lo) / (hi - lo)
        for k, (lo, hi) in ranges.items()
    }

def denormalise(vec, ranges):
    out = {}
    for k, (lo, hi) in ranges.items():
        val = lo + vec[k] * (hi - lo)
        out[k] = int(val) if isinstance(lo, int) else round(val, 3)
    return out

def random_individual(ranges):
    return {k: random.random() for k in ranges}


def mutate(ind, sigma):
    return {
        k: min(1.0, max(0.0, v + random.gauss(0, sigma)))
        for k, v in ind.items()
    }

def sample_params(ranges):
    params = {}
    for k, (lo, hi) in ranges.items():
        if isinstance(lo, int):
            params[k] = random.randint(lo, hi)
        else:
            params[k] = round(random.uniform(lo, hi), 3)
    return params


def perturb(params, ranges, strength=0.2):
    new = params.copy()
    for k, (lo, hi) in ranges.items():
        span = hi - lo
        delta = random.uniform(-strength, strength) * span
        val = max(lo, min(hi, new[k] + delta))
        new[k] = int(val) if isinstance(lo, int) else round(val, 3)
    return new


def run_instance(instance, params, out_file=None):
    if ALGO == "lns":
        cmd = [
            str(SCFPDP_EXE), "lns", str(instance),
            str(params["stagnation"]), str(params["reaction"])
        ]
    else:
        cmd = [
            str(SCFPDP_EXE), "genetic", str(instance),
            str(params["population"]), str(params["elite_frac"]),
            str(params["mutant_frac"]), str(params["elite_bias"]),
            str(params["generations"])
        ]

    if out_file is not None:
        cmd.append(str(out_file))

    try:
        result = subprocess.run(
            cmd,
            timeout=TIMEOUT_SEC,
            capture_output=True,
            text=True
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


def evaluate_config(instances, params, pbar, cutoff=None, min_samples=5):
    costs = []

    for inst in instances:
        for _ in range(REPEATS):
            cost = run_instance(inst, params)
            pbar.update(1)

            if cost is not None:
                costs.append(cost)

            if (
                cutoff is not None
                and len(costs) >= min_samples
                and statistics.mean(costs) > cutoff
            ):
                return float("inf")

    return statistics.mean(costs) if costs else float("inf")

def evaluate(ind, instances, pbar, ranges):
    params = denormalise(ind, ranges)
    costs = []

    for inst in instances:
        for _ in range(REPEATS):
            c = run_instance(inst, params)
            pbar.update(1)
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
        return

    ranges = LNS_RANGES if ALGO == "lns" else GENETIC_RANGES

    MU = 3
    LAMBDA = 6
    GENERATIONS = 5
    SIGMA = 0.2

    population = [random_individual(ranges) for _ in range(MU)]
    scores = {}

    total_evals = (MU + GENERATIONS * LAMBDA) * len(instances) * REPEATS

    with tqdm(total=total_evals, desc=f"Tuning size {size}") as pbar:

        for ind in population:
            scores[id(ind)] = evaluate(ind, instances, pbar, ranges)

        for gen in range(GENERATIONS):
            offspring = [mutate(random.choice(population), SIGMA)
                         for _ in range(LAMBDA)]

            for ind in offspring:
                scores[id(ind)] = evaluate(ind, instances, pbar, ranges)

            combined = population + offspring
            combined.sort(key=lambda ind: scores[id(ind)])
            population = combined[:MU]

            SIGMA *= 0.9  # mild annealing

            tqdm.write(
                f"[size {size}] gen {gen}: best={scores[id(population[0])]:.2f}"
            )

    best = population[0]
    best_params = denormalise(best, ranges)
    best_score = scores[id(best)]

    print(f"\n[size {size}] BEST CONFIG:")
    print(best_params)
    print(f"Average cost: {best_score:.2f}")

    # =====================
    # WRITE REPORT
    # =====================

    out_dir = OUTPUT_BASE / str(size)
    out_dir.mkdir(parents=True, exist_ok=True)

    report_file = out_dir / f"population_final_{size}.csv"
    with report_file.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["rank", "avg_cost", "params"])

        for rank, ind in enumerate(population):
            writer.writerow([
                rank,
                scores[id(ind)],
                denormalise(ind, ranges)
            ])


    # =====================
    # RE-RUN BEST & SAVE SOLUTIONS
    # =====================

    sol_dir = out_dir / "best_solutions"
    sol_dir.mkdir(exist_ok=True)

    for inst in instances:
        out_file = sol_dir / f"{inst.stem}.txt"
        run_instance(inst, best_params, out_file=out_file)

    print(f"\n[size {size}] BEST CONFIG:")
    print(best_params)
    print(f"Average cost: {best_score:.2f}")


# =========================
# ENTRY POINT
# =========================

if __name__ == "__main__":
    for size in SIZES:
        tune_for_size(size)

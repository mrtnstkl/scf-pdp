import random
import subprocess
import time
from pathlib import Path
import csv
import statistics

# =========================
# CONFIG
# =========================

SCFPDP_EXE = Path("./build/release/scfpdp.exe")

ALGO = "lns"       # "lns" or "genetic"
TUNING_MODE = 0    # 0 = random, 1 = adaptive
TIMEOUT_SEC = 300
REPEATS = 3

INSTANCE_ROOT = Path("./instances/50/tuning")
OUTPUT_LOG = Path("tuning_results.csv")

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
        val = new[k] + delta
        val = max(lo, min(hi, val))
        new[k] = int(val) if isinstance(lo, int) else round(val, 3)
    return new


def run_instance(instance, params):
    if ALGO == "lns":
        cmd = [
            str(SCFPDP_EXE),
            "lns",
            str(instance),
            str(params["stagnation"]),
            str(params["reaction"]),
        ]
    else:
        cmd = [
            str(SCFPDP_EXE),
            "genetic",
            str(instance),
            str(params["population"]),
            str(params["elite_frac"]),
            str(params["mutant_frac"]),
            str(params["elite_bias"]),
            str(params["generations"]),
        ]

    try:
        result = subprocess.run(
            cmd,
            timeout=TIMEOUT_SEC,
            capture_output=True,
            text=True,
        )
    except subprocess.TimeoutExpired:
        return None

    # parse first line: instance;cost;...
    try:
        line = result.stdout.splitlines()[0]
        cost = float(line.split(";")[1])
        return cost
    except Exception:
        return None


def evaluate_config(params):
    costs = []
    for inst in INSTANCE_ROOT.glob("*.txt"):
        for _ in range(REPEATS):
            c = run_instance(inst, params)
            if c is not None:
                costs.append(c)
    return statistics.mean(costs) if costs else float("inf")


# =========================
# MAIN TUNING LOOP
# =========================

def tune():
    ranges = LNS_RANGES if ALGO == "lns" else GENETIC_RANGES

    best_params = sample_params(ranges)
    best_score = evaluate_config(best_params)

    history = []

    for it in range(30):
        if TUNING_MODE == 0:
            candidate = sample_params(ranges)
        else:
            candidate = perturb(best_params, ranges)

        score = evaluate_config(candidate)

        history.append((it, score, candidate))

        if score < best_score:
            best_score = score
            best_params = candidate
            print(f"[+] Iter {it}: improved to {best_score:.2f}")

    return best_params, best_score, history


# =========================
# ENTRY POINT
# =========================

if __name__ == "__main__":
    best_params, best_score, history = tune()

    with OUTPUT_LOG.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["iteration", "avg_cost", "params"])
        for it, score, params in history:
            writer.writerow([it, score, params])

    print("\nBest configuration:")
    print(best_params)
    print(f"Average cost: {best_score:.2f}")

import pandas as pd
from pathlib import Path
import matplotlib.pyplot as plt

# ===== CONFIG =====
results_dir = Path("eval/results")
# ==================

records = []

for rf_dir in results_dir.iterdir():
    if not rf_dir.is_dir() or not rf_dir.name.startswith("rf_"):
        continue

    reaction_factor = float(rf_dir.name.replace("rf_", ""))

    for result_file in rf_dir.iterdir():
        with open(result_file, "r", encoding="utf-8") as f:
            line = f.readline().strip()

        instance, total_cost, fairness, execution_time, fairness_weight = [
            x.strip() for x in line.split(";")
        ]

        records.append({
            "instance": instance,
            "reaction_factor": reaction_factor,
            "total_cost": float(total_cost),
            "fairness": float(fairness),
            "execution_time": float(execution_time),
            "fairness_weight": float(fairness_weight),
        })

df = pd.DataFrame(records)

# ===== AGGREGATION =====
mean_cost = (
    df.groupby("reaction_factor", as_index=False)["total_cost"]
      .mean()
      .sort_values("reaction_factor")
)

best_rf = mean_cost.loc[mean_cost["total_cost"].idxmin()]

print("Best reaction factor (mean total cost):")
print(best_rf)

# ===== PLOT =====
plt.figure()
plt.plot(mean_cost["reaction_factor"], mean_cost["total_cost"])
plt.xlabel("Reaction factor")
plt.ylabel("Mean total cost")
plt.title("Mean total cost vs reaction factor")
plt.tight_layout()
plt.show()

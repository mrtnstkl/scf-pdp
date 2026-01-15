import subprocess
from pathlib import Path
from tqdm import tqdm

# ===== CONFIG =====
exe_path = Path("build/release/scfpdp.exe")
instances_dir = Path("instances/100/test")
output_base_dir = Path("eval/results")


stagnation_limit = 1000
reaction_factors = [0.01] + [round(i * 0.1 + 0.01, 2) for i in range(1, 10)] + [0.99]
# ==================

output_base_dir.mkdir(parents=True, exist_ok=True)

instances = list(instances_dir.glob("*.txt"))
total_runs = len(instances) * len(reaction_factors)

with tqdm(total=total_runs, desc="Running experiments") as pbar:
    for rf in reaction_factors:
        rf_dir = output_base_dir / f"rf_{rf:.2f}"
        rf_dir.mkdir(exist_ok=True)

        for instance in instances:
            output_file = rf_dir / instance.stem

            cmd = [
                str(exe_path),
                "lns",
                str(instance),
                str(stagnation_limit),
                str(rf),
                str(output_file)
            ]

            subprocess.run(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                check=True
            )

            pbar.update(1)
import subprocess
import time
from pathlib import Path
import tkinter as tk
from tkinter import ttk, filedialog, messagebox


# =========================
# ALGORITHM DEFINITIONS
# =========================

ALGORITHMS = {
    "greedy": {
        "params": {},
        "builder": lambda inst, p: ["greedy", inst],
    },
    "random": {
        "params": {},
        "builder": lambda inst, p: ["random", inst],
    },
    "beam": {
        "params": {
            "beam_width": ("int", 10),
        },
        "builder": lambda inst, p: ["beam", inst, str(p["beam_width"])],
    },
    "greedy_rand": {
        "params": {
            "candidates": ("int", 5),
            "tightening": ("float", 0.5),
        },
        "builder": lambda inst, p: [
            "greedy_rand",
            inst,
            str(p["candidates"]),
            str(p["tightening"]),
        ],
    },
    "localsearch": {
        "params": {
            "initial_solution": ("str", "greedy"),
            "neighborhood": ("str", "swap"),
            "strategy": ("enum", ["first", "best"]),
        },
        "builder": lambda inst, p: [
            "localsearch",
            inst,
            p["initial_solution"],
            p["neighborhood"],
            p["strategy"],
        ],
    },
    "grasp": {
        "params": {
            "candidates": ("int", 5),
            "tightening": ("float", 0.5),
            "neighborhood": ("str", "swap"),
            "strategy": ("enum", ["first", "best"]),
            "stagnation": ("int", 100),
        },
        "builder": lambda inst, p: [
            "grasp",
            inst,
            str(p["candidates"]),
            str(p["tightening"]),
            p["neighborhood"],
            p["strategy"],
            str(p["stagnation"]),
        ],
    },
    "lns": {
        "params": {
            "stagnation": ("int", 100),
            "reaction": ("float", 0.3),
        },
        "builder": lambda inst, p: [
            "lns",
            inst,
            str(p["stagnation"]),
            str(p["reaction"]),
        ],
    },
}


# =========================
# CORE LOGIC
# =========================

def discover_instances(root: Path):
    instances = []
    root = root.resolve()

    # Case 1: root contains .txt files directly
    for inst in root.glob("*.txt"):
        instances.append((root.name, inst))

    # Case 2: root contains test subfolders
    for test_dir in root.glob("*/test"):
        if test_dir.is_dir():
            size = test_dir.parent.name
            for inst in test_dir.glob("*.txt"):
                instances.append((size, inst))

    # Case 3: root IS a test folder
    if root.name == "test":
        size = root.parent.name
        for inst in root.glob("*.txt"):
            instances.append((size, inst))

    # DEDUPLICATION
    instances = list(
        {inst.resolve(): (size, inst) for size, inst in instances}.values()
    )

    return instances




def run_instance(
    exe: Path,
    algorithm: str,
    params: dict,
    instance: Path,
    size: str,
    timeout_sec: int,
    output_root: Path,
):
    out_dir = output_root / size
    out_dir.mkdir(parents=True, exist_ok=True)
    out_file = out_dir / instance.stem

    cmd = (
        [str(exe)]
        + ALGORITHMS[algorithm]["builder"](str(instance), params)
        + [str(out_file)]
    )

    start = time.time()
    try:
        subprocess.run(
            cmd,
            timeout=timeout_sec,
            capture_output=True,
            text=True,
        )
        status = "ok"
    except subprocess.TimeoutExpired:
        status = "timeout"

    return {
        "instance": instance.name,
        "size": size,
        "status": status,
        "time": round(time.time() - start, 2),
    }


# =========================
# GUI
# =========================

class SCFPDPGui(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("SCF-PDP Batch Evaluator")
        self.geometry("720x680")

        self.param_widgets = {}

        self._build_ui()

    def _build_ui(self):
        base = Path.cwd()

        # Paths
        self.exe_path = self._path_selector(
            "SCFPDP executable",
            default=str(base / "build" / "release" / "scfpdp.exe"),
            is_file=True,
        )

        self.input_path = self._path_selector(
            "Input instances root",
            default="instances",
            is_file=False,
        )

        self.output_path = self._path_selector(
            "Output root",
            default="eval",
            is_file=False,
        )

        # Algorithm
        ttk.Label(self, text="Algorithm").pack(anchor="w", padx=10)
        self.algo_var = tk.StringVar(value="greedy")
        ttk.OptionMenu(
            self,
            self.algo_var,
            "greedy",
            *ALGORITHMS.keys(),
            command=self._build_params,
        ).pack(fill="x", padx=10)

        # Params
        self.params_frame = ttk.LabelFrame(self, text="Algorithm parameters")
        self.params_frame.pack(fill="x", padx=10, pady=10)

        # Timeout
        ttk.Label(self, text="Timeout per instance (seconds)").pack(anchor="w", padx=10)
        self.timeout_var = tk.IntVar(value=300)
        ttk.Entry(self, textvariable=self.timeout_var).pack(fill="x", padx=10)

        # Run
        ttk.Button(self, text="Run evaluation", command=self.run).pack(pady=20)

        self._build_params("greedy")

    def _path_selector(self, label, default="", is_file=False):
        frame = ttk.Frame(self)
        frame.pack(fill="x", padx=10, pady=5)

        ttk.Label(frame, text=label).pack(anchor="w")

        var = tk.StringVar(value=default)
        ttk.Entry(frame, textvariable=var).pack(side="left", fill="x", expand=True)

        def browse():
            if is_file:
                path = filedialog.askopenfilename()
            else:
                path = filedialog.askdirectory()
            if path:
                var.set(path)

        ttk.Button(frame, text="Browse", command=browse).pack(side="right")
        return var

    def _build_params(self, algo):
        for w in self.params_frame.winfo_children():
            w.destroy()

        self.param_widgets.clear()
        params = ALGORITHMS[algo]["params"]

        if not params:
            ttk.Label(self.params_frame, text="No parameters").pack(anchor="w")
            return

        for name, (ptype, default) in params.items():
            ttk.Label(self.params_frame, text=name).pack(anchor="w")

            if ptype == "enum":
                var = tk.StringVar(value=default[0])
                ttk.OptionMenu(self.params_frame, var, default[0], *default).pack(fill="x")
            else:
                var = tk.StringVar(value=str(default))
                ttk.Entry(self.params_frame, textvariable=var).pack(fill="x")

            self.param_widgets[name] = (ptype, var)

    def _collect_params(self):
        params = {}
        for name, (ptype, var) in self.param_widgets.items():
            val = var.get()
            try:
                if ptype == "int":
                    val = int(val)
                elif ptype == "float":
                    val = float(val)
            except ValueError:
                raise ValueError(f"Invalid value for parameter '{name}'")
            params[name] = val
        return params

    def run(self):
        try:
            base = Path.cwd()

            def resolve_path(p: str) -> Path:
                p = Path(p)
                return p if p.is_absolute() else (Path.cwd() / p).resolve()

            exe = resolve_path(self.exe_path.get())
            input_root = resolve_path(self.input_path.get())
            output_root = resolve_path(self.output_path.get())


            if not exe.is_file():
                raise FileNotFoundError(f"Executable not found:\n{exe}")

            instances = discover_instances(input_root)
            if not instances:
                raise RuntimeError(f"No instances found under:\n{input_root}")

            params = self._collect_params()

        except Exception as e:
            messagebox.showerror("Error", str(e))
            return

        results = []
        for size, inst in instances:
            res = run_instance(
                exe=exe,
                algorithm=self.algo_var.get(),
                params=params,
                instance=inst,
                size=size,
                timeout_sec=self.timeout_var.get(),
                output_root=output_root,
            )
            results.append(res)

        messagebox.showinfo(
            "Done",
            f"Finished {len(results)} runs\n"
            f"Timeouts: {sum(r['status'] == 'timeout' for r in results)}",
        )


# =========================
# ENTRY POINT
# =========================

if __name__ == "__main__":
    app = SCFPDPGui()
    app.mainloop()

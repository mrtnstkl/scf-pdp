from pathlib import Path


def parse_solution_file(path: Path):
    with path.open() as f:
        lines = [l.strip() for l in f if l.strip()]

    header = [h.strip() for h in lines[0].split(";")]
    if len(header) != 5:
        raise ValueError(f"Invalid header format in {path}")

    instance = header[0]
    total_cost = float(header[1])
    fairness = float(header[2])
    execution_time = float(header[3])
    fairness_weight = float(header[4])

    routes = []
    for line in lines[1:]:
        routes.append(list(map(int, line.split())))

    num_drivers = len(routes)
    num_stops = sum(len(r) for r in routes)

    return {
        "instance": instance,
        "total_cost": total_cost,
        "fairness": fairness,
        "execution_time": execution_time,
        "fairness_weight": fairness_weight,
        "drivers": num_drivers,
        "stops": num_stops,
    }


def evaluate_folder(folder: Path):
    folder = Path(folder)
    if not folder.is_dir():
        raise FileNotFoundError(f"Folder not found: {folder}")

    results = []
    for file in folder.iterdir():
        if file.is_file():
            results.append(parse_solution_file(file))

    return results

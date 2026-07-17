import itertools
import json
import shutil
import signal
import subprocess
import sys
import time
import urllib.error
import urllib.parse
import urllib.request
from pathlib import Path

ROOT = Path(__file__).resolve().parent
BUILD = ROOT.parent.parent / "build"
PROFILE = ROOT / "profiles" / "japan_elevation.lua"
OUT = ROOT / "out"

NODES = {
    1: (139.0000, 35.0000),
    2: (139.0030, 35.0012),
    3: (139.0060, 35.0012),
    4: (139.0090, 35.0000),
    5: (139.0030, 34.9988),
    6: (139.0060, 34.9988),
    7: (139.0000, 35.0030),
    8: (139.0090, 35.0030),
    9: (139.0000, 34.9960),
    10: (139.0090, 34.9960),
}

ANNOTATIONS = "duration,weight,distance,nodes,speed"


def run(cmd, cwd=None):
    result = subprocess.run(cmd, cwd=cwd, capture_output=True, text=True)
    if result.returncode != 0:
        print(result.stdout)
        print(result.stderr)
        raise SystemExit(f"command failed: {' '.join(str(c) for c in cmd)}")


def build_base(target_dir, node_elevations=None):
    target_dir.mkdir(parents=True, exist_ok=True)
    shutil.copy(OUT / "fixture.osm.pbf", target_dir / "fixture.osm.pbf")
    extract_cmd = [BUILD / "osrm-extract", "fixture.osm.pbf", "-p", PROFILE]
    if node_elevations is not None:
        extract_cmd += ["--node-elevations", node_elevations]
    run(extract_cmd, cwd=target_dir)
    run([BUILD / "osrm-partition", "fixture.osrm"], cwd=target_dir)


def query(port, coords, params):
    coordinates = ";".join(f"{lon},{lat}" for lon, lat in coords)
    url = f"http://127.0.0.1:{port}/route/v1/foot/{coordinates}?{urllib.parse.urlencode(params)}"
    try:
        with urllib.request.urlopen(url, timeout=30) as response:
            return json.load(response)
    except urllib.error.HTTPError as error:
        return json.load(error)


def wait_ready(port):
    for _ in range(120):
        try:
            query(port, [NODES[1], NODES[4]], {"overview": "false"})
            return
        except (urllib.error.URLError, ConnectionError):
            time.sleep(0.25)
    raise SystemExit(f"router on {port} did not become ready")


def normalize(response):
    clone = json.loads(json.dumps(response))
    for route in clone.get("routes", []):
        route.pop("weight_name", None)
    return json.dumps(clone, sort_keys=True)


def main():
    if not (OUT / "fixture.osm.pbf").exists():
        run([sys.executable, ROOT / "make_fixture.py", OUT])

    datasets = {
        "fork_multi": OUT / "fork_multi",
        "vanilla_pop": OUT / "vanilla_pop",
        "vanilla_height": OUT / "vanilla_height",
    }
    for name, path in datasets.items():
        if path.exists():
            shutil.rmtree(path)
        build_base(path, OUT / "elevations.bin" if name == "fork_multi" else None)

    run(
        [
            BUILD / "osrm-customize",
            "fixture.osrm",
            "--metric",
            f"popularity:{OUT / 'popularity.csv'}",
            "--metric",
            f"height:{OUT / 'height.csv'}",
        ],
        cwd=datasets["fork_multi"],
    )
    run(
        [BUILD / "osrm-customize", "fixture.osrm", "--segment-speed-file", OUT / "popularity.csv"],
        cwd=datasets["vanilla_pop"],
    )
    run(
        [BUILD / "osrm-customize", "fixture.osrm", "--segment-speed-file", OUT / "height.csv"],
        cwd=datasets["vanilla_height"],
    )

    ports = {"fork_multi": 5085, "vanilla_pop": 5086, "vanilla_height": 5087}
    servers = []
    try:
        for name, port in ports.items():
            servers.append(
                subprocess.Popen(
                    [
                        BUILD / "osrm-routed",
                        datasets[name] / "fixture.osrm",
                        "--algorithm",
                        "mld",
                        "--ip",
                        "127.0.0.1",
                        "--port",
                        str(port),
                    ],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                )
            )
        for port in ports.values():
            wait_ready(port)

        pairs = list(itertools.permutations(sorted(NODES), 2))
        params = {
            "overview": "full",
            "geometries": "geojson",
            "steps": "true",
            "annotations": ANNOTATIONS,
            "alternatives": "false",
        }

        failures = []
        checks = 0
        divergent_routes = 0
        for a, b in pairs:
            coords = [NODES[a], NODES[b]]
            fork_pop = query(ports["fork_multi"], coords, {**params, "metric": "popularity"})
            fork_height = query(ports["fork_multi"], coords, {**params, "metric": "height"})
            fork_default = query(ports["fork_multi"], coords, params)
            van_pop = query(ports["vanilla_pop"], coords, params)
            van_height = query(ports["vanilla_height"], coords, params)

            checks += 1
            if normalize(fork_pop) != normalize(van_pop):
                failures.append(f"{a}->{b} metric=popularity differs from vanilla popularity")
            if normalize(fork_height) != normalize(van_height):
                failures.append(f"{a}->{b} metric=height differs from vanilla height")
            if normalize(fork_default) != normalize(fork_pop):
                failures.append(f"{a}->{b} default metric differs from first metric")
            if fork_pop.get("routes") and fork_pop["routes"][0].get("weight_name") != "popularity":
                failures.append(f"{a}->{b} popularity weight_name wrong")
            if (
                fork_height.get("routes")
                and fork_height["routes"][0].get("weight_name") != "height"
            ):
                failures.append(f"{a}->{b} height weight_name wrong")
            if normalize(fork_pop) != normalize(fork_height):
                divergent_routes += 1

        determinism_baseline = {}
        for repeat in range(30):
            for metric in ("popularity", "height"):
                response = normalize(
                    query(ports["fork_multi"], [NODES[1], NODES[4]], {**params, "metric": metric})
                )
                if metric not in determinism_baseline:
                    determinism_baseline[metric] = response
                elif determinism_baseline[metric] != response:
                    failures.append(f"alternating repeat {repeat} changed the {metric} response")
                    break

        for metric in ("popularity", "height"):
            with_elevation = query(
                ports["fork_multi"],
                [NODES[1], NODES[4]],
                {**params, "metric": metric, "annotations": ANNOTATIONS + ",elevation"},
            )
            for leg in with_elevation["routes"][0]["legs"]:
                node_ids = leg["annotation"]["nodes"]
                heights = leg["annotation"]["elevation"]
                if len(heights) != len(node_ids):
                    failures.append(f"{metric} elevation annotation length mismatch")
                    continue
                expected = [100.0 + node_id * 10.0 for node_id in node_ids]
                if any(abs(h - e) > 0.01 for h, e in zip(heights, expected)):
                    failures.append(f"{metric} elevation annotation values wrong")
        vanilla_elev = query(
            ports["vanilla_pop"],
            [NODES[1], NODES[4]],
            {**params, "annotations": "nodes,elevation"},
        )
        for leg in vanilla_elev["routes"][0]["legs"]:
            if any(h is not None for h in leg["annotation"]["elevation"]):
                failures.append("dataset without elevations returned non-null heights")

        unknown = query(
            ports["fork_multi"], [NODES[1], NODES[4]], {**params, "metric": "nonexistent"}
        )
        if unknown.get("code") != "InvalidValue":
            failures.append(f"unknown metric returned {unknown.get('code')}")

        st = query(ports["fork_multi"], [NODES[1], NODES[4]], {**params, "metric": "popularity"})
        st_lats = {c[1] for c in st["routes"][0]["geometry"]["coordinates"]}
        if 35.0012 not in st_lats:
            failures.append("popularity route did not choose the tagged north chain")
        st = query(ports["fork_multi"], [NODES[1], NODES[4]], {**params, "metric": "height"})
        st_lats = {c[1] for c in st["routes"][0]["geometry"]["coordinates"]}
        if 34.9988 not in st_lats:
            failures.append("height route did not choose the flat south chain")

        print(f"pairs checked: {checks}")
        print(f"responses where the two metrics differ: {divergent_routes}")
        if failures:
            for failure in failures:
                print("FAIL", failure)
            raise SystemExit(1)
        print("PASS: single dual-metric dataset is byte-equivalent to both vanilla datasets")
    finally:
        for server in servers:
            server.send_signal(signal.SIGTERM)
        for server in servers:
            server.wait(timeout=10)


if __name__ == "__main__":
    main()

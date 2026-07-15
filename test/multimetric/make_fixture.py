import sys
from pathlib import Path

import osmium


POP_MULTIPLIERS = {"0": 0.950, "1": 0.912, "2": 0.855, "3": 0.779, "4": 0.665, "5": 0.551}

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

WAYS = {
    101: {"nodes": [1, 2, 3, 4], "tags": {"highway": "unclassified", "foot": "yes", "yamap:popularity_class": "5"}},
    102: {"nodes": [1, 5, 6, 4], "tags": {"highway": "unclassified", "foot": "yes"}},
    103: {"nodes": [1, 7], "tags": {"highway": "footway", "foot": "yes", "yamap:popularity_class": "2"}},
    104: {"nodes": [7, 8], "tags": {"highway": "path", "foot": "yes", "bridge": "yes", "layer": "1", "yamap:popularity_class": "3"}},
    105: {"nodes": [8, 4], "tags": {"highway": "steps", "foot": "yes", "yamap:popularity_class": "0"}},
    106: {"nodes": [9, 10], "tags": {"highway": "motorway", "motorroad": "yes", "foot": "no"}},
    107: {"nodes": [1, 9], "tags": {"highway": "unclassified", "foot": "yes"}},
    108: {"nodes": [4, 10], "tags": {"highway": "unclassified", "foot": "yes"}},
}

BASE_SPEED = {"unclassified": 5.0, "path": 4.0, "footway": 3.0, "steps": 3.0}

HEIGHT_FACTORS = {101: 2.0, 103: 1.25, 105: 1.5}


def write_pbf(path):
    writer = osmium.SimpleWriter(str(path))
    for node_id in sorted(NODES):
        lon, lat = NODES[node_id]
        writer.add_node(osmium.osm.mutable.Node(id=node_id, location=(lon, lat), version=1))
    for way_id in sorted(WAYS):
        way = WAYS[way_id]
        writer.add_way(osmium.osm.mutable.Way(id=way_id, nodes=way["nodes"], tags=list(way["tags"].items()), version=1))
    writer.close()


def segment_rows(way):
    nodes = way["nodes"]
    return list(zip(nodes, nodes[1:]))


def write_popularity_csv(path):
    with path.open("w", encoding="ascii") as out:
        for way in WAYS.values():
            klass = way["tags"].get("yamap:popularity_class")
            if klass is None or way["tags"].get("foot") == "no":
                continue
            base = BASE_SPEED[way["tags"]["highway"]]
            rate = (base / 3.6) / POP_MULTIPLIERS[klass]
            for a, b in segment_rows(way):
                out.write(f"{a},{b},{base:.3f},{rate:.6f}\n")
                out.write(f"{b},{a},{base:.3f},{rate:.6f}\n")


def write_height_csv(path):
    with path.open("w", encoding="ascii") as out:
        for way_id, factor in sorted(HEIGHT_FACTORS.items()):
            way = WAYS[way_id]
            if way["tags"].get("bridge") == "yes" or way["tags"].get("tunnel") == "yes":
                continue
            base = BASE_SPEED[way["tags"]["highway"]]
            speed = base / factor
            rate = speed / 3.6
            for a, b in segment_rows(way):
                out.write(f"{a},{b},{speed:.3f},{rate:.6f}\n")
                out.write(f"{b},{a},{speed:.3f},{rate:.6f}\n")


def main():
    out_dir = Path(sys.argv[1])
    out_dir.mkdir(parents=True, exist_ok=True)
    write_pbf(out_dir / "fixture.osm.pbf")
    write_popularity_csv(out_dir / "popularity.csv")
    write_height_csv(out_dir / "height.csv")
    print(out_dir / "fixture.osm.pbf")


if __name__ == "__main__":
    main()

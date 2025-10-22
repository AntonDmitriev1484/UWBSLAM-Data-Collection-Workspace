#!/usr/bin/env python3
import argparse
import json
import copy
from pathlib import Path

def main():
    parser = argparse.ArgumentParser(description="Map nucX directories to their JSON arrays")
    parser.add_argument("trial_name", type=str, help="Name of the trial")
    args = parser.parse_args()

    trial_name = args.trial_name
    base_dir = Path(f"../out/{trial_name}_post")

    if not base_dir.exists() or not base_dir.is_dir():
        print(f"Error: directory {base_dir} does not exist")
        return

    nuc_json_map = {}

    for subdir in base_dir.iterdir():
        if subdir.is_dir() and subdir.name.startswith("nuc"):
            nuc_number = subdir.name[3:]  # get X from "nucX"
            json_file = subdir / f"{trial_name}_post" / "all.json"

            if not json_file.exists():
                print(f"Warning: {json_file} does not exist, skipping")
                continue

            with json_file.open("r") as f:
                try:
                    data = json.load(f)
                except json.JSONDecodeError as e:
                    print(f"Error decoding JSON in {json_file}: {e}")
                    continue

            tagged_data = []
            for d in data:
                tagged = d
                tagged["src"] = nuc_number
                tagged_data.append(tagged)
                
            nuc_json_map[nuc_number] = tagged_data

    # Optional: print the mapping or return it
    final_all_json = []
    for nuc, arr in nuc_json_map.items():
        final_all_json = final_all_json + arr

    mirrored_uwb = []
    for mes in final_all_json:
        if mes["type"] == "uwb":
            print(mes)
            mes_copy = copy.deepcopy(mes)
            mes_copy["src"] = mes["id"]
            mes_copy["id"] = mes["src"]
            mirrored_uwb.append(mes_copy)

    final_all_json = list(sorted(final_all_json + mirrored_uwb, key=lambda mes: mes["t"]))

    class NumpyEncoder(json.JSONEncoder):
        def default(self, obj):
            if isinstance(obj, np.ndarray):
                return obj.tolist()
            if hasattr(obj, '__dict__'):
                return vars(obj)
            return super().default(obj)

    with open(f"../out/{trial_name}_post/all.json", 'w') as fs:
        json.dump(final_all_json, fs, cls=NumpyEncoder, indent=1)


if __name__ == "__main__":
    main()

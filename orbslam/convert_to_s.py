import argparse

def convert_timestamps(input_path, output_path):
    with open(input_path, 'r') as fin, open(output_path, 'w') as fout:
        for line in fin:
            parts = line.strip().split()
            if not parts or len(parts) != 8:
                print(f"Skipping malformed line: {line}")
                continue
            try:
                timestamp_ns = float(parts[0])
                timestamp_s = timestamp_ns * 1e-9
                rest = " ".join(parts[1:])
                fout.write(f"{timestamp_s:.9f} {rest}\n")
            except ValueError:
                print(f"Invalid number format in line: {line}")
                continue

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Convert nanosecond timestamps to seconds.")
    parser.add_argument("input_file", help="Path to input file with timestamps in nanoseconds")
    parser.add_argument("output_file", help="Path to output file with timestamps in seconds")

    args = parser.parse_args()
    convert_timestamps(args.input_file, args.output_file)

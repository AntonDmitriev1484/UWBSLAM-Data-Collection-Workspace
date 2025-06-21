#!/bin/bash

# Directory to move into
DEST="./ros2"

# Create destination directory if it doesn't exist
mkdir -p "$DEST"

# Find all directories containing "pilot" in their name (excluding the destination itself)
find . -type d -name "*pilot*" ! -path "./ros2*" | while read dir; do
    echo "Moving: $dir -> $DEST"
    mv "$dir" "$DEST/"
done

echo "Done."

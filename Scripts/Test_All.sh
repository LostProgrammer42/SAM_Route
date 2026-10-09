#!/bin/bash

# ---------- CONFIG ----------
TEST_CELL_DIR="./Test_Cells"
ROUTE="./Route"
VISUALIZER="./Visualize_Tiles.py"

# ---------- COMPILE ----------
echo "Compiling Route..."
g++ -std=c++17 -g SAM_Route.cpp -o Route

# ---------- RUN ALL CELLS ----------
for rect_file in "$TEST_CELL_DIR"/*.rect; do

    # Get filename without directory or .rect extension
    cell_name=$(basename "$rect_file" .rect)

    echo ""
    echo "========================================"
    echo "Running: $cell_name"
    echo "========================================"

    # Remove previous routing output
    rm -f *.sam

    # Run router
    "$ROUTE" "$rect_file" polysilicon,m1

    # Generate visualization PDF
    python3 "$VISUALIZER" "$cell_name"

done

echo ""
echo "========================================"
echo "All test cells completed."
echo "========================================"
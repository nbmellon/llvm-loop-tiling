#!/bin/bash

# Pipeline to tile MLIR loops
# Usage: ./tile_pipeline.sh input.mlir output.mlir [tile-size]
# Note: This works best with affine.for loops without iter_args

INPUT=$1
OUTPUT=$2
TILE_SIZE=${3:-64}

if [ -z "$INPUT" ] || [ -z "$OUTPUT" ]; then
    echo "Usage: $0 input.mlir output.mlir [tile-size]"
    echo "  tile-size: tile size for loops (default: 64)"
    echo ""
    echo "This script tiles affine.for loops."
    echo "If your loops have iter_args (reduction variables), tiling will not work."
    echo "See README_TILING.md for more information."
    exit 1
fi

MLIR_OPT=/home/sashwat/llvm-mlir-pgeist/bin/mlir-opt

# Check if input has affine.for loops
if ! grep -q "affine.for" $INPUT 2>/dev/null; then
    echo "Input has scf.for loops instead of affine.for."
    exit 1;
fi

echo "Tiling affine loops with tile size $TILE_SIZE..."
$MLIR_OPT $INPUT \
    --affine-loop-tile="tile-sizes=$TILE_SIZE,$TILE_SIZE" \
    --canonicalize \
    --cse \
    -o $OUTPUT

if [ $? -eq 0 ]; then
    # Check if output actually has tiled loops (more nested loops)
    AFFINE_LOOPS_IN=$(grep -c "affine.for" $INPUT 2>/dev/null || echo "0")
    AFFINE_LOOPS_OUT=$(grep -c "affine.for" $OUTPUT 2>/dev/null || echo "0")
    
    if [ "$AFFINE_LOOPS_OUT" -gt "$AFFINE_LOOPS_IN" ]; then
        echo "Success! Tiling applied. Output written to $OUTPUT"
        echo "  Input had $AFFINE_LOOPS_IN affine.for loops"
        echo "  Output has $AFFINE_LOOPS_OUT affine.for loops"
    else
        echo "Warning: Tiling may not have been applied."
        echo "  Input had $AFFINE_LOOPS_IN affine.for loops"
        echo "  Output has $AFFINE_LOOPS_OUT affine.for loops"
        echo "  This usually means loops have iter_args which prevents tiling."
        echo "  Output written to $OUTPUT anyway."
    fi
    
    # Clean up temporary file
    rm -f ${OUTPUT}.affine.mlir
else
    echo "ERROR: Tiling failed"
    rm -f ${OUTPUT}.affine.mlir
    exit 1
fi

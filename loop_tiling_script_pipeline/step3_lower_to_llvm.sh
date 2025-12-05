LIB_PREFIX=/home/sashwat/llvm-mlir-pgeist/bin

$LIB_PREFIX/polygeist-opt $1 --lower-affine | $LIB_PREFIX/mlir-opt --convert-scf-to-cf | $LIB_PREFIX/mlir-opt --convert-to-llvm | $LIB_PREFIX/mlir-opt \
    --reconcile-unrealized-casts | $LIB_PREFIX/mlir-translate --mlir-to-llvmir -o $2

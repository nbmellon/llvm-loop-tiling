LIB_PREFIX=/home/sashwat/llvm-mlir-pgeist/bin
$LIB_PREFIX/cgeist $1 --function=$2 -S | $LIB_PREFIX/polygeist-opt --raise-scf-to-affine -o $3

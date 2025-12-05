LIB_PREFIX=/home/sashwat/llvm-mlir-pgeist/bin

$LIB_PREFIX/llc $1 -filetype=obj -o $2
echo "Object file: $2"
$LIB_PREFIX/clang $2 -o $3
echo "Executable: $3"
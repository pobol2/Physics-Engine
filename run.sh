if [ "$1" = "--build" -o "$1" = "-b" ]; then
    cmake --build ./build -j 4 > /dev/null
fi
./build/physics
if [ "$2" = "--plot" -o "$2" = "-p" ]; then
python3 python/plot.py
fi
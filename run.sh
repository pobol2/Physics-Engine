if [ "$1" = "--build" -o "$1" = "-b" ]; then
    cmake --build ./build -j 4 > /dev/null
fi
./build/physics
python3 python/plot.py
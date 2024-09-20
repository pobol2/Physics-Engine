if [ "$1" = "--build" -o "$1" = "-b" ]; then
    cmake --build ./build -j 4 > /dev/null
    # Check if the build command succeeded
    if [ $? -ne 0 ]; then
        echo "Build failed, not running ./build/physics."
        exit 1
    fi
fi
./build/physics
if [ "$2" = "--plot" -o "$2" = "-p" ]; then
python3 python/plot.py
fi
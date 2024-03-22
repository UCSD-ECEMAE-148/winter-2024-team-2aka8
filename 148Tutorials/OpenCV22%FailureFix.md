Many thanks to Winston for figuring this out!
If your opencv compilation fails at 22%, here is the fix:
nano build_opencv.sh
Within the file,
set CUDA_ARCH_BIN = 5.3, 6.2, 7.2     #delete 8.7

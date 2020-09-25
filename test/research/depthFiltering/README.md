# Results from CompareDepthFiltering.cpp

In this experiment, I aimed to compare realsense's ```spatial edge-preserving filter``` against
PCL's ```FastBilateralFilter```. Tuning different parameters and treating the clouds
through decimation filters, realsense's implementation clearly shows better results than PCL's implementation.

oh wait, nvm, let's just get new calibration scans lol

![section opening and closing](example.gif)
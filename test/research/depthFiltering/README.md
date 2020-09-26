# Results from CompareDepthFiltering.cpp

In this experiment, I aimed to compare realsense's ```spatial edge-preserving filter``` against
PCL's ```FastBilateralFilter```. Tuning different parameters and treating the clouds
through decimation filters, realsense's implementation clearly shows better results than PCL's implementation.

So far, I have found that minimal bilateral filtering leads to best plane detection results,
achieving only .179% error on this dataset.

![section opening and closing](example.gif)
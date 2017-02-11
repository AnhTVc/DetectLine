# DetectLine Project
Detect line from video to center line

This program is free software;
you can redistribute it and/or modify
it under the terms of the GNU General Public License version 2
as published by the Free Software Foundation.
# Require
- IE: Visial Studio 2012
- Lib: OpenCV 3.0
- Language: C/C++
# Describe
- Video to frame image
- CropImage take 1/4 frame
- Smooth: Median filter, Gauss
- Find line with canny and HoughLine
- process lines
  + reject near horizontal lines
  + filter with corner median
  + filer distance
  + Filline to line left and line right
  + calculate center line
  + using kalman filter with center line
  + complete
 
# Step to step
- Install IE: VS or code block
- Import OpenCV 3.0 to IE
- Clone project
- Import Project to IE
- Config file input in main class
- Or copy class main, util to your project do not forget file input

# Contact
Contact me: tranvietanh.hust@gmail.com


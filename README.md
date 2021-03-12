# PCD Horization Calibration
This reponsitory contain the code for horizontal calibration of point clouds

# Usage
1. Run the executable file main
2. You can see the "Select plane" and "viewer" visual windows
3. First press the "X" and select the Horizantal plane in the "Select" visual windows, then press the "Q" to confirm
4. Second you can see the Point cloud after calibration.

# Introduction to Directory Structure

1. pcdFiles: this directory contain the point cloud file that need to calibration.
2. calbiFiles: this directory contain the "resume.txt" and the Directory "Rotation_matrix"
3. Rotation_matrix: this directory contain the output calibration matrix.
4. resume.txt: this file is uesd to restore the previous calibration procedure, you can chose the file name to calibrate the file you specify

# Author

* **Zhengxi Hu**

Please raise an issue or send email to hzx@mail.nankai.edu.cn if there are any issues running the code.
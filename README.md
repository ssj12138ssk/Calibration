# Calibration
Calibrate camera and robotic arms.
Run get_camera_array.py to calibrate camera. You need to prepare a chessboard, count the number of internal corners, and measure the length of a black and white grid in reality. The calculation will obtain the camera intrinsic parameters and distortion coefficients.
Run get_camera_xy.py to obtain the coordinates of the target in the image, and run caculate_mdh.py to get the coordinates of the end of the arm.(Remember to modify the number of arms in Tnum8robot)
Record the obtained two-dimensional and three-dimensional coordinates(The calculated coordinate matrix of the end effector of the robotic arm includes rotation and translation, so the three-dimensional coordinates are in rows one to three of the fourth column of the matrix) into caculate_ceT.py.
You can make a test by run world2pixel.py.

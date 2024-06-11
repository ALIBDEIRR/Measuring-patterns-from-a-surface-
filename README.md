# Measuring-patterns-from-a-surface


The following work was the work for my internship as an end of study for masters in Robotics and Mechatronics.
The core of the work is to develope a C++ application for measuring patterns from the surface of laser textured samples. In this cas the patters have sphercal structure.

# File contents

source file: Includes the main script of the c++ code (main.cpp) and the other functions.
py scripts : Includes the python code for creating virtual point clouds.
images: give an examples of the approach developed through the code.
headers: Includes the header files included in the main script (main.cpp).


# How the code works
The code reads a point cloud data from a .pcd file (only) , visulaize this data points , and add so,e noise if wanted.
then the code implement an algorithm to try to fit the point cloud with a sphere if possible and execute its parameters. At the end the code visualize the results and the radius of the fitted sphere is printed to the terminal.

# Operating system
Thw work was done using linux , but you can run this code on windows or in MacOS.


# Tips
Be carefull to the paths defined in the scripts , you need to change paths and make them adapted based on your machine and files directories.


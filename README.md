# imu-constraint-cpp-el-wr
Code pertaining to the following manuscript: Drift-free joint angle calculation using inertial measurement units without magnetometers: an exploration of sensor fusion methods for the elbow and wrist

Tested on PC with Windows 10 with Visual Studio Code 1.76.2

# Contains the following algoriths:
`map_acc` Maximum a posteriori estimatior with linear acceleration constraint 
`map_elbow_acc` Maximum a posteriori estimatior with linear acceleration and elbow rotational constraint
`map_elbow_acc` Maximum a posteriori estimatior with linear acceleration and wrist rotational constraint

# Compile:
g++ -O3 -I .\eigen-3.4.0 kinematics.cpp run_map_acc.cpp map_acc.cpp -o map_acc

g++ -O3 -I .\eigen-3.4.0 kinematics.cpp run_map_elbow_acc.cpp map_elbow_acc.cpp -o map_elbow_acc

g++ -O3 -I .\eigen-3.4.0 kinematics.cpp run_map_wrist_acc.cpp map_wrist_acc.cpp -o map_wrist_acc

# Run: 
`./run_map_acc.sh` for map_acc algorithm
`./run_map_elbow_acc.sh` for map_elbow_acc algorithm
`./run_map_wrist_acc.sh` for map_wrist_acc algorithm

# Notes:
Specify file location and filter settings in the `.sh` file. 

# IMU Data Struture
## Inputs
Note: all scripts require a tab-delimited text file with the following structure:
columns 1 to 3: accel data for proximal segment (m/s^2)

columns 4 to 6: gyro data for proximal segment (m/s^2)

columns 7 to 9: accel data for distal segment (m/s^2)

columns 10 to 12: gyro data for distal segment (m/s^2)

## Outputs
columns 1 to 4: relative orientation in hamiltonian quaternion

columns 5 to 8: orientation of proximal in hamiltonian quaternion

columns 9 to 12: orientation of distal in hamiltonian quaternion
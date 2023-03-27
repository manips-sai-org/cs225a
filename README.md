# cs225a

This repository will contain the homeworks and demos for the class cs225a.

## Dependencies
The project depends on the sai2 libraries. You have received instructions to install sai2 in class.

## Build and run
in the main folder make a build folder and compile from there
```
mkdir build
cd build
cmake .. && make -j4
```
## run the code
go to the bin folder and then to the folder of the application you want to run.
for hw0 for example
```
cd bin/hw0
./hw0
```

### hw0
You have 2 programs there. A visualizer and the actual homework file.
The visualizer is here to help you make sure you are doing what you think you are doing.
To run it, go to bin/hw0 and run ./hw0_viz. You will see a window appear with the robot from hw0 in a configuration close to the one drawn on the pdf.
When you run hw0 and modify the values for the joints, it will modify the position of the visualized robot as long as you publish the new joint values to redis from hw0
```
redis_client.setEigenMatrixJSON(JOINT_ANGLES_KEY,robot->_q);
```

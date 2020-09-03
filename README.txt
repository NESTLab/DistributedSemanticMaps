1. Compile and install point cloud plugin

cd argos-point-cloud
mkdir build
cd build
cmake ../src
make 
sudo make install

2. Compile application

cd argos-application
mkdir build
cmake ..

3. Run experiments

Unit test:
cd argos-application
argos3 -c experiments/point_cloud_detector_unit_test.argos

Collective perception setup:
argos3 -c experiments/collective_perception.argos
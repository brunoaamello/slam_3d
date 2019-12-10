bash init.bash
g++ -Wall setupWiringPi.c -o setupWiringPi -lwiringPi
./setupWiringPi

cd ..
catkin_make --only-pkg-with-deps mouse_sensor scanner robot_ctr servo_ctr
cd src

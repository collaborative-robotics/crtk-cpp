# crtk-cpp


Build instructions:
----------
1. Run this first: 'catkin_make --pkg crtk_msgs'
2. Then run this: 'catkin_make'


Run instructions:
----------
1. make sure the package is built wothout error
2. Load parameters: 'rosparam load src/crtk-cpp/src/crtk_lib_cpp/config/params.yaml'
3. List the parameters: 'rosparam list'
4. Make sure parameters: **robot_namespace**, **arm_namespace**, **grasper_namespace**, and **max_joints** are all on the list.
5. Run the test of your choice. For instance, 'rosrun crtk_test_state crtk_test_state'


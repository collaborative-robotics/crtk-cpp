# crtk-cpp


Build instructions:
----------
1. Run this first: <pre><code>catkin_make --pkg crtk_msgs</pre></code>
2. Then run this: <pre><code>catkin_make</pre></code>


Run instructions:
----------
1. make sure the package is built without error
2. Load parameters from the robot roslaunch file.
3. List the parameters: <pre><code>rosparam list</pre></code>
4. Make sure parameters: **num_joints**, **home_pos**, **home_quat**, **home_jpos** and **grasper_name** are all on the list and under the robot namespace.
5. Run the test with rosrun and a **r_space** rosparameter specifying the robot namespace. For instance, <pre><code>rosrun crtk_test_servo_jp crtk_test_servo_jp _r_space:=arm1</pre></code>


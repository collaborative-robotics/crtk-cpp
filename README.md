# crtk-cpp


Build instructions:
----------
1. Run this first: <pre><code>catkin_make --pkg crtk_msgs</pre></code>
2. Then run this: <pre><code>catkin_make</pre></code>


Run instructions:
----------
1. make sure the package is built wothout error
2. Load parameters: <pre><code>rosparam load src/crtk-cpp/src/crtk_lib_cpp/config/params.yaml</pre></code>
3. List the parameters: <pre><code>rosparam list</pre></code>
4. Make sure parameters: **robot_namespace**, **arm_namespace**, **grasper_namespace**, and **max_joints** are all on the list.
5. Run the test of your choice. For instance, <pre><code>rosrun crtk_test_state crtk_test_state</pre></code>


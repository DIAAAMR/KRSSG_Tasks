# KRSSG_Tasks
  *ts.py, tc.py, tm.py are sockets program.
    To run them you need open server file ts.py first, then input number of minions in it.
    After that run tm.py file according to number of minions you require.
    Then in the end run tc.py, the client file, and input all the required data.
    
  *rrt_connect.py, rrt_connect_tree.py.
    Both files do basically same thing, but the rrt_connect_tree.py shows whole tree.
    The path of input image, value of unit step and obstacle check has been hard coded.
    Change them according to your requirments before running the code. Change the values in all obstacle checking functions.
    The unit length is defined as global l.
    
  *move_turtle.
   This package has dependencies roscpp, rospy, std_msg, turtlesim,  geometry_msgs.
   You will need tor create a newPosition.srv file with content provided in package.
   Follow the ros tutorials to create this package.
   For running the scripts, first run roscore, then turtlesim_node. After that run position.py then move.py.
   All the hardcodings present in rrt_connect are also present here, change it to your preference.
   The new Function path() assumes 11*11 as turtlesim_node dimension which is approximation.
   Also error(distance from destination) of each node is only accurade for this case, it will be inaacurate for non-square image.
   It only affects velocity and shape of path remains same. Still if you want to get more precise results change boh of them accordingly. 
   Rest of angles and points have been mapped accurately.

echo "Create map at current time"
source /root/catkin_ws/devel_isolated/setup.bash
# rosrun map_server map_saver -f $(rospack find "lumi_robot")/maps/mymap
# Finish the first trajectory. No further data will be accepted on it.
rosservice call /finish_trajectory 0

# Ask Cartographer to serialize its current state.
# (press tab to quickly expand the parameter syntax)
rosservice call /write_state "{filename: '/root/catkin_ws/src/beaverbot_localization/beaverbot_cartographer/maps/map.pbstream', include_unfinished_submaps: "true"}"
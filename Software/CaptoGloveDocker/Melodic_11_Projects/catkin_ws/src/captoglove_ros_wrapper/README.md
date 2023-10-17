# Necessary packages for simulation: 

```
rotors_simulator
mav_comm
gazebo_plugins
```



# TODO

- [x] Check linking of CaptoGloveAPI and threading, seems that ROS thread stops QThread
- [x] Get finger states from protobuffer message that's updated by CaptoGloveAPI
- [x] Add simulation to launch file (Added simple mav.launch to danieli_tcp_ip pkg) 
- [ ] Add simple control node (new ROS pkg/glove_uav_control) that sends commands to the UAV based on finger state 

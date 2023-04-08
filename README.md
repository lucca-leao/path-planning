# path-planning

Some path planning algorithms simulated in Stage ROS.

# Instructions
- Run:
```
roscore
```
- To start the simulation, open a new terminal and run:
```
rosrun stage_ros stageros -d worlds/<map_name>.world
```
- To run the A* algorithm, run:
```
python3 scripts/Astar.py
```
and enter the x,y target coordinates.
- To run the RRT algorithm, run:
```
python3 scripts/RRT.py
```
and enter the x,y target coordinates.
- To run the GVD algorithm, run:
```
python3 scripts/gvd.py
```
- To run the trapezoidal decomposition algorithm, run:
```
python3 scripts/boustrophedon.py
```
# Videos
- [A*](https://youtube.com/shorts/AgSWtU_pL00?feature=share)
- [RRT](https://youtube.com/shorts/Bc41di0hBDk?feature=share)
- [Boustrophedon](https://youtu.be/ldUx_XrVpcE)

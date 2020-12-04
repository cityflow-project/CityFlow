# Generator

`generate_grid_scenario.py` can generate NxM grid road network with traffic flows. 

### Quick Start

To generate a 3x4 grid road network

```
python generate_grid_scenario.py 3 4 
```

To generate a 3x4 grid road network with 2 straight lanes and with a predefined working traffic light plan

```
python generate_grid_scenario.py 3 4 --numStraightLanes 2 --tlPlan
```

### Other arguments
- `--rowDistance`: int, default=300, distance between consecutive intersections of each row (East-West Roads
- `--columnDistance`: int, default=300, distance between consecutive intersections of each column (South-North Roads
- `--intersectionWidth`: int, default=30
- `--numLeftLanes`: int, default=1
- `--numStraightLanes`: int, default=1
- `--numRightLanes`: int, default=1
- `--laneMaxSpeed`: int, default=16.67, meters/second
- `--vehLen`: float, default=5.0, meters
- `--vehWidth`: float, default=2.0, meters
- `--vehMaxPosAcc`: float, default=2.0
- `--vehMaxNegAcc`: float, default=4.5
- `--vehUsualPosAcc`: float, default=2.0
- `--vehUsualNegAcc`: float, default=4.5
- `--vehMinGap`: float, default=2.5
- `--vehMaxSpeed`: float, default=16.67
- `--vehHeadwayTime`: float, default=1.5
- `--dir`: str, default="./"
- `--roadnetFile`: str, generated road network file
- `--turn`: if specified, generate turning flows instead of straight flows
- `--tlPlan`: if specified, generate working predefined traffic signal plan instead of plans with default orders
- `--interval`: float, default=2.0, time (seconds) between each vehicle for each flow
- `--flowFile`: str, generated flow file

# SMAP Planning
The current version shows 3 path candidates which are evaluated in terms of their safety (reachability probability) as well as certainty (variance of voxels traversed by each path).

![True Map](https://github.com/Lolu28/smap/raw/doc/truemap.png)

The following plot shows the full map error evolution over time for the 3 different paths:
![Full Error](https://github.com/Lolu28/smap/raw/doc/fullerror.png)

Each path is curently evaluated on a fresh belief map with prior belief of 0.5. Measurements are taken along the path, starting at the start node and continuing with 3 more, equidistant points on the path.

Currently, the following statistics are computed:
|                | Reachability     | Variance     |
| -------------- | ----------------:| ------------:|
| Path 1 (red)   |       **39.60%** |     0.001038 |
| Path 2 (blue)  |           29.40% | **0.000909** |
| Path 3 (green) |           15.72% |     0.002696 |

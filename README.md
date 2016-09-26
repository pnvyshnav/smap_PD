# SMAP Planning
The current version shows 3 path candidates which are evaluated in terms of their safety (reachability probability) as well as certainty (variance of voxels traversed by each path).

![True Map](https://github.com/Lolu28/smap/raw/doc/truemap.png)

The following plot shows the full map error evolution over time for the 3 different paths:
![Full Error](https://github.com/Lolu28/smap/raw/doc/fullerror.png)

Each path is currently evaluated on a fresh belief map with prior belief of 0.5. Measurements are taken along the path, starting at the start node and continuing with 3 more, equidistant points on the path.

Currently, the following statistics are computed:

|                | Reachability     | Variance     |
| -------------- | ----------------:| ------------:|
| Path 1 (red)   |       **39.60%** |     0.001038 |
| Path 2 (blue)  |           29.40% | **0.000909** |
| Path 3 (green) |           15.72% |     0.002696 |

## Issues

### Computing variance
I used the formula for computing the [variance of the product of independent random variables](http://stats.stackexchange.com/a/52699). However, since the voxel densities and variance are all << 0, the overall variance also converges to 0 (less than 1E-10).

Instead, I am computing the average variance for each path, i.e. sum of variance of voxels along path divided by number of those voxels.

### Higher density in free area when robot is closer to obstacle
When the robot is close to the occupied areas, even the free voxels in front of them are believed to be occupied. The following figure illustrates the problem where the robot starts in pose `[0.3, 0.0, np.pi/2.]` on the same map as shown fully revealed in the image above. The voxels directly above and below the robot are believed to be occupied (with high certainty) while in reality, there is a gap of one free voxel in both directions until the occupied areas begin.

![Belief Map Problem](https://github.com/Lolu28/smap/raw/doc/beliefmapproblem.png)

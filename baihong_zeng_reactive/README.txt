this package helps the simulation car to run along the track while avoiding the obstacles.

video demo link: https://youtu.be/MyOgqdrQzxU

final approach:
1 preprocess the "scan_range" array, delete the "nan" range and produce a new array called "new_scan"
2 find the shortest range as the nearest point, use trigonometric functions to find the points in a circular sector and set all of them to be zero.
3 set a threshold, only the point with range larger than the threshold is considered to be in the gap. Find the maximum length gap.
4 find the disparity between adjacent points. If disparity happens, set an extra length of ranges to be zero in order to consider the width of the car.
5 find the middle index in the max gap and go towards that direction.

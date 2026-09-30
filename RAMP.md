# Configuring the ramp

Edit `include/config/ramp.hpp`, rebuild, and upload:

- `ENABLED`: whether the ramp is present.
- `CENTRE_X_MM` / `CENTRE_Y_MM`: measured centre in the fixed field coordinate system, in millimetres. These do not mirror with team colour.
- `LENGTH_AXIS`: `ramp::Axis::X` or `ramp::Axis::Y`, selecting the direction of the 1200 mm length. Width is always 400 mm.

The checked-in centre (1200, 2400) mm is a placeholder, with length along Y and the ramp enabled. Change it to match the physical field.

The map uses 50 mm tiles. All tiles intersecting the rectangle are covered; the outermost row along each long edge is permanently occupied (255). The interior is limited to unknown (127) or free (0?126): lidar returns can raise a free cell toward unknown but never occupied, and clear rays can lower it. The short ends have no imposed wall. Normal lidar mapping continues elsewhere. These rules also apply after clearing/resetting the map.

At a grid-aligned position, the ramp occupies 24 by 8 cells: two occupied side rows and six interior rows (300 mm). Off-grid positions round outward to cover the rectangle. Keep the whole rectangle inside the map boundary; the firmware checks this at compile time.

Pathfinding sees the ramp through the occupancy grid: its long edges are ordinary fully occupied cells, and its interior remains unknown or free. No ramp-specific behaviour is added to autonomous control, steering, path smoothing, or direct-path fallbacks.

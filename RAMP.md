# Configuring the ramp

With motors inhibited, move the joystick up/down to select a field and left/right to change it. Return the stick to centre between every movement, including each Ramp Y adjustment.

- `Ramp`: cycles `None`, `X`, `Y`. X/Y select the direction of the 1200 mm length; width is always 400 mm.
- `Ramp Y`: changes centre Y from 0 to 4800 mm in 100 mm steps. Each movement changes it once; holding the joystick does not repeat. It stops at either limit. The position can be edited even when mode is None.
- Centre X stays fixed at 1212 mm (`CENTRE_X_MM` in `include/config/ramp.hpp`).

The boot default is Ramp: None, with centre Y 1200 mm ready for selection. Menu selections last until power-off; boot defaults are in `include/config/ramp.hpp`. Coordinates use the fixed field frame, independent of team colour.

The occupancy grid applies a changed selection on the next lidar update and resets the map, removing the previous forced edges before rebuilding from lidar. Disabling the ramp restores normal mapping. The display never writes grid cells from its own task.

The map uses 50 mm tiles. The outermost row along each long edge is permanently occupied (255); the interior stays unknown (127) or free (0-126). Lidar hits can raise free cells toward unknown but never occupied, and clear rays can lower them. The short ends have no imposed ramp wall. At Y positions near a field limit, the rectangle is clipped to the grid and field boundary cells remain occupied.

Pathfinding sees the ramp through ordinary occupancy-grid cells. Autonomous control, steering, path smoothing, and direct-path fallbacks have no ramp-specific behaviour.

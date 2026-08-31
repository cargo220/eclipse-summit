# TARS Foxglove navigation layout

## 3D panel settings

- Fixed frame: `map`
- Display frame: `base_link`
- Follow mode: `Heading`
- 3D view: `2D`
- Global location: `/gps/fix`
- ENU frame: `map`
- Goal publish topic: `/goal_pose`
- Cancel service: `/navigation/cancel`

`map` is the fixed world frame. `base_link` is the moving robot frame, so
their origins should coincide only at the chosen initial datum; they must not
be forced to coincide during motion. The Heading follow mode keeps the robot
centered and rotates the view with its yaw while keeping the horizon level.

## Persisting the layout

Panel “Import/Export Settings” only changes the selected panel. To keep the
cancel button and 3D settings after reconnects or Jetson reboots:

1. Open the Layout menu next to `Default`.
2. Create or open a personal layout named `TARS Navigation`.
3. Add the Service Call panel and configure `/navigation/cancel`.
4. Configure the 3D panel with the values above.
5. Use the layout context menu and choose `Save changes`.
6. Export the whole layout from the same layout context menu as a backup.

The saved layout belongs to Foxglove on the laptop/browser, not to the ROS
container. Jetson rebooting does not need to recreate it once it is saved.

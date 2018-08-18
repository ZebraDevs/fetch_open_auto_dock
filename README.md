# Auto Docking for Fetch

Offers two actions:

 * dock - Used to dock the robot.
 * undock - Used to undock the robot (by backing up), optionally
   can rotate the robot 180 degrees so it is pointing off the dock

See the .action files and the scripts folder for documentation
of the action parameters and examples of calling the actions.

# Future Work

 * Improve perception and/or controls to avoid hard docks.
 * Add logic to avoid trying to dock and undock at same time.
 * Add support for use_move_base to get to pre-dock position.

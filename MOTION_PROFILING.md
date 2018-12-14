# Motion Profiling Quick Guide

For motion profiling, do the following:

1. Export a CSV file from [vannaka's Motion Profile Generator](https://github.com/vannaka/Motion_Profile_Generator) (on Windows, press Alt and spam arrow keys until you find the File menu).
2. Use `generate_motion_profile_class.py` to turn each side's motion profile CSV file into a Java class.
3. Instantiate a `MotionProfileFeeder` for each motor side in the yet-to-be-created command.

## TODO
* Add command that contains all the motion profile feeders based on [this](https://github.com/CrossTheRoadElec/Phoenix-Examples-Languages/blob/36b5ea48628421096a95c4685382f029ada28df8/Java/MotionProfileAuxiliary%5BMotionProfileArc%5D/src/org/usfirst/frc/team217/robot/Robot.java#L216).
# Motion Profiling Quick Guide

For motion profiling, do the following:

1. Export a CSV file from [vannaka's Motion Profile Generator](https://github.com/vannaka/Motion_Profile_Generator) (on Windows, press Alt and mess around with the arrow keys until you find the File menu).
2. Use `generate_motion_profile_class.py` to turn the motion profile CSV file into a Java class. Run the script with no arguments to see the arguments it takes. The resulting motion profile class will be saved in `commands/profiles`.
3. Instantiate a `MotionProfileCommand` for the motion profile and associated primary motor.
4. Start the command (profile playback begins immediately). The command will exit after playback is complete.
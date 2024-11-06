Shared Code - Update (November 11th, 2024)


This is Shared Code, a repository meant to be forked every year. The code in this is meant to be used every year under the assumption that we are using a mecanum drive. It contains many functions needed for Field Relative Drive. For additional functions that may be reused, see the latest Teamcode base, (as of writing, it is IntoTheDeep2025Fork). 

Create a new folder at or above the directory containing "mecanum" and "util" called "current". This will house the Commands, opmodes, subsystems, and any other code specific to the season. A common Drive paradigm that we have used is:

OpMode (typically a CommandBasedOpMode) -> Commands (Different Commands for Arm, Intake, Drive, etc) -> Mecanum20XX (Many commands will reference your Mecanum subsystem. This can be to use specific variables, or to call specific functions inside of it.) -> BaseMecanumDrive (This is the main file contained within this Repository, and contains many of the default Drive functions such as MoveFieldRelative, MoveFieldRelativeForPID, and move. For FieldRelativeDrive, it bypasses the Mecanum subsystem and calls BaseMecanumDrive directly)


For more resources, see:
https://gm0.org/en/latest/	(Incredibly useful for covering many programming related topics) 
https://cookbook.dairy.foundation/introduction.html	(Useful for roadrunner 1.0 or 0.56, PID Controllers, and wiring Odo pods)


To read more about PID loops specifically, see:
https://gm0.org/en/latest/docs/software/concepts/control-loops.html


<h1>Shared Code - Update (November 11th, 2024)</h1>


This is Shared Code, a repository meant to be forked every year. The code in this is meant to be used every year under the assumption that we are using a mecanum drive. It contains many functions needed for Field Relative Drive. For additional functions that may be reused, see the latest Teamcode base, (as of writing, it is IntoTheDeep2025Fork). 



Create a new folder at or above the directory containing "mecanum" and "util" called "current". This will house the Commands, opmodes, subsystems, and any other code specific to the season. A common Drive paradigm that we have used is:

OpMode (typically a CommandBasedOpMode) -> Commands (Different Commands for Arm, Intake, Drive, etc) -> Mecanum20XX (Many commands will reference your Mecanum subsystem. This can be to use specific variables, or to call specific functions inside of it.) -> BaseMecanumDrive (This is the main file contained within this Repository, and contains many of the default Drive functions such as MoveFieldRelative, MoveFieldRelativeForPID, and move. For FieldRelativeDrive, it bypasses the Mecanum subsystem and calls BaseMecanumDrive directly)


For more resources, see:
https://gm0.org/en/latest/	(Incredibly useful for covering many programming related topics) 
https://cookbook.dairy.foundation/introduction.html	(Useful for roadrunner 1.0 or 0.56, PID Controllers, and wiring Odo pods)


To read more about PID loops specifically, see:
https://gm0.org/en/latest/docs/software/concepts/control-loops.html




You will typically have a PIDController for different parts of your robot. For example, a translationController, RotationController, and SlideController. These all contain unique PID values, (Proportional, Integral, and Derivative). You must tune these values to find the best error control. Often, all 3 are not necessary, but you will need to have at least P, and possibly I. 




To tune these values, and other values easily, use FTCDashboard. Connect to your robots wifi, then go to:
192.168.43.1:8080/dash

For more information on FTCDashboard, see:
https://acmerobotics.github.io/ftc-dashboard/gettingstarted





You will then be able to configure your values in real time as long as you put them within this syntax:

@Config
public static class NAME {
  int number = 3; 
}

You will then be able to change the value of "number" live on FTCDashboard and see the changes that occur on the robot.  






// KINEMATIC EQUATIONS //

Equation for Heading: (Heading is simply where the robot is facing, always starts at zero)

Theta = ( R - L )/Distance between right and left odometry pods

Theta = Variable for heading
R = Distance that Right Odometry Pod has moved
L = Distance that Left Odometry Pod has moved

For a visual representation of this and more similar concepts, see:
https://www.youtube.com/watch?v=ixsxDn_ddLE&t=150s

//////////////////////////////////







// TELEMETRY PACKETS //

Telemetry Packets are very important in the debugging process (something that you will probably run into quite a bit). Telemtry packets have an advantage over the traditional:
telemetry.addData("Sample Data", x);

Because they are able to be run in any file as opposed to "telemetry.addData" or "telemetry.addLine", which can only be run within OpMode loops. This restricts what data you can view.
The syntax for making a telemetry packet can be found here:
https://acmerobotics.github.io/ftc-dashboard/features.html

The general structure is:

TelemetryPacket newPacket = New TelemetryPacket(); 

newPacket.put("X Distance Travelled ",  m_odo.getPose().getX());
// The "put" method takes in two parameters, a label, which can be anything you want, and a variable. In the example above I grabbbed the X distance travelled according to m_odo (which gathers that information from the odometry pods, which are passed into it)

However before seeing this data, you must also create an FtcDashboard instance, and pass the packet into it, using the following syntax

FtcDashboard dashboard = FtcDashboard.getInstance();
dashboard.sendTelemetryPacket(newPacket);

Then you will be able to see the packet by going to the same website as mentioned above:
192.168.43.1:8080/dash




 

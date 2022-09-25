package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Drivetrain;
import frc.robot.commands.Drive;
import frc.robot.commands.FollowPath;

public class RobotContainer {

    private XboxController Driver;
    private final Drivetrain mDrivetrainSubsystem = new Drivetrain();
    
	SendableChooser<Command> mChooser = new SendableChooser<>();
    

    

    public Drivetrain GetDrivetrainSubsystem() {
        return mDrivetrainSubsystem;
    }

    public RobotContainer() {
        Driver = new XboxController(0);
        CommandScheduler.getInstance().registerSubsystem(mDrivetrainSubsystem);
        CommandScheduler.getInstance().setDefaultCommand(mDrivetrainSubsystem,
                new Drive(mDrivetrainSubsystem, Driver, true));


        SmartDashboard.putData(mChooser);
        mChooser.setDefaultOption("Auto", new FollowPath(Paths.Auto, mDrivetrainSubsystem));
        mChooser.addOption("Test Path", new FollowPath(Paths.TestPath, mDrivetrainSubsystem));
        mChooser.addOption("Test Path 2", new FollowPath(Paths.TestPath2, mDrivetrainSubsystem));

        
    }


    public Command getAutonomousCommand() {
		// The selected command will be run in autonomous

		// Reset odometry to the starting pose of the trajectory.
		return mChooser.getSelected();

	}

}

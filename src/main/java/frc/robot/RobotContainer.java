package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Drivetrain;
import frc.robot.commands.Drive;

public class RobotContainer {

    private XboxController Driver;
    private final Drivetrain mDrivetrainSubsystem = new Drivetrain();
    
    

    

    public Drivetrain GetDrivetrainSubsystem() {
        return mDrivetrainSubsystem;
    }

    public RobotContainer() {
        Driver = new XboxController(0);
        CommandScheduler.getInstance().registerSubsystem(mDrivetrainSubsystem);
        CommandScheduler.getInstance().setDefaultCommand(mDrivetrainSubsystem,
                new Drive(mDrivetrainSubsystem, Driver, true));
    }

}

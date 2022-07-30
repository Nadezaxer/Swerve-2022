package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.math.filter.SlewRateLimiter;
import frc.robot.Constants.DRIVER;
import frc.robot.Constants.HARDWARE;
import frc.robot.subsystems.Drivetrain;
import frc.robot.commands.Drive;

public class RobotContainer {
    
    private XboxController Driver;
    // private final Joystick mLeftJoystick = new Joystick( HARDWARE.LEFT_JOYSTICK );;
    // private final Joystick mRightJoystick = new Joystick( HARDWARE.RIGHT_JOYSTICK );;
    private final Drivetrain mDrivetrainSubsystem = new Drivetrain();
    private final SlewRateLimiter mForwardSpeedLimiter = new SlewRateLimiter( DRIVER.DRIVE_SLEW_RATE_LIMITER );
    private final SlewRateLimiter mStafeSpeedLimiter = new SlewRateLimiter( DRIVER.DRIVE_SLEW_RATE_LIMITER );
    private final SlewRateLimiter mTurnSpeedLimiter = new SlewRateLimiter( DRIVER.DRIVE_SLEW_RATE_LIMITER );


    private double applyDeadband ( double value, double deadband ) {
        if ( Math.abs( value ) > deadband ) {
            if ( value > 0.0 ) {
                return (value - deadband) / ( 1.0 - deadband );
            } else {
                return ( value + deadband ) / ( 1.0 - deadband );
            }
        } else {
            return 0.0;
        }
    }
    
    private double GetDriveForward () {
        return mForwardSpeedLimiter.calculate( applyDeadband( -Driver.getLeftY(), DRIVER.JOYSTICK_DEADBAND ) ) * DRIVER.MAX_DRIVE_VELOCITY;
    }

    private double GetDriveStrafe () {
        return -mStafeSpeedLimiter.calculate( applyDeadband( Driver.getLeftX(),
            DRIVER.JOYSTICK_DEADBAND ) ) * DRIVER.MAX_DRIVE_VELOCITY;
    }

    private double GetDriveRotation () {
        return mTurnSpeedLimiter.calculate( applyDeadband( -Driver.getRightX(),
            DRIVER.ROTATION_DEADBAND ) ) * DRIVER.MAX_ROTATION_VELOCITY;
    }

    public Drivetrain GetDrivetrainSubsystem () {
        return mDrivetrainSubsystem;
    }

    public RobotContainer() {
        Driver = new XboxController(0);
        CommandScheduler.getInstance().registerSubsystem( mDrivetrainSubsystem );
        CommandScheduler.getInstance().setDefaultCommand( mDrivetrainSubsystem, 
            new Drive( mDrivetrainSubsystem, this::GetDriveForward, this::GetDriveStrafe, this::GetDriveRotation, true ) );
    }

}

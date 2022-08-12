package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DRIVER;
import frc.robot.Constants.DRIVETRAIN;
import frc.robot.subsystems.Drivetrain;

public class Drive extends CommandBase {
    private Drivetrain mDrivetrain;
    private boolean mFieldOriented;

    private double prevTime;
    private PIDController mTurnPID;

    private final SlewRateLimiter mForwardSpeedLimiter = new SlewRateLimiter(DRIVER.DRIVE_SLEW_RATE_LIMITER);
    private final SlewRateLimiter mStafeSpeedLimiter = new SlewRateLimiter(DRIVER.DRIVE_SLEW_RATE_LIMITER);
    private double mTargetHeading;
    private XboxController mDriver;

    private double applyDeadband(double value, double deadband) {
        if (Math.abs(value) > deadband) {
            if (value > 0.0) {
                return (value - deadband) / (1.0 - deadband);
            } else {
                return (value + deadband) / (1.0 - deadband);
            }
        } else {
            return 0.0;
        }
    }

    public Drive(Drivetrain drivetrain, XboxController driver, boolean fieldOriented) {
        mDrivetrain = drivetrain;
        mFieldOriented = fieldOriented;
        mDriver = driver;

        prevTime = RobotController.getFPGATime() / 1e6;
        mTargetHeading = mDrivetrain.GetHeading();

        mTurnPID = new PIDController(DRIVETRAIN.HEADING_P_GAIN, DRIVETRAIN.HEADING_I_GAIN, DRIVETRAIN.HEADING_D_GAIN);

        addRequirements(mDrivetrain);

        SmartDashboard.putBoolean("Field Orientated", fieldOriented);
    }

    @Override
    public void execute() {
        double xSpeed = mForwardSpeedLimiter.calculate(applyDeadband(-mDriver.getLeftY(), DRIVER.JOYSTICK_DEADBAND))
                * DRIVER.MAX_DRIVE_VELOCITY;
        double ySpeed = -mStafeSpeedLimiter.calculate(applyDeadband(mDriver.getLeftX(), DRIVER.JOYSTICK_DEADBAND))
                * DRIVER.MAX_DRIVE_VELOCITY;
        double roatationSpeed = applyDeadband(mDriver.getRightX(), DRIVER.JOYSTICK_DEADBAND) * DRIVER.MAX_ROTATION_VELOCITY;

        mFieldOriented = SmartDashboard.getBoolean("Field Orientated", mFieldOriented);

        double now = RobotController.getFPGATime() / 1e6;
        double delta_sec = now - prevTime;

        prevTime = now;

        mTargetHeading += roatationSpeed * delta_sec;

        if (mTargetHeading > Math.PI) {
            mTargetHeading -= 2 * Math.PI;
        }
        if (mTargetHeading < -Math.PI) {
            mTargetHeading += 2 * Math.PI;
        }

        SmartDashboard.putNumber("X", xSpeed);
        SmartDashboard.putNumber("Y", ySpeed);
        SmartDashboard.putNumber("Target Heading", mTargetHeading);
        SmartDashboard.putNumber("Roatation Speed", roatationSpeed);
        SmartDashboard.putNumber("Time Delta", delta_sec);
        SmartDashboard.putNumber("Drivetrain Heading", mDrivetrain.GetHeading());
        
        double error = mDrivetrain.GetHeading() - mTargetHeading;



        if (error > Math.PI) {
            error -= 2 * Math.PI;
        }
        if (error < -Math.PI) {
            error += 2 * Math.PI;
        }

        SmartDashboard.putNumber("Error", error);

        double roationSpeed = mTurnPID.calculate(error, 0);

        roationSpeed = MathUtil.clamp(roationSpeed, -DRIVETRAIN.MAX_TURN_SPEED, DRIVETRAIN.MAX_TURN_SPEED);

        roatationSpeed = applyDeadband(roatationSpeed, 0.05);

        SmartDashboard.putNumber("Commanded Roatation Speed", roationSpeed);


        mDrivetrain.Drive(xSpeed, ySpeed, roationSpeed, mFieldOriented);
    }

    @Override
    public void end(boolean interrupted) {
        mDrivetrain.Drive(0, 0, 0, false);
    }

}
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

    /** Seconds */
    private double prevTime;
    private double prevValue;

    private PIDController mTurnPID;

    private final SlewRateLimiter mForwardSpeedLimiter = new SlewRateLimiter(DRIVER.DRIVE_SLEW_RATE_LIMITER);
    private final SlewRateLimiter mStafeSpeedLimiter = new SlewRateLimiter(DRIVER.DRIVE_SLEW_RATE_LIMITER);
    private double mHeading = 0.0;

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

    private double GetCommandedHeading() {
        double curentInput = -Math.atan2(mDriver.getRightY() , mDriver.getRightX()) - Math.PI / 2;
        double magnitude = Math.sqrt(Math.pow(mDriver.getRightY(), 2) + Math.pow(mDriver.getRightX(), 2));
        if (magnitude > DRIVER.TURN_JOYSTICK_DEADBAND) {
            mHeading = curentInput;
            return curentInput;
        } else
            return mHeading;
    }

    private double GetTurnSpeed() {
        double magnitude = Math.sqrt(Math.pow(mDriver.getRightY(), 2) + Math.pow(mDriver.getRightX(), 2));
        if (magnitude > DRIVER.TURN_JOYSTICK_DEADBAND)
            return magnitude;
        else
            return DRIVER.TURN_JOYSTICK_DEADBAND;
    }

    public Drive(Drivetrain drivetrain, XboxController driver, boolean fieldOriented) {
        mDrivetrain = drivetrain;
        mFieldOriented = fieldOriented;
        mDriver = driver;

        prevTime = RobotController.getFPGATime() * 1e6;
        prevValue = mDrivetrain.GetHeading();

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
        double heading = GetCommandedHeading();
        double roatationSpeed = GetTurnSpeed();

        mFieldOriented = SmartDashboard.getBoolean("Field Orientated", mFieldOriented);

        SmartDashboard.putNumber("X", xSpeed);
        SmartDashboard.putNumber("Y", ySpeed);
        SmartDashboard.putNumber("Heading", heading);
        SmartDashboard.putNumber("Roatation Speed", roatationSpeed);

        double now = RobotController.getFPGATime() * 1e6;
        double delta = now - prevTime;

        double turnRateLimit = roatationSpeed * delta;
        double TargetHeading = prevValue + MathUtil.clamp(heading - prevValue, -turnRateLimit, turnRateLimit);

        
        double error = mDrivetrain.GetHeading() - heading;

        if (error > Math.PI) {
            error -= 2 * Math.PI;
        }
        if (error < -Math.PI) {
            error += 2 * Math.PI;
        }

        SmartDashboard.putNumber("Error", error);

        double roationSpeed = mTurnPID.calculate(error, 0);

        roationSpeed = MathUtil.clamp(roationSpeed, -DRIVER.MAX_ROTATION_VELOCITY, DRIVER.MAX_ROTATION_VELOCITY);

        SmartDashboard.putNumber("Commanded Roatation Speed", roationSpeed);


        mDrivetrain.Drive(xSpeed, ySpeed, roationSpeed, mFieldOriented);
    }

    @Override
    public void end(boolean interrupted) {
        mDrivetrain.Drive(0, 0, 0, false);
    }

}
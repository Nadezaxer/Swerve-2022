package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DRIVETRAIN;
import frc.robot.subsystems.Drivetrain;

public class Drive extends CommandBase {
    private Drivetrain mDrivetrain;
    private DoubleSupplier mXSpeed;
    private DoubleSupplier mYSpeed;
    private DoubleSupplier mCommandHedeading;
    private DoubleSupplier mTurnSpeed;
    private boolean mFieldOriented;

    /** Seconds */
    private double prevTime;
    private double prevValue;

    private PIDController mTurnPID;

    

    public Drive ( Drivetrain drivetrain, DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier commandedHeading, DoubleSupplier turnSpeed, boolean fieldOriented ) {
        mDrivetrain = drivetrain;
        mXSpeed = xSpeed;
        mYSpeed = ySpeed;
        mCommandHedeading = commandedHeading;
        mTurnSpeed = turnSpeed;
        mFieldOriented = fieldOriented;

        prevTime = RobotController.getFPGATime() * 1e6;
        prevValue = mDrivetrain.GetHeading();

        mTurnPID = new PIDController(DRIVETRAIN.HEADING_P_GAIN, DRIVETRAIN.HEADING_I_GAIN, DRIVETRAIN.HEADING_D_GAIN);

        addRequirements( mDrivetrain );

        SmartDashboard.putBoolean("Field Orientated", fieldOriented);
    }

    @Override
    public void execute() {
        double xSpeed = mXSpeed.getAsDouble();
        double ySpeed = mYSpeed.getAsDouble();
        double heading = mCommandHedeading.getAsDouble();
        double roatationSpeed = mCommandHedeading.getAsDouble();

        mFieldOriented = SmartDashboard.getBoolean("Field Orientated", mFieldOriented);

        SmartDashboard.putNumber("X", xSpeed);
        SmartDashboard.putNumber("Y", ySpeed);
        SmartDashboard.putNumber("Heading", heading);
        SmartDashboard.putNumber("Roatation Speed", roatationSpeed);

        double now = RobotController.getFPGATime() * 1e6;
        double delta = now - prevTime;
        
        double turnRateLimit = mTurnSpeed.getAsDouble() * delta;
        double TargetHeading = prevValue + MathUtil.clamp(mCommandHedeading.getAsDouble() - prevValue, -turnRateLimit, turnRateLimit);
        
        double roationSpeed = mTurnPID.calculate(mDrivetrain.GetHeading(), TargetHeading);

        mDrivetrain.Drive( xSpeed, ySpeed, roationSpeed, mFieldOriented );
    }

    @Override
    public void end( boolean interrupted ) {
        mDrivetrain.Drive( 0, 0, 0, false );
    }

}
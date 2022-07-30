package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class Drive extends CommandBase {
    private Drivetrain mDrivetrain;
    private DoubleSupplier mXSpeed;
    private DoubleSupplier mYSpeed;
    private DoubleSupplier mRoation;
    private boolean mFieldOriented;

    public Drive ( Drivetrain drivetrain, DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier rotation, boolean fieldOriented ) {
        mDrivetrain = drivetrain;
        mXSpeed = xSpeed;
        mYSpeed = ySpeed;
        mRoation = rotation;
        mFieldOriented = fieldOriented;
        addRequirements( mDrivetrain );

        SmartDashboard.putBoolean("Field Orientated", fieldOriented);
    }

    @Override
    public void execute() {
        double xSpeed = mXSpeed.getAsDouble();
        double ySpeed = mYSpeed.getAsDouble();
        double roation = mRoation.getAsDouble();

        mFieldOriented = SmartDashboard.getBoolean("Field Orientated", mFieldOriented);

        SmartDashboard.putNumber("X", xSpeed);
        SmartDashboard.putNumber("Y", ySpeed);
        SmartDashboard.putNumber("Rot", roation);
        mDrivetrain.Drive( xSpeed, ySpeed, roation, mFieldOriented );
    }

    @Override
    public void end( boolean interrupted ) {
        mDrivetrain.Drive( 0, 0, 0, false );
    }

}
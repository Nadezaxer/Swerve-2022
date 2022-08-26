package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class RestHeading extends CommandBase{
    private final Drivetrain mDrivetrain;
    
    public RestHeading(Drivetrain drivetrain) {
        mDrivetrain = drivetrain;
    }

    @Override
    public void initialize() {
        mDrivetrain.resetHeading();
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}

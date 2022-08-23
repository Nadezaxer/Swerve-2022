package frc.robot.commands;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DRIVETRAIN;
import frc.robot.Constants.PathFollowing;
import frc.robot.subsystems.Drivetrain;

public class FollowPath {

    public static Command GeneratePathFollowingCommand(PathPlannerTrajectory trajectory, Drivetrain drivetrain) {
        PIDController xController = new PIDController(PathFollowing.TRANSLATION_P_GAIN, PathFollowing.TRANSLATION_I_GAIN, PathFollowing.TRANSLATION_D_GAIN);
        PIDController yController = new PIDController(PathFollowing.TRANSLATION_P_GAIN, PathFollowing.TRANSLATION_I_GAIN, PathFollowing.TRANSLATION_D_GAIN);
        ProfiledPIDController thetaController = new ProfiledPIDController(DRIVETRAIN.HEADING_P_GAIN, DRIVETRAIN.HEADING_I_GAIN, DRIVETRAIN.HEADING_D_GAIN, 
            new TrapezoidProfile.Constraints(DRIVETRAIN.MAX_TURN_SPEED, DRIVETRAIN.MAX_TURN_ACCELERATION));
        return new PPSwerveControllerCommand(trajectory, drivetrain::GetPose, drivetrain.getKinematics(), xController, yController, thetaController, drivetrain::SetSwerveModuleStates, drivetrain);
    }

    public static Command GeneratePathFollowingCommand( String name, Drivetrain drivetrain) {
        return GeneratePathFollowingCommand(PathPlanner.loadPath(name, 0.5, 0.5), drivetrain);
    }
}

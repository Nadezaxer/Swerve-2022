package frc.robot.commands;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DRIVETRAIN;
import frc.robot.Constants.PathFollowing;
import frc.robot.subsystems.Drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class FollowPath extends CommandBase {
    private final Timer m_timer = new Timer();
    private final Drivetrain mDrivetrain;
    private final PathPlannerTrajectory m_trajectory;
    private final SwerveDriveKinematics m_kinematics;
    private final HolonomicDriveController m_controller;

    public FollowPath(PathPlannerTrajectory trajectory, Drivetrain drivetrain) {
        m_trajectory = trajectory;
        m_kinematics = drivetrain.getKinematics();
        m_controller = new HolonomicDriveController(
                new PIDController(PathFollowing.TRANSLATION_P_GAIN, PathFollowing.TRANSLATION_I_GAIN,
                        PathFollowing.TRANSLATION_D_GAIN),
                new PIDController(PathFollowing.TRANSLATION_P_GAIN, PathFollowing.TRANSLATION_I_GAIN,
                        PathFollowing.TRANSLATION_D_GAIN),
                new ProfiledPIDController(DRIVETRAIN.HEADING_P_GAIN, DRIVETRAIN.HEADING_I_GAIN,
                        DRIVETRAIN.HEADING_D_GAIN,
                        new TrapezoidProfile.Constraints(DRIVETRAIN.MAX_TURN_SPEED, DRIVETRAIN.MAX_TURN_ACCELERATION)));
        mDrivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    public FollowPath(String pathName, Drivetrain drivetrain) {
        m_trajectory = PathPlanner.loadPath(pathName, 0.5, 0.5);
        m_kinematics = drivetrain.getKinematics();
        m_controller = new HolonomicDriveController(
                new PIDController(PathFollowing.TRANSLATION_P_GAIN, PathFollowing.TRANSLATION_I_GAIN,
                        PathFollowing.TRANSLATION_D_GAIN),
                new PIDController(PathFollowing.TRANSLATION_P_GAIN, PathFollowing.TRANSLATION_I_GAIN,
                        PathFollowing.TRANSLATION_D_GAIN),
                new ProfiledPIDController(DRIVETRAIN.HEADING_P_GAIN, DRIVETRAIN.HEADING_I_GAIN,
                        DRIVETRAIN.HEADING_D_GAIN,
                        new TrapezoidProfile.Constraints(DRIVETRAIN.MAX_TURN_SPEED, DRIVETRAIN.MAX_TURN_ACCELERATION)));
        mDrivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        m_timer.reset();
        m_timer.start();
        mDrivetrain.setPose(m_trajectory.getInitialPose());
    }

    @Override
    public void execute() {
        double curTime = m_timer.get();
        PathPlannerState desiredState = (PathPlannerState) m_trajectory.sample(curTime);
        
        ChassisSpeeds targetChassisSpeeds = m_controller.calculate(mDrivetrain.GetPose(), desiredState, desiredState.holonomicRotation);
        var targetModuleStates = m_kinematics.toSwerveModuleStates(targetChassisSpeeds);
        
        mDrivetrain.SetSwerveModuleStates(targetModuleStates);

        SmartDashboard.putNumber("Auto Time", curTime);
        double[] chassisSpeeds = {targetChassisSpeeds.vxMetersPerSecond, targetChassisSpeeds.vyMetersPerSecond, targetChassisSpeeds.omegaRadiansPerSecond};
        SmartDashboard.putNumberArray("Target Chassis Speeds", chassisSpeeds);

    }

    @Override
    public void end(boolean interrupted) {
        m_timer.stop();
        mDrivetrain.Drive(0, 0, 0, false);
    }

    @Override
    public boolean isFinished() {
        return m_timer.hasElapsed(m_trajectory.getTotalTimeSeconds());
    }
}

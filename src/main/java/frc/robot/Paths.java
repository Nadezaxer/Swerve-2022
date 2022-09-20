package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

public class Paths {
    public static final PathPlannerTrajectory TestPath = PathPlanner.loadPath("TestPath", 0.5, 0.5);
    public static final PathPlannerTrajectory TestPath2 = PathPlanner.loadPath("TestPath2", 0.5, 0.5);
}

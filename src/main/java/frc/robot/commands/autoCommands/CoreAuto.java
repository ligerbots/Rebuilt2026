
package frc.robot.commands.autoCommands;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.FieldConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class CoreAuto extends AutoCommandInterface {

    protected Pose2d m_initPose;
    private CommandSwerveDrivetrain m_driveTrain;

    PathConstraints constraints = new PathConstraints(
            4.0, 2.0,
            Math.toRadians(540), Math.toRadians(720));

    public static CoreAuto createCoreAuto(CommandSwerveDrivetrain driveTrain, boolean isDepotSide) {
        String[] pathNames = {
                "Start Bump to Fuel Begin",
                "Fuel Begin to Fuel End",
                "Fuel End to Bump Finish",
                "Bump Finish to Climb A"
        };
        return new CoreAuto(pathNames, driveTrain, isDepotSide);
    }
    
    /** Creates a new CoreAuto. */
    private CoreAuto(String[] pathFiles, CommandSwerveDrivetrain driveTrain, boolean isDepotSide) {

        m_driveTrain = driveTrain;

        SmartDashboard.putBoolean("autoStatus/runningIntake", false);
        SmartDashboard.putBoolean("autoStatus/runningShooter", false);

        try {

            PathPlannerPath startPath = PathPlannerPath.fromPathFile(pathFiles[0]);
            if (isDepotSide) {
                startPath = startPath.mirrorPath();
            }
            m_initPose = startPath.getStartingHolonomicPose().get();

            for (String pathName : pathFiles) {
                PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
                if (isDepotSide) {
                    path = path.mirrorPath();
                }
                addCommands(m_driveTrain.followPath(path));
            }
        } catch (Exception e) {
            DriverStation.reportError("Unable to load PP path Test", true);
            m_initPose = new Pose2d();
        }
    }

    @Override
    public Pose2d getInitialPose() {
        return FieldConstants.flipPose(m_initPose);
    }
}

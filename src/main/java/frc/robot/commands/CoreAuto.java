
package frc.robot.commands;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.FieldConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class CoreAuto extends AutoCommandInterface {

    protected Pose2d m_initPose;
    private CommandSwerveDrivetrain m_driveTrain;

    PathConstraints constraints = new PathConstraints(
            4.0, 2.0,
            Math.toRadians(540), Math.toRadians(720));

    /** Creates a new CoreAuto. */
    public CoreAuto(String[] pathFiles, CommandSwerveDrivetrain driveTrain, boolean isDepotSide) {

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


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
    public CoreAuto(CommandSwerveDrivetrain driveTrain, boolean isDepotSide) {

        m_driveTrain = driveTrain;

        SmartDashboard.putBoolean("autoStatus/runningIntake", false);
        SmartDashboard.putBoolean("autoStatus/runningShooter", false);

        try {

            AutoPathSpec[] pathSpecs = {
                    new AutoPathSpec("Start Bump to Fuel Begin", false, false),
                    // new AutoPathSpec("Fuel Begin to Fuel End", true, false),
                    new AutoPathSpec("Fuel Begin to Fuel End With Events", true, false),
                    new AutoPathSpec("Fuel End to Bump Finish With Events", false, false),
                    new AutoPathSpec("Bump Finish to Climb A", false, true)
            };

            PathPlannerPath startPath = PathPlannerPath.fromPathFile(pathSpecs[0].pathName());
            if (isDepotSide) {
                startPath = startPath.mirrorPath();
            }
            m_initPose = startPath.getStartingHolonomicPose().get();

            for (AutoPathSpec pathSpec : pathSpecs) {
                addCommands(
                        buildActivePathCommand(pathSpec, isDepotSide)
                        );
            }
        } catch (Exception e) {
            DriverStation.reportError("Unable to load PP path Test", true);
            m_initPose = new Pose2d();
        }
    }

    private Command buildActivePathCommand(AutoPathSpec pathSpec, boolean isDepotSide)
            throws FileVersionException, IOException, ParseException {
        PathPlannerPath path = null;
        path = PathPlannerPath.fromPathFile(pathSpec.pathName());
        if (isDepotSide) {
            path = path.mirrorPath();
        }
        ParallelCommandGroup activePath = new ParallelCommandGroup(m_driveTrain.followPath(path));
        activePath.addCommands(new InstantCommand(() -> SmartDashboard.putBoolean("autoStatus/runningIntake", false)));

        // if (pathSpec.runIntake()) {
        //     activePath.addCommands(new PrintCommand("Running Intake on path: " + pathSpec.pathName()));
        // } else {
        //     activePath.addCommands(new PrintCommand("Stopping Intake for path: " + pathSpec.pathName()));
        // }

        // activePath.addCommands(new InstantCommand(() -> SmartDashboard.putBoolean("autoStatus/runningIntake", pathSpec.runIntake())));

        if (pathSpec.runShooter()) {
            // activePath.addCommands(new RunShooterCommand(...));
            activePath.addCommands(new PrintCommand("Running Shooter on path: " + pathSpec.pathName()));
            // activePath.addCommands(new InstantCommand(() -> set_shooterRunning(true)));
        } else {
            activePath.addCommands(new PrintCommand("Stopping Shooter for path: " + pathSpec.pathName()));
        }

        activePath.addCommands(new InstantCommand(() -> SmartDashboard.putBoolean("autoStatus/runningShooter", pathSpec.runShooter())));

        return activePath;
    }

    @Override
    public Pose2d getInitialPose() {
        return FieldConstants.flipPose(m_initPose);
    }
}

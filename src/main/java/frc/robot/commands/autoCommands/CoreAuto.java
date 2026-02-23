
package frc.robot.commands.autoCommands;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.InternalButton;
import frc.robot.FieldConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class CoreAuto extends AutoCommandInterface {

    protected Pose2d m_initPose;
    private CommandSwerveDrivetrain m_driveTrain;
    // private PathPlannerAuto m_pathPlannerAuto;

    PathConstraints constraints = new PathConstraints(
            4.0, 2.0,
            Math.toRadians(540), Math.toRadians(720));

    public static CoreAuto getInstance(String[] pathFiles, CommandSwerveDrivetrain driveTrain, boolean isOutpostSide, double preloadShootTime,
            InternalButton virtualShootButton) {
        return new CoreAuto(pathFiles, driveTrain, isOutpostSide, preloadShootTime, virtualShootButton);
    }
    
    /** Creates a new CoreAuto. 
     * @param m_shooter 
     * @param m_turret 
     */
    private CoreAuto(String[] pathFiles, CommandSwerveDrivetrain driveTrain, boolean isOutpostSide,
            double preloadShootTime, InternalButton virtualShootButton) {

        m_driveTrain = driveTrain;

        SmartDashboard.putBoolean("autoStatus/runningIntake", false);
        SmartDashboard.putBoolean("autoStatus/runningShooter", false);

        try {

            PathPlannerPath startPath = PathPlannerPath.fromPathFile(pathFiles[0]);
            if (isOutpostSide) {
                startPath = startPath.mirrorPath();
            }
            m_initPose = startPath.getStartingHolonomicPose().get();

            // Shoot preloaded balls for N seconds before driving
            if (preloadShootTime > 0) {

                addCommands(new InstantCommand(() -> virtualShootButton.setPressed(true))
                        .alongWith(new WaitCommand(preloadShootTime),
                                new InstantCommand(
                                        () -> SmartDashboard.putBoolean("autoStatus/runningShooter", true))));

                addCommands(new InstantCommand(() -> virtualShootButton.setPressed(false)),
                        new InstantCommand(
                                () -> SmartDashboard.putBoolean("autoStatus/runningShooter", false)));

            }

            for (String pathName : pathFiles) {
                PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
                if (isOutpostSide) {
                    path = path.mirrorPath();
                }
                addCommands(m_driveTrain.followPath(path));
            }
            // Clmber code goes here, but we don't have a climber yet so we'll leave it out for now
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

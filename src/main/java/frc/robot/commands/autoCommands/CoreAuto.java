
package frc.robot.commands.autoCommands;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.FieldConstants;
import frc.robot.commands.Shoot;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShotType;
import frc.robot.subsystems.shooter.ShooterFeeder;
import frc.robot.subsystems.shooter.Turret;

public class CoreAuto extends AutoCommandInterface {

    protected Pose2d m_initPose;
    private CommandSwerveDrivetrain m_driveTrain;
    // private PathPlannerAuto m_pathPlannerAuto;

    PathConstraints constraints = new PathConstraints(
            4.0, 2.0,
            Math.toRadians(540), Math.toRadians(720));

    public static CoreAuto getInstance(String[] pathFiles, CommandSwerveDrivetrain driveTrain, boolean isOutpostSide, double preloadShootTime, 
    Shooter m_shooter, Turret m_turret, ShooterFeeder m_shooterFeeder, Hopper m_hopper) {
        return new CoreAuto(pathFiles, driveTrain, isOutpostSide, preloadShootTime, m_shooter, m_turret, m_shooterFeeder, m_hopper);
    }
    
    /** Creates a new CoreAuto. 
     * @param m_shooter 
     * @param m_turret */
    private CoreAuto(String[] pathFiles, CommandSwerveDrivetrain driveTrain, boolean isOutpostSide, double preloadShootTime, 
    Shooter m_shooter, Turret m_turret, ShooterFeeder m_shooterFeeder, Hopper m_hopper) {

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
            if(preloadShootTime > 0) {
                addCommands(
                    new Shoot(m_shooter, m_turret, m_shooterFeeder, driveTrain::getPose, ShotType.HUB).alongWith(m_hopper.pulseCommand())
                    .withTimeout(preloadShootTime)
                    );
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

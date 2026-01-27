
package frc.robot.commands;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.FieldConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class CtreTestAuto extends AutoCommandInterface {

    protected Pose2d m_initPose;
    private CommandSwerveDrivetrain m_driveTrain;
    
    PathConstraints constraints =  new PathConstraints(
    4.0, 2.0,
    Math.toRadians(540), Math.toRadians(720));


    /** Creates a new TestCtreAuto. */
    public CtreTestAuto(CommandSwerveDrivetrain driveTrain, boolean isProcessorSide) {

        m_driveTrain = driveTrain;

        try {
            PathPlannerPath startPath = PathPlannerPath.fromPathFile("Start2 to ReefH");
            
            m_initPose = startPath.getStartingHolonomicPose().get();
            
            addCommands(m_driveTrain.followPath(startPath));
            
            
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

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Some points:
//
// When adding a vision measurement to a WPILib PoseEstimator, the code will
// throw out existing measurements which are newer, so it helps to add them in
// time order, especially across multiple cameras.

package frc.robot.subsystems;

import java.io.UncheckedIOException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.Robot.RobotType;

public class AprilTagVision {
    static final AprilTagFields APRILTAG_FIELD = AprilTagFields.k2026RebuiltWelded;

    // static final String CUSTOM_FIELD = "2025-reefscape-andymark_custom.json"; // old

    // Use the multitag pose estimator
    static final PoseStrategy POSE_STRATEGY = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;
    
    // Plot vision solutions
    static final boolean PLOT_VISIBLE_TAGS = true;
    static final boolean PLOT_POSE_SOLUTIONS = true;
    // static final boolean PLOT_ALTERNATE_POSES = false;

    // throw away single tags with high ambiguity
    static final double SINGLE_TAG_AMBIGUITY_THRESHOLD = 0.4;

    // Base standard deviations for vision results
    static final Matrix<N3, N1> SINGLE_TAG_BASE_STDDEV = VecBuilder.fill(0.9, 0.9, 0.9);
    static final Matrix<N3, N1> MULTI_TAG_BASE_STDDEV = VecBuilder.fill(0.45, 0.45, 0.45);

    private class Camera {
        PhotonCamera photonCamera;
        Transform3d robotToCam;
        PhotonPoseEstimator poseEstimator;

        private Camera(String name, Transform3d robotToCam) {
            this.robotToCam = robotToCam;
            photonCamera = new PhotonCamera(name);

            // In Case of Emergencies!!
            // pCamera.setVersionCheckEnabled(false);

            // Use the standard PoseStrategy
            // Don't need to set a strategy, since we don't use update()
            poseEstimator = new PhotonPoseEstimator(m_aprilTagFieldLayout, robotToCam);

            // set the driver mode to false
            photonCamera.setDriverMode(false);
        }
    }

    private Camera[] m_cameras;

    // Used to hold results from the cameras. Sorts into time increasing order.
    private class CameraMeasurement implements Comparable<CameraMeasurement> {
        public Camera camera;
        public PhotonPipelineResult pipelineResult;
        public CameraMeasurement(Camera c, PhotonPipelineResult pRes) {
            camera = c;
            pipelineResult = pRes;
        }
        @Override
        public int compareTo(CameraMeasurement other) {
            double tMe = pipelineResult.getTimestampSeconds();
            double tOther = other.pipelineResult.getTimestampSeconds();
            if (tMe < tOther) return -1;
            if (tOther < tMe) return 1;
            return 0;
        }
    };

    private AprilTagFieldLayout m_aprilTagFieldLayout;
    private final Field2d m_field;

    // Simulation support
    private VisionSystemSim m_visionSim;
    private boolean m_isSimulation;

    public AprilTagVision(RobotType robotType, Field2d field) {
        try {
            m_aprilTagFieldLayout = AprilTagFieldLayout.loadField(APRILTAG_FIELD);
            SmartDashboard.putString("aprilTagVision/field", APRILTAG_FIELD.toString());

            // String fieldpath = Filesystem.getDeployDirectory().getPath() + "/" + CUSTOM_FIELD;
            // m_aprilTagFieldLayout = new AprilTagFieldLayout(fieldpath);
            // SmartDashboard.putString("aprilTagVision/field", CUSTOM_FIELD);
        } catch (UncheckedIOException e) {
            System.out.println("Unable to load AprilTag layout " + e.getMessage());
            m_aprilTagFieldLayout = null;
        } 
        // catch (IOException e) {
        //     System.out.println("Unable to load AprilTag layout " + e.getMessage());
        //     m_aprilTagFieldLayout = null;
        // }

        // used to publish found tags, etc
        m_field = field;

        // initialize cameras
        if (robotType == RobotType.TESTBOT) {
            // Test Bot
            m_cameras = new Camera[] {
                    new Camera("ArducamFrontRight", new Transform3d(
                            new Translation3d(Units.inchesToMeters(9.32), Units.inchesToMeters(-9.5),
                                    Units.inchesToMeters(10.53)),
                            new Rotation3d(0.0, Math.toRadians(-10), 0)
                                    .rotateBy(new Rotation3d(0, 0, Math.toRadians(12.5)))))
            };
        } 
        else
        {
            m_cameras = new Camera[] {
                    new Camera("ArducamBackRight", new Transform3d(
                            new Translation3d(Units.inchesToMeters(-8.89), Units.inchesToMeters(-12.07),
                                    Units.inchesToMeters(15.08)),
                            new Rotation3d(0.0, Math.toRadians(-10), 0)
                                    .rotateBy(new Rotation3d(0, 0, Math.toRadians(180.0-20.0))))),
                    new Camera("ArducamBackLeft", new Transform3d(
                            new Translation3d(Units.inchesToMeters(-6.56), Units.inchesToMeters(12.87),
                                    Units.inchesToMeters(17.27)),
                            new Rotation3d(0.0, Math.toRadians(-10), 0)
                                    .rotateBy(new Rotation3d(0, 0, Math.toRadians(90.0))))),
                    new Camera("ArducamRight", new Transform3d(
                            new Translation3d(Units.inchesToMeters(-0.187), Units.inchesToMeters(-13.35),
                                    Units.inchesToMeters(14.84)),
                            new Rotation3d(0.0, Math.toRadians(-10), 0)
                                    .rotateBy(new Rotation3d(0, 0, Math.toRadians(-90.0)))))
            };
        }
    
        m_isSimulation = RobotBase.isSimulation();
        if (Constants.SIMULATION_SUPPORT && m_isSimulation) {
            // initialize a simulated camera. Must be done after creating the tag layout
            initializeSimulation();
        }
    }

    public void updateSimulation(CommandSwerveDrivetrain swerve) {    
        m_visionSim.update(swerve.getPose());
    }
    
    // Update all Pose estimates with the vision measurements
    public void addVisionMeasurements(CommandSwerveDrivetrain swerve) {
        // Cannot do anything if there is no field layout
        if (m_aprilTagFieldLayout == null)
            return;

        // Some lists for later plotting
        // Accumulate the results, and then plot them at the end
        ArrayList<Pose2d> visibleTags = new ArrayList<Pose2d>();
        ArrayList<Pose2d> globalMeasurements = new ArrayList<Pose2d>();

        try {
            // First collect all the camera measurements into a list
            ArrayList<CameraMeasurement> camFrames = new ArrayList<CameraMeasurement>();
            for (Camera cam : m_cameras) {
                boolean isConnected = cam.photonCamera.isConnected();
                SmartDashboard.putBoolean("aprilTagVision/" + cam.photonCamera.getName(), isConnected);
                if (!isConnected)
                    continue;

                for (PhotonPipelineResult pipeRes : cam.photonCamera.getAllUnreadResults()) {
                    camFrames.add(new CameraMeasurement(cam, pipeRes));
                }
            }

            // Sort the frames in time order
            Collections.sort(camFrames);

            Pose2d currentPose = swerve.getState().Pose;

            // Work through all the available frames, in time order, and use any measurements
            for (CameraMeasurement frame : camFrames) {
                // loop over the individual tags in the frame and 
                // add them to a list so we can plot them
                if (PLOT_VISIBLE_TAGS) {
                    for (PhotonTrackedTarget target : frame.pipelineResult.targets) {
                        int targetFiducialId = target.getFiducialId();
                        if (targetFiducialId <= 0)
                            continue;
                        Optional<Pose3d> targetPosition = m_aprilTagFieldLayout.getTagPose(targetFiducialId);
                        if (targetPosition.isEmpty())
                            continue;

                        visibleTags.add(targetPosition.get().toPose2d());
                    }
                }

                // find the best global pose estimate, and update the odometry
                try {
                    Optional<EstimatedRobotPose> estPose = frame.camera.poseEstimator.estimateCoprocMultiTagPose(frame.pipelineResult);
                    // if we got not estimate, try single tag method
                    if (estPose.isEmpty()) {
                        estPose = closestToReferenceHeading(frame, currentPose.getRotation().getRadians());
                        // System.out.println("single tag:" + estPose.isEmpty());
                    }
                    if (estPose.isEmpty())
                        continue;

                    EstimatedRobotPose poseEstimate = estPose.get();
                    Optional<Matrix<N3, N1>> estStdDev = estimateStdDev(poseEstimate);
                    if (estStdDev.isPresent()) {
                        // Everything succeeded. Update the main poseEstimator with the vision result
                        // Make sure to use the timestamp of this result
                        Pose2d pose = poseEstimate.estimatedPose.toPose2d();

                        // For simulation, the robot just drifts across the field,
                        //  because vision seems to be slightly biased
                        // TODO: remove the test we get a physics-based robot sim
                        if (!m_isSimulation)
                            swerve.addVisionMeasurement(pose, poseEstimate.timestampSeconds, estStdDev.get());
                            
                        globalMeasurements.add(pose);
                    }
                } catch (Exception e) {
                    // bad! log this and keep going
                    DriverStation.reportError("Exception running PhotonPoseEstimator", e.getStackTrace());
                }
            }
        } catch (Exception e) {
            DriverStation.reportError("Error updating odometry from AprilTags " + e.getLocalizedMessage(), false);
        }

        if (PLOT_VISIBLE_TAGS) {
            plotPoses(m_field, "visibleTags", visibleTags);
        }
        if (PLOT_POSE_SOLUTIONS) {
            plotPoses(m_field, "visionPoses", globalMeasurements);
        }
    }

    // get the pose for a tag.
    // will return null if the tag is not in the field map (eg -1)
    public Optional<Pose2d> getTagPose(int tagId) {
        // optional in case no target is found
        Optional<Pose3d> tagPose = m_aprilTagFieldLayout.getTagPose(tagId);
        if (tagPose.isEmpty()) {
            return Optional.empty(); // returns an empty optional
        }
        return Optional.of(tagPose.get().toPose2d());
    }

    // Calculates "confidence" in the pose estimate
    // This algorithm is a heuristic that creates dynamic standard deviations based
    // on number of tags, estimation strategy, and distance from the tags.
    private Optional<Matrix<N3, N1>> estimateStdDev(EstimatedRobotPose poseEst) {
        int numTags = poseEst.targetsUsed.size();
        // Should not happen, but protect against divide by zero
        if (numTags == 0)
            return Optional.empty();
        
        // discard if not on the field
        double x = poseEst.estimatedPose.getX();
        double y = poseEst.estimatedPose.getY();
        if (x < 0.0 || x > FieldConstants.FIELD_LENGTH || y < 0.0 || y > FieldConstants.FIELD_WIDTH)
            return Optional.empty();

        boolean usedMultitag = poseEst.strategy == PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR
                || poseEst.strategy == PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;
        
        // if there are >1 tag and we did not use MultiTag, BAD
        if (numTags > 1 && !usedMultitag)
            return Optional.empty();

        // Pose present. Start running Heuristics

        // Find the average distance for the tags used
        double avgDist = 0;
        for (PhotonTrackedTarget tgt : poseEst.targetsUsed) {
            avgDist += tgt.getBestCameraToTarget().getTranslation().getNorm();;
        }
        avgDist /= numTags;

        // Single tags further away than 4 meter (~13 ft) are useless
        if (numTags == 1 && avgDist > 4.0) 
            return Optional.empty();

        // Starting estimate = multitag or not
        // Matrix<N3, N1> estStdDev;
        // if (usedMultitag)
        //     estStdDev = MULTI_TAG_BASE_STDDEV;
        // else
        //     estStdDev = SINGLE_TAG_BASE_STDDEV;
        Matrix<N3, N1> estStdDev = SINGLE_TAG_BASE_STDDEV;

        // Increase std devs based on (average) distance
        // This is taken from YAGSL vision example.
        // TODO figure out why
        // double scaleFactorOld = 1.0 + avgDist * avgDist / 30.0;

        // Mechanical Advantage version
        // Includes a downscale for more tags, so start from a single Matrix
        double scaleFactor = Math.pow(avgDist, 1.2) / Math.pow(numTags, 2.0);

        estStdDev = estStdDev.times(scaleFactor);

        return Optional.of(estStdDev);
    }

    // Implement a Closest To Reference *Heading* strategy for single tag results
    private Optional<EstimatedRobotPose> closestToReferenceHeading(CameraMeasurement frame, final double refHeadingRad)
    {
        Camera cam = frame.camera;
        double timestamp = frame.pipelineResult.getTimestampSeconds();

        PhotonTrackedTarget bestTarget = null;
        Pose3d bestPose = null;
        double bestDiff = 1e12;

        for (PhotonTrackedTarget target : frame.pipelineResult.targets) {
            Optional<Pose3d> targetPosition = m_aprilTagFieldLayout.getTagPose(target.fiducialId);
            if (targetPosition.isEmpty())
                continue;

            // check ambiguity ratio
            double ambiguity = target.getPoseAmbiguity();
            if (ambiguity > SINGLE_TAG_AMBIGUITY_THRESHOLD)
                continue;

            // Check the 2 possible vision poses
            Pose3d pose1 = targetPosition.get()
                    .transformBy(target.getBestCameraToTarget().inverse())
                    .transformBy(cam.robotToCam.inverse());

            // Use MathUtil.angleModulus() to map the difference to -PI --> PI
            double diff1 = Math.abs(MathUtil.angleModulus(pose1.toPose2d().getRotation().getRadians() - refHeadingRad));

            if (diff1 < bestDiff) {
                bestDiff = diff1;
                bestTarget = target;
                bestPose = pose1;
            }

            // also need to check the altPose
            Pose3d pose2 = targetPosition.get()
                    .transformBy(target.getAlternateCameraToTarget().inverse())
                    .transformBy(cam.robotToCam.inverse());
            double diff2 = Math.abs(MathUtil.angleModulus(pose2.toPose2d().getRotation().getRadians() - refHeadingRad));

            if (diff2 < bestDiff) {
                bestDiff = diff2;
                bestTarget = target;
                bestPose = pose2;
            }

            // System.out.println("refHeading " + Math.toDegrees(MathUtil.angleModulus(refHeadingRad)));
            // System.out.println("pose1 " + pose1.toPose2d());
            // System.out.println("pose2 " + pose2.toPose2d());
            // System.out.println("single tag: " + ambiguity + " d1 =" + Math.toDegrees(diff1) + " d2=" + Math.toDegrees(diff2));
        }

        // return the closest pose, if there is one
        if (bestPose == null)
            return Optional.empty();

        // Note: PoseStrategy does not have value for this strategy, so just use Closes
        return Optional.of(new EstimatedRobotPose(bestPose, timestamp, List.of(bestTarget),
                PoseStrategy.CLOSEST_TO_REFERENCE_POSE));
    }

    // private static AprilTag constructTag(int id, double x, double y, double z,
    // double angle) {
    // return new AprilTag(id, new Pose3d(x, y, z, new Rotation3d(0, 0,
    // Math.toRadians(angle))));
    // }

    // // add a new tag relative to another tag. Assume the orientation is the same
    // private static AprilTag constructTagRelative(int id, Pose3d basePose, double
    // x, double y, double z) {
    // return new AprilTag(id, new Pose3d(basePose.getX() + x, basePose.getY() + y,
    // basePose.getZ() + z, basePose.getRotation()));
    // }

    private void initializeSimulation() {
        m_visionSim = new VisionSystemSim("AprilTag");

        // roughly our Arducam camera
        SimCameraProperties prop = new SimCameraProperties();
        prop.setCalibration(800, 600, Rotation2d.fromDegrees(90.0));
        prop.setFPS(60);
        prop.setAvgLatencyMs(20.0);
        prop.setLatencyStdDevMs(5.0);

        // Note: NetworkTables does not update the timestamp of an entry if the value does not change.
        // The timestamp is used by PVLib to know if there is a new frame, so in a simulation
        // with no uncertainty, it thinks that it is not detecting a tag if the robot is static.
        // So, always have a little bit of uncertainty.
        prop.setCalibError(0.5, 0.3);

        for (Camera c : m_cameras) {
            PhotonCameraSim camSim = new PhotonCameraSim(c.photonCamera, prop);
            // open web page with a simulate camera image. 
            camSim.enableDrawWireframe(true);
            camSim.setMaxSightRange(Units.feetToMeters(15.0));
            m_visionSim.addCamera(camSim, c.robotToCam);
        }

        m_visionSim.addAprilTags(m_aprilTagFieldLayout);
    }

    // --- Routines to plot the vision solutions on a Field2d ---------

    private void plotPoses(Field2d field, String tagName, List<Pose2d> poses) {
        if (field == null)
            return;
        if (poses == null || poses.size() == 0)
            field.getObject(tagName).setPoses();
        else
            field.getObject(tagName).setPoses(poses);
    }
}
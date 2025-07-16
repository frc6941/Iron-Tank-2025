package frc.robot;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class SuperstructureVisualizer {
        //todo: change the mechenism location to fit the real robot
    private static SuperstructureVisualizer instance;

    // Conversion helper
    private static double mmToM(double mm) {
        return mm / 1000.0;
    }

    // Intake constants
    private static final Translation3d INTAKE_PIVOT_START = new Translation3d(
            mmToM(280), mmToM(-181), mmToM(250));
    private static final Translation3d INTAKE_PIVOT_END = new Translation3d(
            mmToM(280), mmToM(181), mmToM(250));
    private static final Translation3d INTAKE_CENTER = INTAKE_PIVOT_START.interpolate(INTAKE_PIVOT_END, 0.5);
    private static final double INTAKE_LENGTH = mmToM(452);

    // Visualization components
    private final LoggedMechanism2d intakeMechanism;
    private final LoggedMechanismLigament2d intakeArm;

    // Current state tracking
    private double currentIntakeAngleDeg = 0.0;

    // Coral diameter in meters
    private static final double CORAL_DIAMETER = mmToM(100); // Adjust this value based on actual coral size


    public static SuperstructureVisualizer getInstance() {
        if (instance == null) {
            instance = new SuperstructureVisualizer();
        }
        return instance;
    }

    public SuperstructureVisualizer() {
        // Intake mechanism setup
        intakeMechanism = new LoggedMechanism2d(
                0,
                0,
                new Color8Bit(Color.kWhite));

        LoggedMechanismRoot2d intakeRoot = intakeMechanism.getRoot(
                "IntakePivot",
                0.25,
                INTAKE_CENTER.getZ());

        intakeArm = new LoggedMechanismLigament2d(
                "intakeArm",
                INTAKE_LENGTH,
                90,
                8,
                new Color8Bit(Color.kRed));

        intakeRoot.append(intakeArm);
    }

    // Full update
    public void update(double elevatorHeight, double intakeAngleRad, double endEffectorAngleRad) {
        this.currentIntakeAngleDeg = intakeAngleRad;
        updateVisuals();
    }

    // Intake-only update
    public void updateIntake(double intakeAngleDeg) {
        this.currentIntakeAngleDeg = intakeAngleDeg;
        updateVisuals();
    }

    /**
     * Logs the Coral pose 3D if coral is detected in the intake
     */
    private void logCoralPose3D() {
        // Get the GamepieceTracker instance

        /*
        // Check if coral is detected in the intake
        if (GamepieceTracker.getInstance().isIntakeHasCoral()) {
        Pose3d robotPose = new Pose3d(Swerve.getInstance().getLocalizer().getCoarseFieldPose(Timer.getFPGATimestamp()));
            // Calculate the position of the coral at the middle of the intake arm
            // The coral is positioned at the end of the intake arm
            double intakeAngleRad = Math.toRadians(currentIntakeAngleDeg - 90);
            
            // Create a rotation matrix for the intake angle
            Rotation3d intakeRotation = new Rotation3d(0, intakeAngleRad, 0);
            
            // Calculate the position of the coral at the end of the intake arm
            Pose3d coralPosition = robotPose.transformBy(new Transform3d(INTAKE_CENTER.plus(new Translation3d(INTAKE_LENGTH/2,0,0).rotateBy(intakeRotation)), intakeRotation));
            
            // Log the Coral pose 3D
            Logger.recordOutput("Superstructure/Coral/InakeCoral", 
                    coralPosition);
        } else {
            // If no coral is detected, log an empty pose
            Logger.recordOutput("Superstructure/Coral/InakeCoral", new Pose3d());
        }
        if (GamepieceTracker.getInstance().isEndeffectorHasCoral()) {
            Pose3d robotPose = new Pose3d(Swerve.getInstance().getLocalizer().getCoarseFieldPose(Timer.getFPGATimestamp()));
            
            // Calculate the position of the coral at the middle of the end effector arm coral
            double endEffectorAngleRad = Math.toRadians(-currentEndEffectorAngleDeg-180);
            
            // Create a rotation matrix for the end effector angle
            Rotation3d endEffectorRotation = new Rotation3d(0, endEffectorAngleRad, 0);

            Translation3d endEffectorPosition = END_EFFECTOR_CENTER.plus(new Translation3d(0, 0, currentElevatorHeight+STAGE3_LENGTH));
            
            // Calculate the position of the coral at the middle of the end effector arm coral
            Pose3d coralPosition = robotPose.transformBy(new Transform3d(
                endEffectorPosition.plus(new Translation3d(-END_EFFECTOR_LENGTH_CORAL, 0, END_EFFECTOR_MOUNT_ARM_LENGTH).rotateBy(endEffectorRotation)),
                endEffectorRotation));
            
            Logger.recordOutput("Superstructure/Coral/EECoral", coralPosition);
        } else {
            Logger.recordOutput("Superstructure/Coral/EECoral", new Pose3d());
        }
        if (GamepieceTracker.getInstance().isEndeffectorHasAlgae()) {
                Pose3d robotPose = new Pose3d(Swerve.getInstance().getLocalizer().getCoarseFieldPose(Timer.getFPGATimestamp()));
                
                // Calculate the position of the coral at the middle of the end effector arm coral
                double endEffectorAngleRad = Math.toRadians(-currentEndEffectorAngleDeg-180);
                
                // Create a rotation matrix for the end effector angle
                Rotation3d endEffectorRotation = new Rotation3d(0, endEffectorAngleRad, 0);
    
                Translation3d endEffectorPosition = END_EFFECTOR_CENTER.plus(new Translation3d(0, 0, currentElevatorHeight+STAGE3_LENGTH));
                
                // Calculate the position of the coral at the middle of the end effector arm coral
                Pose3d coralPosition = robotPose.transformBy(new Transform3d(
                    endEffectorPosition.plus(new Translation3d(END_EFFECTOR_LENGTH_ALGAE, 0, END_EFFECTOR_MOUNT_ARM_LENGTH).rotateBy(endEffectorRotation)),
                    endEffectorRotation));
                
                Logger.recordOutput("Superstructure/Coral/EEAlgae", coralPosition);
            } else {
                Logger.recordOutput("Superstructure/Coral/EEAlgae", new Pose3d());
            }*/
    }

    private void updateVisuals() {
        // Update intake components
        intakeArm.setAngle(Rotation2d.fromRadians(Math.toRadians(-currentIntakeAngleDeg + 90)));
        
        // Log Coral pose 3D
        logCoralPose3D();

        // Log 2D mechanisms
        Logger.recordOutput("Superstructure/Intake/Mechanism2d", intakeMechanism);
    }
}
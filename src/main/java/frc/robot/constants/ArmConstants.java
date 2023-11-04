package frc.robot.constants;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class ArmConstants {
    public static final int elevatorMotorId = 19;
    public static final int elbowMotorId = 10;
    public static final int wristMotorId = 11;
    public static final int intakeMotorOneId = 12;
    public static final int intakeMotorTwoId = 13;

    public static final int topBreakerId = 0;
    public static final int bottomBreakerId = 1;

    public static final int topLinebreakerId = 0;
    public static final int bottomLinebreakerId = 1;

    public static final boolean elevatorMotorInverted = false;
    public static final boolean elbowMotorInverted = true;
    public static final boolean intakeMotorInverted = false;

    // Need very small tune
    public static double elevatorP = 108.0;
    public static final double elevatorI = 0.0;
    public static final double elevatorD = 0.0;

    // Tune:
    public static double elbowP = 5.1;
    public static final double elbowI = 0.0;
    public static final double elbowD = 0.0;

    // Maybe change:
    public static final ArmFeedforward elbowFF = new ArmFeedforward(0.13, 0.50, 0.00);

    // Tune:
    public static double wristP = 3.1;
    public static final double wristI = 0.0;
    public static final double wristD = 0.0;

    public static double elevatorGoal = 0.0;
    public static double elbowGoal = 0.0;
    public static double wristGoal = 0.0;

    // Maybe change:
    public static final ArmFeedforward wristFF = new ArmFeedforward(0.13, 0.27, 0.00);

    public static final double elevatorMinHeight = 0.04;
    public static final double elevatorMaxHeight = 0.947;

    public static final double elbowMaxRotation = Math.toRadians(140.0);
    public static final double elbowMinRotation = -(1.0 / 2.0) * Math.PI;

    public static final double wristMaxRotation = 15.0;
    public static final double wristMinRotation = -160.0;

    // Multipliers for unit conversion and stuff
    // These values obtained from tuning
    public static final double elevatorEncoderVelocityConversionFactor = (0.003010870139 * 2.4) / 60.0; // This should turn revs/min to meters/sec
    public static final double elevatorEncoderPositionConversionFactor = 1.0; // This should turn revs to meters

    // Could need small tuning:
    public static final double elbowEncoderPosOffset = -1.4;
    public static final double wristEncoderPosOffset = 0.15;

    public static final double elevatorZeroingVoltage = -1.5;
}
package frc.robot.subsystems;

//import com.revrobotics.CANSparkMaxLowLevel;
//import com.revrobotics.CANSparkMax;
//import com.revrobotics.RelativeEncoder;
//import com.revrobotics.SparkMaxAbsoluteEncoder;
//import edu.wpi.first.math.controller.ProfiledPIDController;
//import edu.wpi.first.math.geometry.Rotation2d;
//import edu.wpi.first.math.trajectory.TrapezoidProfile;
//import edu.wpi.first.networktables.BooleanPublisher;
//import edu.wpi.first.networktables.DoubleEntry;
//import edu.wpi.first.networktables.DoublePublisher;
//import edu.wpi.first.networktables.NetworkTableInstance;
//import edu.wpi.first.wpilibj.DigitalInput;
//import edu.wpi.first.wpilibj.XboxController;
//import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.constants.ArmConstants;
//import frc.robot.constants.CommandValues;
//
//public class PickAndPlaceSubsystem extends SubsystemBase {
//
//    private CANSparkMax elevatorMotor = new CANSparkMax(ArmConstants.elevatorMotorId, CANSparkMaxLowLevel.MotorType.kBrushless);
//    private CANSparkMax elbowMotor = new CANSparkMax(ArmConstants.elbowMotorId, CANSparkMaxLowLevel.MotorType.kBrushless);
//    private CANSparkMax wristMotor = new CANSparkMax(ArmConstants.wristMotorId, CANSparkMaxLowLevel.MotorType.kBrushless);
//    private CANSparkMax intakeOneMotor = new CANSparkMax(ArmConstants.intakeMotorOneId, CANSparkMaxLowLevel.MotorType.kBrushless);
//    private CANSparkMax intakeTwoMotor = new CANSparkMax(ArmConstants.intakeMotorTwoId, CANSparkMaxLowLevel.MotorType.kBrushless);
//
//    private RelativeEncoder elevatorEncoder = elevatorMotor.getEncoder();
//    private SparkMaxAbsoluteEncoder elbowEncoder = elbowMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
//    private SparkMaxAbsoluteEncoder wristEncoder = wristMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
//
//    private DigitalInput forwardLimit = new DigitalInput(ArmConstants.topBreakerId);
//    private DigitalInput reverseLimit = new DigitalInput(ArmConstants.bottomBreakerId);
//
//    private boolean elevatorZeroed = false;
//
//
//    private double intakeOnesVoltage = 0.0;
//    private double intakeTwosVoltage = 0.0;
//
//    private double wristVoltage = 0.0;
//    private double elbowVoltage = 0.0;
//    private double elevatorVoltage = 0.0;
//
//    //TELEMETRY
//
//    public static class Telemetry{
//        public static DoublePublisher elbowPosition = NetworkTableInstance.getDefault().getTable("Arm").getDoubleTopic("ElbowPosition").publish();
//        public static DoublePublisher elbowVelocity = NetworkTableInstance.getDefault().getTable("Arm").getDoubleTopic("ElbowVelocity").publish();
//        public static DoublePublisher elevatorPosition = NetworkTableInstance.getDefault().getTable("Arm").getDoubleTopic("ElevatorPosition").publish();
//        public static DoublePublisher elevatorVelocity = NetworkTableInstance.getDefault().getTable("Arm").getDoubleTopic("ElevatorVelocity").publish();
//
//        public static DoublePublisher elevatorVoltage = NetworkTableInstance.getDefault().getTable("Arm").getDoubleTopic("ElevatorVoltage").publish();
//        public static BooleanPublisher bottomHit = NetworkTableInstance.getDefault().getTable("Arm").getBooleanTopic("BottomHit").publish();
//        public static BooleanPublisher topHit = NetworkTableInstance.getDefault().getTable("Arm").getBooleanTopic("TopHit").publish();
//        public static DoublePublisher wristPosition = NetworkTableInstance.getDefault().getTable("Arm").getDoubleTopic("WristPosition").publish();
//        public static DoublePublisher wristVoltage = NetworkTableInstance.getDefault().getTable("Arm").getDoubleTopic("WristVoltage").publish();
//        public static DoublePublisher elbowVoltage = NetworkTableInstance.getDefault().getTable("Arm").getDoubleTopic("ElbowVolts").publish();
//        public static DoublePublisher intakeVoltage = NetworkTableInstance.getDefault().getTable("Arm").getDoubleTopic("IntakeVolts").publish();
//
//        public static DoubleEntry elevatorP = NetworkTableInstance.getDefault().getTable("Arm").getDoubleTopic("ElevatorP").getEntry(ArmConstants.elevatorP);
//        public static DoubleEntry
//        public static DoubleEntry
//
//        public static BooleanPublisher
//
//        public static double
//
//    }
//
//    public PickAndPlaceSubsystem() {
//        //intake
//        elbowMotor.setSmartCurrentLimit(38);
//        wristMotor.setSmartCurrentLimit(38);
//        intakeOneMotor.setSmartCurrentLimit(38);
//        intakeTwoMotor.setSmartCurrentLimit(38);
//        elbowMotor.setInverted(ArmConstants.elbowMotorInverted);
//        wristMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
//        wristMotor.setInverted(true);
//        intakeTwoMotor.setInverted(false);
//        elbowMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
//        elevatorMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
//        intakeOneMotor.setInverted(false);
//        intakeTwoMotor.setInverted(true);
//
//        elbowEncoder.setPositionConversionFactor(2.0 * Math.PI);
//        elbowEncoder.setVelocityConversionFactor((2.0 * Math.PI) / 60.0);
//        elbowEncoder.setInverted(true);
//
//        elevatorEncoder.setPositionConversionFactor(0.003010870139 * 2.4);
//        elevatorEncoder.setVelocityConversionFactor((0.003010870139 * 2.4) / 60.0);
//
//        wristEncoder.setPositionConversionFactor(2.0 * Math.PI);
//        wristEncoder.setVelocityConversionFactor((2.0 * Math.PI) / 60.0);
//        wristEncoder.setInverted(false);
//
//        Telemetry.elevatorP.set(ArmConstants.elevatorP);
//        Telemetry.elbowP.set(ArmConstants.elbowP);
//        Telemetry.wristP.set(ArmConstants.wristP);
//
//        Telemetry.elevatorGoalEntry.set(ArmConstants.elevatorGoal);
//        Telemetry.elbowGoalEntry.set(ArmConstants.elbowGoal);
//        Telemetry.wristGoalEntry.set(ArmConstants.wristGoal);
//    }
//
//
//
//    public double getElbowPositionRadians() {
//        Rotation2d elbowPositionFirst = new Rotation2d(elbowEncoder.getPosition()).plus(new Rotation2d(getAbsoluteWristPosition()));
//        return elbowPositionFirst.minus(new Rotation2d(ArmConstants.elbowEncoderPosOffset)).getRadians();
//    }
//
//    public double getElevatorPositionMeters() {
//        return elevatorEncoder.getPosition();
//    }
//
//    public double getAbsoluteWristPosition() {
//        return Rotation2d.fromDegrees(-wristEncoder.getPosition())
//                .minus(Rotation2d.fromDegrees(ArmConstants.wristEncoderPosOffset)).getRadians();
//    }
//
//
//
//    public void setIntakeOnesVoltage(double voltage) {
//        intakeOnesVoltage = voltage;
//        intakeOneMotor.setVoltage(voltage);
//    }
//
//    public void setIntakeTwosVoltage(double voltage) {
//        intakeTwosVoltage = voltage;
//        intakeTwoMotor.setVoltage(voltage);
//    }
//
//    public void setWristVoltage(double voltage) {
//        wristVoltage = voltage;
//        wristMotor.setVoltage(voltage);
//    }
//
//    public void setElbowVoltage(double voltage) {
//        elbowVoltage = voltage;
//        elbowMotor.setVoltage(voltage);
//    }
//
//    public void setElevatorVoltage(double voltage) {
//        elevatorVoltage = voltage;
//        Telemetry.elevatorVoltage.set(elevatorVoltage);
//        if (!elevatorZeroed) {
//            elevatorMotor.setVoltage(ArmConstants.elevatorZeroingVoltage);
//        } else if (isBottomHit() && voltage < 0.0) {
//            elevatorMotor.setVoltage(0.0);
//        } else if (isTopHit() && voltage > 0.0) {
//            elevatorMotor.setVoltage(0.0);
//        } else {
//            elevatorMotor.setVoltage(voltage);
//        }
//    }
//
//    @Override
//    public void periodic() {
//        super.periodic();
//        if (!elevatorZeroed) {
//            elevatorMotor.setVoltage(ArmConstants.elevatorZeroingVoltage);
//        }
//        if (isBottomHit()) {
//            if (!elevatorZeroed) {
//                elevatorEncoder.setPosition(0.0);
//            }
//            elevatorZeroed = true;
//        } else if (isTopHit()) {
//            //elevatorEncoder.position = ArmConstants.elevatorMaxHeight;
//        }
//
//        Telemetry.elevatorZeroed.set(elevatorZeroed);
//        Telemetry.elbowPosition.set(getElbowPositionRadians());
//        Telemetry.elevatorPosition.set(getElevatorPositionMeters());
//        Telemetry.elbowVelocity.set(elbowEncoder.getVelocity());
//        Telemetry.topHit.set(isTopHit());
//        Telemetry.bottomHit.set(isBottomHit());
//        Telemetry.wristVoltage.set(wristVoltage);
//        Telemetry.elbowVoltage.set(elbowVoltage);
//
//        Telemetry.intakeVoltage.set(intakeOnesVoltage);
//        Telemetry.intakeVoltage.set(intakeTwosVoltage);
//        Telemetry.elevatorPosition.set(getElevatorPositionMeters());
//        Telemetry.elevatorVelocity.set(elevatorEncoder.getVelocity());
//        Telemetry.wristPosition.set(getAbsoluteWristPosition());
//
//        Telemetry.cubeNT.set(CommandValues.cube);
//        Telemetry.coneNT.set(CommandValues.cone);
//        Telemetry.floorNT.set(CommandValues.floor);
//        Telemetry.chuteNT.set(CommandValues.chute);
//        Telemetry.pickupNT.set(CommandValues.pickup);
//        Telemetry.middlePlaceNT.set(CommandValues.middlePlace);
//        Telemetry.highPlaceNT.set(CommandValues.highPlace);
//        Telemetry.groundNT.set(CommandValues.ground);
//
//        Telemetry.visionNT.set(CommandValues.vision);
//
//        ArmConstants.elevatorP = Telemetry.elevatorP.get();
//        ArmConstants.elevatorGoal = Telemetry.elevatorGoalEntry.get();
//        ArmConstants.elbowGoal = Telemetry.elbowGoalEntry.get();
//        ArmConstants.wristGoal = Telemetry.wristGoalEntry.get();
//    }
//}

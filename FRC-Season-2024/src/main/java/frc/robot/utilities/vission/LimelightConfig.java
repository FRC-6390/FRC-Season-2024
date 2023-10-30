package frc.robot.utilities.vission;

public class LimelightConfig {
    private static String DEFUALT_TABLE = "limelight";   
    private double mountingAngle;
    private double mountingHeightMeters;

    public LimelightConfig(double mountingAngle, double mountingHeightMeters)
    {
        this.mountingAngle = mountingAngle;
        this.mountingHeightMeters = mountingHeightMeters;
    }

    public static LimelightConfig defualt(){
        return new LimelightConfig(DEFUALT_TABLE, 0.0, 0.0);
    }
}


// public record SwerveModuleConfig(int driveMotor, boolean driveMotorReversed, int rotationMotor, boolean rotationMotorReversed, int encoder, double encoderOffset, String canbus) {
    
//     public SwerveModuleConfig(int driveMotor, boolean driveMotorReversed, int rotationMotor, boolean rotationMotorReversed, int encoder, double encoderOffset){
//         this(driveMotor, driveMotorReversed, rotationMotor, rotationMotorReversed, encoder, encoderOffset, "can");
//     }
// }

public class SwerveModuleConfig {
    private int driveMotor;
    private boolean driveMotorReversed;
    private int rotationMotor;
    private boolean rotationMotorReversed;
    private int encoder;
    private double encoderOffset;
    private String canbus;

    public SwerveModuleConfig(int driveMotor, boolean driveMotorReversed, int rotationMotor,
            boolean rotationMotorReversed, int encoder, double encoderOffset, String canbus) {
        this.driveMotor = driveMotor;
        this.driveMotorReversed = driveMotorReversed;
        this.rotationMotor = rotationMotor;
        this.rotationMotorReversed = rotationMotorReversed;
        this.encoder = encoder;
        this.encoderOffset = encoderOffset;
        this.canbus = canbus;
    }

    // Getter methods
    public int driveMotor() {
        return driveMotor;
    }

    public boolean driveMotorReversed() {
        return driveMotorReversed;
    }

    public int rotationMotor() {
        return rotationMotor;
    }

    public boolean rotationMotorReversed() {
        return rotationMotorReversed;
    }

    public int encoder() {
        return encoder;
    }

    public double encoderOffset() {
        return encoderOffset;
    }

    public String canbus() {
        return canbus;
    }
}


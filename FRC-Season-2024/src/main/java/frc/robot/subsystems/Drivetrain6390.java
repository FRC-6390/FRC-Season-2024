package frc.robot.subsystems;

//Imports
import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DRIVETRAIN;
import frc.robot.Constants.SWERVEMODULE;
import frc.robot.utilities.controlloop.PID;
import frc.robot.utilities.controlloop.PIDConfig;
import frc.robot.utilities.swerve.SwerveModule;

public class Drivetrain6390 extends SubsystemBase {

  /* 
  Create an array of swerve modules. The swerve module class can be found in utilities. 
  Each module accepts a config class which conatains drive motor channel, rotation motor channel, and encoder channels.
  */
  private static SwerveModule[] swerveModules;

  private static PowerDistribution pdh;
  private static Pigeon2 gyro;

  //Chassis speeds is a class which can contain x-axis speeds, the y-axis speeds, and the rotational speeds.
  private static ChassisSpeeds chassisSpeeds, feedbackSpeeds;

  /*
  Kinematics is a WPILIB class which handles the coordination of swerve modules, 
  converting ChassisSpeeds objects into angles and wheel speeds for each swerve module. 
  */
  public static SwerveDriveKinematics kinematics;

  //Odometry uses encoder positions and gyro heading to calculate the approximate postition of the robot relative to the field
  private static SwerveDriveOdometry odometry;

  //Pose is a class which represents the postition of the robot relative to the field, including X Y and rotation
  private static Pose2d pose;

  //Shuffleboard tabs are different pages on shuffleboard. Widgets which display data can be placed into them
  private static ShuffleboardTab tab, autoTab;

  //A drawing of the ChargedUp gamefield
  private static Field2d gameField;

  //Stuff for drift correction - idk what mathias and mohammed were doing
  private static double desiredHeading;
  private static PIDConfig driftCorrectionPID = new PIDConfig(0.09, 0, 0.1).setILimit(20).setContinuous(-Math.PI, Math.PI);
  private static PID pid;
  private static PIDController rotationPidController = new PIDController(0.3, 0, 0);

  static {

    //Initializing all the variables
    tab = Shuffleboard.getTab("Drive Train");
    autoTab = Shuffleboard.getTab("Auto");
    gameField = new Field2d();
    swerveModules = new SwerveModule[4];
    swerveModules[0] = new SwerveModule(DRIVETRAIN.FRONT_LEFT_MODULE_CONFIG, tab);
    swerveModules[1] = new SwerveModule(DRIVETRAIN.FRONT_RIGHT_MODULE_CONFIG, tab);
    swerveModules[2] = new SwerveModule(DRIVETRAIN.BACK_LEFT_MODULE_CONFIG, tab);
    swerveModules[3] = new SwerveModule(DRIVETRAIN.BACK_RIGHT_MODULE_CONFIG, tab);  
    gyro = new Pigeon2(DRIVETRAIN.PIGEON, DRIVETRAIN.CANBUS);
   
    pdh = new PowerDistribution(DRIVETRAIN.REV_PDH, ModuleType.kRev);
    chassisSpeeds = new ChassisSpeeds();
    feedbackSpeeds = new ChassisSpeeds();

    SwerveModulePosition[] SwervePositions = {swerveModules[0].getPostion(), swerveModules[1].getPostion(), swerveModules[2].getPostion(), swerveModules[3].getPostion()};

    kinematics = new SwerveDriveKinematics(DRIVETRAIN.SWERVE_MODULE_LOCATIONS);
    odometry = new SwerveDriveOdometry(kinematics, Rotation2d.fromDegrees(gyro.getYaw()), SwervePositions);
    pose = new Pose2d();

    pid = new PID(driftCorrectionPID).setMeasurement(() -> pose.getRotation().getDegrees());
    
    rotationPidController.enableContinuousInput(-Math.PI, Math.PI);
  }

  //This function goes and sets each swerve module to an angle and runs them at a certain speed
  public void translate(double angle, double speed)
  {
    swerveModules[0].setToAngle(angle);
    swerveModules[1].setToAngle(angle);
    swerveModules[2].setToAngle(angle);
    swerveModules[3].setToAngle(angle);

    swerveModules[0].setDriveMotor(speed);
    swerveModules[1].setDriveMotor(speed);
    swerveModules[2].setDriveMotor(speed);
    swerveModules[3].setDriveMotor(speed);
  }

  //This functions orients the modules into a spinning configurations and runs them, making the chassis rotate
  public void rotate( double speed)
  {
    swerveModules[0].setToAngle(135);
    swerveModules[1].setToAngle(-45);
    swerveModules[2].setToAngle(45);
    swerveModules[3].setToAngle(-135);

    swerveModules[0].setDriveMotor(speed);
    swerveModules[1].setDriveMotor(speed);
    swerveModules[2].setDriveMotor(speed);
    swerveModules[3].setDriveMotor(speed);
  }

  //Writes data to shuffleboard for debugging
  public void shuffleboard(){
    tab.addDouble("Front Left Encoder", () -> swerveModules[0].getAbsolutePosition());
    tab.addDouble("Front Right Encoder", () -> swerveModules[1].getAbsolutePosition());
    tab.addDouble("Back Left Encoder", () -> swerveModules[2].getAbsolutePosition());
    tab.addDouble("Back Right Encoder", () -> swerveModules[3].getAbsolutePosition());
    
    autoTab.addDouble("Desired Heading", () -> pose.getRotation().getDegrees()).withWidget(BuiltInWidgets.kTextView);
    autoTab.addDouble("PID Desired Heading", () -> pid.calculate(pose.getRotation().getDegrees())).withWidget(BuiltInWidgets.kTextView);

    autoTab.add(gameField);
    autoTab.addDouble("Odometry Heading", () -> pose.getRotation().getDegrees()).withWidget(BuiltInWidgets.kTextView);
    autoTab.addDouble("Odometry X", () -> pose.getX()).withWidget(BuiltInWidgets.kTextView);
    autoTab.addDouble("Odometry Y", () -> pose.getY()).withWidget(BuiltInWidgets.kTextView);

  }

  //Resets everything
  public void init(){
    pdh.clearStickyFaults();
    zeroHeading();
    resetOdometry(pose);
  }

  //Resets gyro + encoder postitions
  public void zeroHeading(){
    gyro.setYaw(0);
    resetOdometry(pose);
  }

  //Resets just gyro
  public static void resetHeading(){
    gyro.setYaw(0);
  }

  //These return gyro values. Call them in commands to get the orientation of the robot
  public double getRoll(){
    return Math.IEEEremainder(gyro.getRoll(), 360); 
  }

  public double getPitch(){
    return Math.IEEEremainder(gyro.getPitch(),360); 
  }

  public double getHeading(){
    return Math.IEEEremainder(gyro.getYaw(), 360);
  }

  public Rotation2d getRotation2d(){
    return Rotation2d.fromDegrees(getHeading());
  }
  

  //Idk if this is ever used but it should calculate where the robot is and move it there
  public void driftCorrection(ChassisSpeeds speeds){
    // double speed = Math.abs(getAverageSpeed()); //method removed pls refer to github if needed
    if(Math.abs(speeds.omegaRadiansPerSecond) > 0.0) desiredHeading = pose.getRotation().getDegrees();
    else speeds.omegaRadiansPerSecond += pid.calculate(desiredHeading);
  }

  /*
  Feed chassis speeds into this function and the variable speeds in this subsystem will be set to that, which is then fed into kinematics in periodic 
  */ 
  public void drive(ChassisSpeeds speeds){
    chassisSpeeds = speeds;
  }

  //Returns position of the robot
  public Pose2d getPose(){
    return pose;
  }

  //Resets the drivetrain to coords 0,0
  public void resetOdometry(Pose2d pose){
    odometry.resetPosition(getRotation2d(), getModulePostions(), pose);
  }

  /*
  Takes in an array of swerve module states, a class which contains the angle and speed of each module. It then sets each module to the desired state
   */
  public void setModuleStates(SwerveModuleState[] states){
    SwerveDriveKinematics.desaturateWheelSpeeds(states, SWERVEMODULE.MAX_SPEED_METERS_PER_SECOND);
    for (int i = 0; i < swerveModules.length; i++) {
      swerveModules[i].setDesiredState(states[i]);
    }
  }

  //A swerve module postion stores both the rotation encoder value and the drive encoder value. Returns these values for every module
  private SwerveModulePosition[] getModulePostions(){
    SwerveModulePosition[] positions = new SwerveModulePosition[swerveModules.length];
    for (int i = 0; i < swerveModules.length; i++) {
      positions[i] = swerveModules[i].getPostion();
    }
    return positions;
  }

  //Never used
  public void feedbackDrive(ChassisSpeeds speeds){
    feedbackSpeeds = speeds;
  }

  //Stops the drive motors on every wheel
  public void stopWheels(){
    for(int i = 0; i < swerveModules.length; i++){
      swerveModules[i].stop();
    }
  }

  //Orients the wheels so that translational motion is no longer possible
  public void lockWheels(){
    
    for (int i = 0; i < swerveModules.length; i++) {
      swerveModules[i].lock();
    }
    swerveModules[0].setToAngle(Math.toRadians(45));
    swerveModules[1].setToAngle(Math.toRadians(135));
    swerveModules[2].setToAngle(Math.toRadians(-45));
    swerveModules[3].setToAngle(Math.toRadians(-135));
  }

  //Undoes the locking
  public void unlockWheels(){
    for (int i = 0; i < swerveModules.length; i++) {
      swerveModules[i].unlock();
    }
  }

  /*
  Updates the position of the robot relative to the field based on encoder values and gyro. Commented stuff is for limelight Also shows the position of the robot on the field in shuffleboard 
  */
  private void updateOdometry(){
    // if(limeLight.hasBotPose()){
      // if(limeLight.getPipeline() == 0){
      //   APRILTAGS tag = APRILTAGS.getByID((int)limeLight.getAprilTagID());
      //   if(!tag.equals(APRILTAGS.INVALID)){
      //     Pose2d relativePose =limeLight.getBot2DPosition();
      //     Pose2d tagPose = tag.getPose2d();
      //     pose = new Pose2d(relativePose.getX() + tagPose.getX(), relativePose.getY() + tagPose.getY(), getRotation2d());
      //   }
      // }
    // }else{
      odometry.update(getRotation2d(), getModulePostions());
      pose = odometry.getPoseMeters();
    // }

    gameField.setRobotPose(pose);
  }

  // public LimeLight getLimelight(){
  //   return limeLight;
  // }

  // public REVBlinkin getBlinkin(){
  //   return blinkin;
  // }

  //This is the main thing - whenever this subsystem is called, this will happen.
  @Override
  public void periodic() {

    //Stores horizontal and vertical speed as well as rotational speed.
    double xSpeed = chassisSpeeds.vxMetersPerSecond + feedbackSpeeds.vxMetersPerSecond;
    double ySpeed = chassisSpeeds.vyMetersPerSecond + feedbackSpeeds.vyMetersPerSecond;
    double thetaSpeed = chassisSpeeds.omegaRadiansPerSecond + feedbackSpeeds.omegaRadiansPerSecond;

    //Combines the individual speeds into a class which stores all of them
    ChassisSpeeds speed = new ChassisSpeeds(xSpeed, ySpeed, thetaSpeed);

    //AGHH WHAT IS THIS BRUH
    driftCorrection(speed);

    /* 
       This converts chasssis speeds into swerve module states. 
       In other words, it calculates the angle and power of each swerve module from the x speed y speed and theta speed 
    */
    SwerveModuleState[] states = kinematics.toSwerveModuleStates(speed);
    
    setModuleStates(states);

    updateOdometry();
  }

  @Override
  public void simulationPeriodic() {
  }

  // @Override
  // public ArrayList<SystemTestAction> getDevices() {
  //   ArrayList<SystemTestAction> actions = new ArrayList<>();

  //   for (int i = 0; i < swerveModules.length; i++) {
  //     actions.add(new SystemTestAction(swerveModules[i]::setDriveMotor, 0.5));
  //     actions.add(new SystemTestAction(swerveModules[i]::setRotationMotor, 0.5));
  //   }
  //   return actions;
  // }

}
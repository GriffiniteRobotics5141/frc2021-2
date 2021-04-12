

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//Robot 1-25, http://10.51.41.11:5801/ 

/* to do list
  organize imports
*/
package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import com.kauailabs.navx.frc.AHRS.SerialDataType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SerialPort;
//import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;

//import com.revrobotics.ColorSensorV3;
//import com.revrobotics.ColorMatchResult;
//import com.revrobotics.ColorMatch;

import com.ctre.phoenix.*;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  boolean win = true;
  boolean ShotsMissed = false;
  private static final String kCenter = "Center";
  private static final String kLeft = "Left";
  private static final String kRight = "Right";
  private static final String kOff = "Off";

  private boolean test;

  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private static final String kManual = "Manual";
  private static final String kSensor = "Sensor";

  private String m_controlSelected;
  private final SendableChooser<String> m_control = new SendableChooser<>();

  private String m_challengeSelected;
  private final SendableChooser<String> m_challange = new SendableChooser<>();

  private static final String kComp = "Competition";
  private static final String kTask1 = "Task1";

  String Mode = "manual";// manual v. sensor

  Joystick gamePad0 = new Joystick(0);
  /*
   * Button mapping 1 - AutoFace 2 - Color 3 - Rotate 4 - Conveyer input ? 5 -
   * Intake toggle 6 - Shooter 7 - 8 - Auto Kill
   */
  VictorSP left0 = new VictorSP(6);
  VictorSP left1 = new VictorSP(7);
  VictorSP right0 = new VictorSP(8);
  VictorSP right1 = new VictorSP(9);
  SpeedControllerGroup leftDrive = new SpeedControllerGroup(left1, left0);
  SpeedControllerGroup rightDrive = new SpeedControllerGroup(right0, right1);
  DifferentialDrive driveTrain = new DifferentialDrive(leftDrive, rightDrive);
  VictorSP colMotor = new VictorSP(3);
  VictorSP conveyor = new VictorSP(5);
  TalonFX shooter = new TalonFX(1);
  VictorSP intake = new VictorSP(4);
  VictorSP climb = new VictorSP(0);
  DigitalInput upperSwitch = new DigitalInput(6);
  DigitalInput lowerSwitch = new DigitalInput(7);
  DigitalOutput ultrasonicPing1 = new DigitalOutput(0);
  DigitalInput ultrasonicEcho1 = new DigitalInput(1);
  DigitalOutput ultrasonicPing2 = new DigitalOutput(2);
  DigitalInput ultrasonicEcho2 = new DigitalInput(3);
  DigitalOutput ultrasonicPing3 = new DigitalOutput(4);
  DigitalInput ultrasonicEcho3 = new DigitalInput(5);
  Ultrasonic ultrasonic1 = new Ultrasonic(ultrasonicPing1, ultrasonicEcho1);
  Ultrasonic ultrasonic2 = new Ultrasonic(ultrasonicPing2, ultrasonicEcho2);
  Ultrasonic ultrasonic3 = new Ultrasonic(ultrasonicPing3, ultrasonicEcho3);

  AHRS navx;

  Timer autoPilotTimer = new Timer();
  Timer autonamousTimer = new Timer();
  Timer autoPeriod = new Timer();
  Timer conveyTimer = new Timer();

  double h2 = 92; // height of target "inches"156+7=163
  double h1 = 33; // height of camera
  double a1 = 22; // angle of camera 20 on test bot
  double disXnum;
  double aimnum;
  double difYnum = h2 - h1 + 7; // +7 is distance between shoter and limelight
  double airtim;
  double fixedAngle = (45) * Math.PI / 180; // angle of shooter 45-47 degrees
  double veloFwoosh; // angular velocity variable
  double velocityToMotorRatio = 1022; // conversion rate

  int navxStep = 0;
  int rotatenum = 0;
  double targetAngle;
  boolean autoFace;
  double autoFaceTimeNeeded;
  boolean autoPilotState = false;
  boolean doAutoPilotNow = false;
  boolean doubleAuto = false;
  double naenae; // it got less funny
  boolean Ball1;
  boolean Ball2;
  boolean Ball3;
  boolean intakeOn = false;
  String colorString;
  String gameData;
  String nextColor = "Purple Baby";
  String gameSadFace = "Mehh";
  boolean manualMode = true;

  int tapePipeline = 0;
  int ballPipeline = 1;
  int conePipeline = 2;
/*
  private final I2C.Port cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 cSensor = new ColorSensorV3(cPort);
  private final ColorMatch m_colorMatcher = new ColorMatch();
  private final Color BlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
  private final Color GreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
  private final Color RedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
  private final Color YellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);
*/
  double yaw;

  NetworkTable table;
  NetworkTableEntry tx;
  NetworkTableEntry ty;
  NetworkTableEntry ta;
  NetworkTableEntry tv;
  double x;
  double y;
  double a;
  double v;

  int seenColor;

  double range1;
  double range2;
  double range3;

  int memBall;

  Timer warmUp = new Timer();

  String navDrive = "null";
  double setAngle = 0;
  double angledYaw;
  double AOC = 85; //Area of correction


  Timer challengeTimer = new Timer();
  int challengeTimerCheckpoint;
  double route;

  double minCorrectNavX = .34;
  double maxCorrectNavX = .65;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    // m_choosers on bottom
    SmartDashboard.putData("Auto positions", m_chooser);
    m_chooser.addOption("Center", kCenter);
    m_chooser.addOption("Left", kLeft);
    m_chooser.addOption("Right", kRight);
    m_chooser.setDefaultOption("Off", kOff);

    SmartDashboard.putData("Control Mode", m_control); //Conveyor
    m_control.setDefaultOption("Manual", kManual);
    m_control.addOption("Sensor", kSensor);

    SmartDashboard.putData("Game Mode", m_challange);
    m_challange.setDefaultOption("Competition", kComp);
    m_challange.addOption("Task1", kTask1);
    m_challenge.addOption("Task2", kTask2);
    m_challenge.addOption("Task3", kTask3);
    m_challenge.addOption("Task4", kTask4);


    navx = new AHRS(SerialPort.Port.kMXP, SerialDataType.kProcessedData, (byte)50);
    // (byte)50);
    right0.setInverted(true);
    right1.setInverted(true);
    colMotor.setInverted(true);
    conveyor.setInverted(true);
    CameraServer.getInstance().startAutomaticCapture();

    autoPilotTimer.reset();
    autoPilotTimer.stop();
    autonamousTimer.reset();
    autonamousTimer.stop();
    autoPeriod.reset();
    autoPeriod.stop();
    // navx.reset();
    // navx.zeroYaw();
    colMotor.set(0);
    rotatenum = 0;
    seenColor = 0;
    warmUp.reset();

    table.getEntry("ledMode").setNumber(3);
    memBall = 0;
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {

    yaw = navx.getYaw();

    table = NetworkTableInstance.getDefault().getTable("limelight");
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");
    tv = table.getEntry("tv");

    x = (tx.getDouble(0.0)); // x & y is negative because limelight is upsidedown
    y = (ty.getDouble(0.0));
    a = ta.getDouble(0.0);
    v = tv.getDouble(0.0);

    double marginXerror = 172;// 168
    double ratioX = x / 27; // was (x-6)/27 on 3/9
    // double ratioY = (1.81-y)/20; // was y/20 Based of angle target is seen at
    double ratioY = (disXnum - marginXerror) / 25; // Based of distance of target from dsXnum 108, 35
    double ratioA = (2.68 - a);// changed <--- thank you very cool 1/25
    double minCorrectX = .25;
    double maxCorrectX = .6;
    double minCorrectY = .1;
    double maxCorrectY = .4;

    // double sineWithSignum =
    // Math.signum(ratioX)*(1-min)*Math.sin(ratioX*Math.PI/2)+(1+min)/2;
    double sineX = Math.signum(ratioX) * ((maxCorrectX - minCorrectX) / 2) * Math.sin(Math.PI * (ratioX - .5))
        + Math.signum(ratioX) * ((maxCorrectX + minCorrectX) / 2);
    double sineY = Math.signum(ratioY) * ((maxCorrectY - minCorrectY) / 2) * Math.sin(Math.PI * (ratioY - .5))
        + Math.signum(ratioY) * ((maxCorrectY + minCorrectY) / 2);

    SmartDashboard.putNumber("SineX", sineX);
    SmartDashboard.putNumber("SineY", sineY);

    if (doAutoPilotNow && v == 1) { // a button
      autoPilotState = true;
    } else {
      doAutoPilotNow = false;
      autoPilotState = false;
    }

    if (autoPilotState) {
      if (v == 1) {
        driveTrain.tankDrive(sineX + sineY, -(sineX) + sineY);
      }
      if (x > -1 && x < 1 && disXnum > (marginXerror - 1.5) && disXnum < (marginXerror + 1.5)) {
        if (autoPilotTimer.get() == 0) {
          autoPilotTimer.start();
        } // autoPilotTimer stops prolonged aim jitter
      } else {
        autoPilotTimer.reset();
        autoPilotTimer.stop();
      }
      if (autoPilotTimer.hasPeriodPassed(1)) {
        autoPilotState = false;
        doAutoPilotNow = false;
      }
    }

    // disXnum = ((h2-h1)/Math.tan((a1-y)*Math.PI/180));
    disXnum = (h2 - h1) / (Math.tan((a1 + y) * Math.PI / 180)) + 7; // +7 is distance between shooter and limelight

    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightA", a);
    SmartDashboard.putNumber("ratioX", ratioX);
    SmartDashboard.putNumber("D-Pad", gamePad0.getPOV());
    // SmartDashboard.putNumber("Yaw",navx.getYaw());
    // SmartDashboard.putNumber("NavxStep",navxStep);

    SmartDashboard.putBoolean("Auto", doAutoPilotNow);
    SmartDashboard.putBoolean("isBall1", Ball1);
    SmartDashboard.putBoolean("isBall2", Ball2);
    SmartDashboard.putBoolean("isBall3", Ball3);

    SmartDashboard.putNumber("DisXNum", disXnum);
    SmartDashboard.putNumber("DifYNum", difYnum);
    SmartDashboard.putNumber("aimnum", aimnum);
    SmartDashboard.putNumber("Velocity", veloFwoosh);
    SmartDashboard.putNumber("airtime", airtim);


    double ratioNavX;

    if (Math.abs(yaw - setAngle) <= 180){
      angledYaw = yaw - setAngle;
    }else{
      angledYaw = -Math.signum(yaw - setAngle)*(360-Math.abs(yaw - setAngle));
    }

    if(Math.abs(angledYaw) < (AOC)) {
      ratioNavX =angledYaw/AOC;
    } else if (angledYaw > 0){
      ratioNavX = 1;
    } else { ratioNavX = -1;}

    //double sineWithSignum = Math.signum(ratioNavX)*(1-min)*Math.sin(ratioNavX*Math.PI/2)+(1+min)/2;
    double sineNavX = Math.signum(ratioNavX)*((maxCorrectNavX - minCorrectNavX)/2)*Math.sin(Math.PI*(ratioNavX-.5))+Math.signum(ratioNavX)*((maxCorrectNavX + minCorrectNavX)/2);
    
  if(navDrive.length() > 0) {
    switch (navDrive.charAt(0)) {
      
      case 'T' :
        minCorrectNavX = .34; //.34
        maxCorrectNavX = .5;
        AOC = 85;

        driveTrain.tankDrive(-sineNavX,sineNavX);
        break;
      case 'D' :
        minCorrectNavX = 0.51;//.4
        maxCorrectNavX = 0.81;//.75
        AOC = 15;//15

        double cCorrection = (-sineNavX > 0) ? -sineNavX : minCorrectNavX;
        double ccCorrection  = (sineNavX > 0) ? sineNavX : minCorrectNavX;

        driveTrain.tankDrive(cCorrection,ccCorrection); 
    
        break;
      default :
        minCorrectNavX = .34;
        maxCorrectNavX = .65;
        AOC = 85;
        break;
    }
  }


  }

  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    m_challengeSelected = m_challange.getSelected();

    autonamousTimer.reset();
    autonamousTimer.stop();
    autoPeriod.reset();
    autoPeriod.stop();
    table.getEntry("ledMode").setNumber(3);


    navx.zeroYaw();
    navx.reset();

    challengeTimer.stop();
    challengeTimer.reset();
    navDrive = "null";

    shooter.set(ControlMode.PercentOutput, 0);

    manualMode = false;

    memBall = 0;

    ultrasonic1.setEnabled(true);
    ultrasonic2.setEnabled(true);
    ultrasonic3.setEnabled(true);

  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    doUltraSonics();

    /*switch (m_autoSelected) {
    case kLeft:
      // Put left auto code here
      driveTrain.tankDrive(-.5, .5);
      break;
    case kRight:
      driveTrain.tankDrive(.5, -.5);
      // Put right auto code here
      break;
    case kCenter:
      if (autonamousTimer.get() == 0) {
        autonamousTimer.start();
      }
      if (autonamousTimer.get() <= 1) {
        driveTrain.tankDrive(-.5, -.5);
        shooter.set(ControlMode.PercentOutput, 0); //0.8
      } else if (v != 1) {
        driveTrain.tankDrive(.45, -.45);
      } else if (v == 1) {
        if (autoPeriod.get() == 0) {
          autoPeriod.start();
        }
        doAutoPilotNow = true;
        if (autoPeriod.get() > 2 && autoPeriod.get() <= 4) {
          doAutoPilotNow = false;
          conveyor.set(.7);
          intake.set(.5);
        }
      }

      break;
    case kOff:
      break;
    default:
      // funny
      driveTrain.tankDrive(0, 0);
      break;
    }*/


    SmartDashboard.putNumber("challengeTimer", challengeTimer.get());
    SmartDashboard.putNumber("route number", challengeTimer.get());
  
      switch (m_challengeSelected) {
        case kComp:
          // Put left auto targetting and shooting code here
          //driveTrain.tankDrive(-.5, .5);
          break;

        //Use Yellow Limelight Snapshot setting
        case kTask1:
          if (challengeTimer.get() == 0) {
            route = y;
            challengeTimer.start();
            table.getEntry("pipeline").setNumber(ballPipeline);
          } 

          if( route <= 0 ) { //blue config A
            intakeOn = true;
            autoIntake();

            if ( challengeTimer.get() <= 1.0) { // following seconds are ~1 second(s) are shorter
              navDrive = "Drive";
            } else if ( challengeTimer.get() <= 3) {
              navDrive = "null";
            } else if ( challengeTimer.get() <= 4) {
              turnThing(20, 4);
            } else if ( challengeTimer.get() <= 5) {//this if is broken, also check if turn progress bollean is ok
              navDrive = "Drive";
            } else if ( challengeTimer.get() <= 7) {
              navDrive = "null";
            } else if ( challengeTimer.get() <= 8){
              turnThing(-68, 8);
            } else if ( challengeTimer.get() <= 10) {//this if is broken, also check if turn progress bollean is ok
              navDrive = "Drive";
            } else if ( challengeTimer.get() <= 13) {
              navDrive = "null";
            } else if ( challengeTimer.get() <= 14){
              turnThing(0, 14);
            } else if ( challengeTimer.get() <= 20) {//this if is broken, also check if turn progress bollean is ok
              navDrive = "Drive";
            } else {
              route = -1;
              //challengeTimer.reset();
            }

            /*if(challengeTimer.get() < 5) { // following seconds are ~1 second(s) are shorter
              navDrive = "Drive";
            } else if(((int)challengeTimer.get()) == 5){
              turnThing(0, 5);
            } 
            else if (challengeTimer.get() < 8) {//this if is broken, also check if turn progress bollean is ok
              navDrive = "Drive";
            } else if(((int)challengeTimer.get()) == 8){
              turnThing(-71.57, 8);
            }else if (challengeTimer.get() < 11) {//this if is broken, also check if turn progress bollean is ok
              navDrive = "Drive";
            } else if(((int)challengeTimer.get()) == 11){
              turnThing(63.4, 11);
            } else if (challengeTimer.get() < 15) {//this if is broken, also check if turn progress bollean is ok
              navDrive = "Drive";
            } else if(((int)challengeTimer.get()) == 15){
              turnThing(0, 15);
            } else {
              challengeTimer.reset();
            }*/
          }

          
          
           else  { //red config A


            
            }


          break;
        default:
          break;
    }


  }

  @Override
  public void teleopInit() {

    autoPilotTimer.start();
    navx.reset();
    navx.zeroYaw();
    colMotor.set(0);
    rotatenum = 0;
    ultrasonic1.setEnabled(true);
    ultrasonic2.setEnabled(true);
    ultrasonic3.setEnabled(true);
    table.getEntry("ledMode").setNumber(3);

    shooter.setInverted(true);

    table.getEntry("ledMode").setNumber(3);

    navDrive = "null";




  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {

    m_controlSelected = m_control.getSelected();

    /* SmartDashboard.putString("gameData", gameData);
    SmartDashboard.putString("DetectedColor",colorString); just so Iremember
    SmartDashboard.putString("nextColor", nextColor);
    SmartDashboard.putString("gameSadFace", gameSadFace);
    SmartDashboard.putNumber("rotatenum", rotatenum);
    */
    SmartDashboard.putNumber("ultrasonicRange1", ultrasonic1.getRangeMM());
    SmartDashboard.putNumber("ultrasonicRange2", ultrasonic2.getRangeMM());
    SmartDashboard.putNumber("ultrasonicRange3", ultrasonic3.getRangeMM());
    SmartDashboard.putBoolean("ultrasonic1", Ball1);
    SmartDashboard.putBoolean("ultrasonic2", Ball2);
    SmartDashboard.putBoolean("ultrasonic3", Ball3);

    SmartDashboard.putBoolean("Intake Test", test);

    /*
     * if(ultrasonic1.getRangeMM()>0.1){range1 = ultrasonic1.getRangeMM();}
     * if(ultrasonic2.getRangeMM()>0.1){range2 = ultrasonic2.getRangeMM();}
     * if(ultrasonic3.getRangeMM()>0.1){range3 = ultrasonic1.getRangeMM();}
     */

    if (ultrasonic1.getRangeMM() != 0.0) {
      if (ultrasonic1.getRangeMM() > 125) {
        Ball1 = false;
      } else {
        Ball1 = true;
      }
    }
    if (ultrasonic2.getRangeMM() != 0.0) {
      if (ultrasonic2.getRangeMM() > 100) {
        Ball2 = false;
      } else {
        Ball2 = true;
      }
    }
    if (ultrasonic3.getRangeMM() != 0.0) {
      if (ultrasonic3.getRangeMM() > 106) {
        Ball3 = false;
      } else {
        Ball3 = true;
      }
    }
    ultrasonic1.ping();
    ultrasonic2.ping();
    ultrasonic3.ping();

    SmartDashboard.putNumber("WarmUP", warmUp.get());
    SmartDashboard.putNumber("mem ball", memBall);

    if (gamePad0.getRawButtonPressed(7)) {
      manualMode = !manualMode;
      intakeOn = false;
    }
    SmartDashboard.putBoolean("Manual", manualMode);

    if (manualMode) {

      if (gamePad0.getPOV() != 270) {
        intake.set(gamePad0.getRawAxis(2));
      } else {
        intake.set(-.5);
      }
      if (gamePad0.getRawButton(5) && gamePad0.getPOV() != 270) {
        conveyor.set(1);
      } else if (gamePad0.getPOV() == 270) {
        conveyor.set(-.5);
      } else {
        conveyor.set(0);
      }

      if (gamePad0.getRawButton(6)) {
        shooter.set(ControlMode.PercentOutput, 0.8);
        memBall = 0;
      } else {
        shooter.set(ControlMode.PercentOutput, 0);
      }
    } else { //autoMode

      if (gamePad0.getRawButtonPressed(4)) {
        intakeOn = !intakeOn;
      }

      if (gamePad0.getRawButton(6)) {// shooter Righth bumper
        shooter.set(ControlMode.PercentOutput, 0.8/* veloFwoosh*velocityToMotorRatio */);// shooter value depending on target distance x and y
      } else {
        shooter.set(ControlMode.PercentOutput, 0);
      }

      if (gamePad0.getRawButton(5)) {
        // intake.set(.7);
        conveyor.set(1);
      }
      autoIntake();
      /*if (!gamePad0.getRawButton(6) || !gamePad0.getRawButton(5) || !(gamePad0.getPOV() == 270)) { // Not pressing these buttons
        if (intakeOn && memBall < 2) {// auto intake
          intake.set(.6);
          if (Ball1) { // button 4 questionable, propose we do it autonomous
            conveyor.set(1);
            conveyTimer.stop();
            conveyTimer.reset();
          } else {
            if (conveyTimer.get() == 0) { // when clock is zero, start it
              conveyTimer.start();

            }
            if (conveyTimer.get() >= .34) { // when clock is over X stop conveyor
              if (conveyor.get() != 0) {
                conveyor.stopMotor();
                memBall++;
              }

            }

          }


        } else if (intakeOn && memBall == 2 && !Ball1){
          intake.set(.6);
        } else if (memBall == 2 && Ball1 && Ball2) {
          manualMode = true;
        } else if (gamePad0.getPOV() == 270) {
          intake.set(-.5);
          conveyor.set(-.85);
        } else if (!Ball1 && Ball3 && intakeOn) {// not shooting
          intake.set(.65); // coolDown Turn off for testing
        } else {// default
          intake.stopMotor();
          conveyor.stopMotor();
          conveyor.set(0);
        }
    }*/


      if (gamePad0.getPOV() == 270) {
        intakeOn = false;
      }
    }
  

    SmartDashboard.putBoolean("Upperswitch", upperSwitch.get());
    SmartDashboard.putBoolean("Lowerswitch", lowerSwitch.get());

    if (gamePad0.getPOV() == 0 && upperSwitch.get()) {
      climb.set(.7);
    } else if (gamePad0.getPOV() == 180 && lowerSwitch.get()) {
      climb.set(-.7);
    } else {// default
      climb.set(0);
    }

    if (!autoPilotState) {
      SmartDashboard.putNumber("leftStick", gamePad0.getRawAxis(1));
      SmartDashboard.putNumber("rightStick", gamePad0.getRawAxis(5));
      double leftStick = (-gamePad0.getRawAxis(1) * .6) * (gamePad0.getRawAxis(3) + 1);
      // double leftStick =
      // (-gamePad0.getRawAxis(1)*((gamePad0.getRawAxis(3)==1)?.6:.8));
      double rightStick = (-gamePad0.getRawAxis(5) * .6) * (gamePad0.getRawAxis(3) + 1);
      // double rightStick =
      // (-gamePad0.getRawAxis(5)*((gamePad0.getRawAxis(3)==1)?.6:.8));
      driveTrain.tankDrive(leftStick, rightStick);// 12/13 is motor ratio for simon none for flash
    }

    if (autoPilotState || autoFace) {
      gamePad0.setRumble(RumbleType.kRightRumble, 1);
      gamePad0.setRumble(RumbleType.kLeftRumble, 1);
    } else {
      gamePad0.setRumble(RumbleType.kRightRumble, 0);
      gamePad0.setRumble(RumbleType.kLeftRumble, 0);
    }

    SmartDashboard.putBoolean("autopilot", autoPilotState);

    if (gamePad0.getRawButtonPressed(1)) {
      doAutoPilotNow = !doAutoPilotNow;
    }
    if (gamePad0.getRawButtonPressed(6) || gamePad0.getRawButtonPressed(10) || Math.abs(gamePad0.getRawAxis(1)) >= .2
        || Math.abs(gamePad0.getRawAxis(5)) >= .2) { // a button
      autoPilotState = false;
      doAutoPilotNow = false;
    }

    if (gamePad0.getRawButtonPressed(8)) { // start button is kill switch for autoPilot, autoTurnLeft, and autoTurnRight
      autoPilotState = false;
      doAutoPilotNow = false;
      seenColor = 0;
      rotatenum = 0;
      warmUp.reset();
      warmUp.stop();
      shooter.set(ControlMode.PercentOutput, 0);
      intake.set(0);
      conveyor.set(0);
      intakeOn = false;
    }

  }

  /**
   * This function is called periodically during test mode.
   */

  @Override
  public void testPeriodic() {
  }

  public void turnThing(double angle, double time) {
    System.out.println("turning");
    setAngle = angle;
    if(challengeTimer.get() <= time) { //move forward to first ball
      challengeTimer.stop();
      driveTrain.tankDrive(0, 0);
      if (Math.abs(angledYaw) <= 2) {
        navDrive = "null";
        challengeTimer.start();
      } else {
        navDrive = "Turn";
        challengeTimer.stop();
        }
      }
  }
  public void autoIntake() {
    if (!gamePad0.getRawButton(6) || !gamePad0.getRawButton(5) || !(gamePad0.getPOV() == 270)) { // Not pressing these buttons
      if (intakeOn && memBall < 2) {// auto intake
        intake.set(.6);
        if (Ball1) { // button 4 questionable, propose we do it autonomous
          conveyor.set(1);
          conveyTimer.stop();
          conveyTimer.reset();
        } else {
          if (conveyTimer.get() == 0) { // when clock is zero, start it
            conveyTimer.start();

          }
          if (conveyTimer.get() >= .34) { // when clock is over X stop conveyor
            if (conveyor.get() != 0) {
              conveyor.stopMotor();
              memBall++;
            }

          }

        }


      } else if (intakeOn && memBall == 2 && !Ball1){
        intake.set(.6);
      } else if (memBall == 2 && Ball1 && Ball2) {
        manualMode = true;
      } else if (gamePad0.getPOV() == 270) {
        intake.set(-.5);
        conveyor.set(-.85);
      } else if (!Ball1 && Ball3 && intakeOn) {// not shooting
        intake.set(.65); // coolDown Turn off for testing
      } else {// default
        intake.stopMotor();
        conveyor.stopMotor();
        conveyor.set(0);
      }
  }

}
public void doUltraSonics() {
  if (ultrasonic1.getRangeMM() != 0.0) {
    if (ultrasonic1.getRangeMM() > 125) {
      Ball1 = false;
    } else {
      Ball1 = true;
    }
  }
  if (ultrasonic2.getRangeMM() != 0.0) {
    if (ultrasonic2.getRangeMM() > 100) {
      Ball2 = false;
    } else {
      Ball2 = true;
    }
  }
  if (ultrasonic3.getRangeMM() != 0.0) {
    if (ultrasonic3.getRangeMM() > 106) {
      Ball3 = false;
    } else {
      Ball3 = true;
    }
  }
  ultrasonic1.ping();
  ultrasonic2.ping();
  ultrasonic3.ping();
}
}
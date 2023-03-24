// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.



//notes
// 2 direction tap controlls 
// straghit up and down  
// grip and relese 
// map second controller
// tune pids 
// interlock to prevent extention while rotating 
// anolog rotation 
// manual overide and locking
package frc.robot.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
//import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import com.revrobotics.CANSparkMax;
//import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.robot.subsystems.Swerve;
import pabeles.concurrency.ConcurrencyOps.NewInstance;
import frc.robot.robot.Constants;
import java.sql.Time;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
//import edu.wpi.first.wpilibj.Timer;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static CTREConfigs ctreConfigs;
  private final Timer m_timer = new Timer();
  private final Timer autoTimer = new Timer();
   // Joysticks
  private final Joystick m_Flight = new Joystick(0);
  private final static XboxController m_Driver2 = new XboxController(1);

  // solenoids
  private final Solenoid m_IntakeSolenoid = new Solenoid(31,PneumaticsModuleType.REVPH, 1);
  private Solenoid m_gripperSolenoid = new Solenoid(31,PneumaticsModuleType.REVPH, 0);
  private final Solenoid mrflippySolenoid = new  Solenoid(31, PneumaticsModuleType.REVPH, 4 );
  private final Solenoid mrflippySolenoidinvesre = new  Solenoid(31, PneumaticsModuleType.REVPH, 5 );
  public static boolean mrflippystate;
  private final Solenoid brakSolenoid = new Solenoid(31,
  PneumaticsModuleType.REVPH, 3 ); // you madmen wrote the WHOLE CODE with a misspelling of "break?!?!?!?!?"
 // Motors

 

 //Auto Balance Varibles 
 public static double autobalancespeed;
 public static boolean autobalancesbutton;
 public double gyropitch;
 public static boolean autonautobalance;


// Analog Input
//public AnalogInput m_armAnalogInput = new AnalogInput(0);
//led
private PWMSparkMax Blinken = new PWMSparkMax(0); // and blinkin replaced with "blinken!?!?!?!"

 //Digital Inputs
public DigitalInput armhomingswitch = new DigitalInput(0);
public DigitalInput cubesensor = new DigitalInput(1);
public DigitalInput armhomingswitchbase = new DigitalInput(9);
public boolean cubeaquired;


 //Intake
private static final int IntakeDeviceID = 23;
private CANSparkMax m_IntakeMotor;
private boolean reverseintake;
//private static final int ConvayorDeviceID = 26;
//private CANSparkMax m_convayor;
 //Telescope
private static final int TelescopeDeviceID = 25;
//private CANSparkMax m_Telescope;
private WPI_TalonSRX m_Telescope;
//Rotate Motor
private static final int RotateID = 24;
private CANSparkMax m_rotate;

 // Controller
private static final int kSolenoidButton = 12;
// Telescope Stuff
public SparkMaxPIDController m_pidController;
//private RelativeEncoder m_encoder;
public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
public int m_telescopehome;
public double telescopeencoderpos;
public int homingstep;
public Double telescopecommandposition;
static final double[] kTelescopearray = {-45000,-23000,-13500,-13500,-23000,-45000};
public double stickDeadband;
//{-45000,-23000,-22000,-14000,-14000,-22000,-23000,-45000}
private boolean extenedTelescope;
private double telescopemanual;
private boolean telescopesafe;

//Arm Rotation Stuff
static final int kPotChannel = 0;  
static final double kFullHeightMeters = 5;
static final double[] kSetpointsMeters = {1.45,1.60,2.87,2.98,4.25,4.39};
//{1.45,1.53,2.1,2.85,2.98,3.55,4.25,4.39}
private static final double kPa = 1.7;
private static final double kIa = 0.0;
private static final double kDa = 0.0;
public double armoffsetamount = .1;//was at .025
public double armwithoffset;
public boolean armhigh;
public boolean armmid;
public boolean armlow;
public boolean armrotated;
SendableChooser<Integer> m_numChooser = new SendableChooser<>();

public final PIDController m_armpidController = new PIDController(kPa,kIa, kDa);  
private final AnalogPotentiometer m_potentiometer = 
  new AnalogPotentiometer(kPotChannel, 5);
 private int m_index;
private double targetpos = 0;
private  boolean  isarminmanuel;

private boolean stopauto;
private Integer autonselection;
//private boolean armnotatcommandedpos;

//private double autoroutine;
private Integer autonnumber;
public Integer autostep;
private Command m_CENTERAUTO;
private Command m_LEFT2PIECE;
private Command m_LEFTPLACEBALANCE;
private Command m_RIGHT2PIECE;
private Command m_RIGHTPLACEBALANCE;
private RobotContainer m_robotContainer;


/**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    //s_swerve = new Swerve();
    SmartDashboard.putNumber("P Gain", kP);
    autonselection = 0;
    SmartDashboard.putNumber("auto selection", autonselection);
    ctreConfigs = new CTREConfigs();
    isarminmanuel = false;
    homingstep = 0;
    
    
    telescopemanual = 0;
    m_telescopehome = 0;
    telescopecommandposition = 0.0;
    telescopesafe = false;
    m_Telescope = new WPI_TalonSRX(TelescopeDeviceID);
    m_Telescope.configFactoryDefault();
    m_Telescope.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,0,100);
    m_Telescope.setInverted(false);
    m_Telescope.enableCurrentLimit(true);
    m_Telescope.configPeakCurrentLimit(50, 100);
    m_Telescope.configClosedloopRamp(.15);
    m_Telescope.setSensorPhase(true);
    m_Telescope.configNominalOutputForward(0);
		m_Telescope.configNominalOutputReverse(0);
		m_Telescope.configPeakOutputForward(1);
		m_Telescope.configPeakOutputReverse(-1);
    m_Telescope.configAllowableClosedloopError(0,0);
    telescopeencoderpos = 1500;
    		/* Config Position Closed Loop gains in slot0, tsypically kF stays zero. */
		
		m_Telescope.config_kP(0, .3);
		m_Telescope.config_kI(0, 0);
		m_Telescope.config_kD(0, 0);
    m_Telescope.config_kF(0, 0);
    m_rotate = new CANSparkMax(RotateID, MotorType.kBrushless);
    
    autostep = 0;
    stopauto = false;
    autonnumber = 0;
		 //auto balance 
  autobalancespeed = 0;
   autobalancesbutton = false;
   gyropitch = 0;
   autonautobalance = false; 
  m_robotContainer = new RobotContainer(); 
 
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
   // auto chooser 

   Shuffleboard.getTab("Autonomous").add(m_numChooser);
   m_numChooser.setDefaultOption("center auto then balance", 0);
   m_numChooser.addOption("right side 2 game piece", 1);
   m_numChooser.addOption("right side place then balance", 2); 
   m_numChooser.addOption("left side 2 game piece", 3); 
   m_numChooser.addOption("left side place then balance", 4); 
   m_numChooser.addOption("do nothing", 5); 

    //motors
   m_IntakeMotor = new CANSparkMax(IntakeDeviceID, MotorType.kBrushed);
   reverseintake = false;
   m_IntakeMotor.setOpenLoopRampRate(2);
   //m_convayor = new CANSparkMax(ConvayorDeviceID, MotorType.kBrushed);
  
  


   // Resets
   m_IntakeMotor.restoreFactoryDefaults();
   //m_convayor.restoreFactoryDefaults();
   m_rotate.restoreFactoryDefaults(); 
   m_rotate.setIdleMode(IdleMode.kBrake);
   m_rotate.setInverted(true);
   m_rotate.setOpenLoopRampRate(.4);
   m_rotate.setClosedLoopRampRate(.2);
   
   Blinken.set(-.89);
   

   //Rotate PID Stuff
  kP = 0.1; 
   kI = 0;
   kD = 0; 
   kIz = .1; 
   kFF = .1; 
   kMaxOutput = .1; 
   kMinOutput = -.1;


   SmartDashboard.putNumber("P Gain", kP);
   SmartDashboard.putNumber("I Gain", kI);
   SmartDashboard.putNumber("D Gain", kD);
   SmartDashboard.putNumber("I Zone", kIz);
   SmartDashboard.putNumber("Feed Forward", kFF);
   SmartDashboard.putNumber("Max Output", kMaxOutput);
   SmartDashboard.putNumber("Min Output", kMinOutput);
   SmartDashboard.putNumber("Set Rotations", 0);
   
  //Arm rotate stuff 
    // Move to the bottom setpoint when teleop starts
    m_index = 3;
    //m_armpidController.setSetpoint(kSetpointsMeters[m_index]);  
    extenedTelescope = false;

  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    double tyb = LimelightHelpers.getTY("limelight-bottom");
    double tab = LimelightHelpers.getTA("limelight-bottom");
    double tyt = LimelightHelpers.getTY("limelight-top");
    double tat = LimelightHelpers.getTA("limelight-top");
//autobalance logic
gyropitch = Swerve.gyro.getRoll();
autobalancesbutton = m_Flight.getRawButton(13);
if (autobalancesbutton == true || autonautobalance ==true){
 
 autobalancespeed = 0-gyropitch*.055;
   /* if(gyropitch < -12){
      autobalancespeed = .55;
    }else if(gyropitch > 12){
      autobalancespeed = -.55;}
*/
    }
 SmartDashboard.putNumber("gyro pitch", gyropitch);
 SmartDashboard.putBoolean("autobalance button", autobalancesbutton);
 SmartDashboard.putBoolean("stop auto?", stopauto);
 SmartDashboard.getBoolean("stop auto?", false);
SmartDashboard.putNumber("selected auto number", m_numChooser.getSelected());
autonnumber = m_numChooser.getSelected();

 if (tyt< -.25
 && tyt>-5.25
&& tat != 0
&& (m_index == 1|| m_index == 0)){
Blinken.set(.57);
}else{
 if (tyb<1.39
     && tyb>-3.61
    && tab != 0
    && (m_index == 4|| m_index == 5)){
  Blinken.set(.57);
 }else{
    if (m_index == 0 || m_index == 5){
      armhigh = true;
      Blinken.set(.23);
    }else{
     Blinken.set(-.89);}
    if (m_index == 1 || m_index == 4){
      armmid = true;
      Blinken.set(-.31);
     }
     if (m_index == 2){
      armmid = true;
      Blinken.set(.83);
     }}}
     
     

    //Telecope Section


    SmartDashboard.putNumber("arm position", m_potentiometer.get());
  SmartDashboard.putNumber("telescopecommandposition", telescopecommandposition);      
  SmartDashboard.putNumber("m index", m_index);    
  SmartDashboard.putBoolean("arm manual mode", isarminmanuel);
  SmartDashboard.putBoolean("arm homing switch", armhomingswitch.get());
  SmartDashboard.putBoolean("arm homing switch base", armhomingswitchbase.get());  
  SmartDashboard.putNumber("Target Arm POS", targetpos);
  //SmartDashboard.putNumber("calculated arm pos", m_armpidController.calculate(position));
  SmartDashboard.putNumber("arm motor speed", m_rotate.getAppliedOutput());}

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {

    autostep = 0;
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */

  @Override
  public void autonomousInit() {

   
    m_CENTERAUTO = m_robotContainer.getCENTERAUTO();
    m_LEFT2PIECE = m_robotContainer.getLEFT2PIECE();
    m_LEFTPLACEBALANCE = m_robotContainer.getLEFTPLACEBALANCE();
    m_RIGHT2PIECE = m_robotContainer.getRIGHT2PIECE();
    m_RIGHTPLACEBALANCE = m_robotContainer.getRIGHTPLACEBALANCE();
    homingstep = 0;
    m_telescopehome = 0;
    cubeaquired = false;

    // schedule the autonomous command (example)
    /*if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }*/
    
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    brakSolenoid.set(true);
    if (homingstep == 0){
      homingstep = 1;
    }
    
    //1
    if (homingstep == 1 
      && !armhomingswitchbase.get() 
      && (m_potentiometer.get()+armoffsetamount>3.1) 
      && (m_potentiometer.get()+armoffsetamount< 3.6))
      {
      homingstep = 2;}
      else if (homingstep == 1){homingstep = 2;
    }
    
    //2
    if (homingstep == 2 && !armhomingswitch.get()){
      homingstep = 5;}
      else if (homingstep == 2){homingstep = 3;
    }
    
    //3
    if (homingstep == 3 && !armhomingswitch.get()){
      homingstep = 5;
    }
      else if (homingstep ==3){ 
      m_Telescope.set(.25); 
    }
    

    //4
    if (homingstep == 4){
      m_Telescope.set(0);
      m_Telescope.setSelectedSensorPosition(-7700);
      m_telescopehome = 1;
      homingstep = 6;
    }
    //5
    if (homingstep == 5){
      m_Telescope.set(0);
      m_Telescope.setSelectedSensorPosition(0);
      m_telescopehome = 1;
      homingstep = 6;}
/*auton routine number and auton step number
 * reset gyro
 * verify arm is homed
 * rotate arm to position
 * once arm in positon move to next step
 * extend arm
 * drop cone
 * retract arm
 * rotate arm 
 * run intake
 * call drive sub
 * extend arm
 * grip
 * retract arm
 * rotate arm
 * extend
 * limelight drive in
 * drop*/

 double position = m_potentiometer.get()+armoffsetamount; 
 double pidOut = m_armpidController.calculate(position); 
if(m_telescopehome == 1){
  m_armpidController.setSetpoint(kSetpointsMeters[m_index]);
  m_rotate.set(pidOut);
}
 if (autostep == 0 && autonnumber != 5){
  autostep = 1;
 }
// Run the PID Controller


 
 if (autostep == 1 && m_telescopehome==1){
  autostep = 2;
 }

 if (autostep == 2 ){
  m_index = 5;
  if((Math.abs(kSetpointsMeters[5]-position))<.75){autostep=3;
  }
}
  if (autostep == 3){
       m_Telescope.set(ControlMode.Position,kTelescopearray[5]);
   // m_armpidController.setSetpoint(kSetpointsMeters[5]);
   // m_rotate.set(pidOut);
    m_index = 5;
    autostep = 4;
   }

   //was <-44000
  if (autostep == 4 && m_Telescope.getSelectedSensorPosition()<-40000){
    m_index = 5;
    autostep = 5;
   }

   if (autostep == 5){
    m_index = 5;
    m_gripperSolenoid.set(true);
    Timer.delay(.4);
    autostep = 6;
   }

   
   if (autostep == 6){
    m_index = 5;
    m_gripperSolenoid.set(true);
    m_Telescope.set(ControlMode.Position,0);
    autostep = 7;
   }
   if (autostep == 7 && m_Telescope.getSelectedSensorPosition()>-20000){
    //m_index = 3;
    m_gripperSolenoid.set(true);
    m_Telescope.set(ControlMode.Position,0);    
    autostep = 8;
   }

   if (autostep == 8){
    //m_index = 3;
    m_Telescope.set(ControlMode.Position,0);    
    autostep = 9;
   }

   if (autostep == 9){
    autostep = 10;
    m_index = 3;
    autoTimer.restart();
   }

   if(autostep == 9 && (autonnumber == 1 || autonnumber ==3)){
    m_IntakeMotor.set(-.65);
    m_IntakeSolenoid.set(true); 
   }
     
    if (autostep == 10){
    if (m_CENTERAUTO != null) {if (autonnumber == 0){m_CENTERAUTO.schedule();}autostep = 11;}
    if (m_RIGHT2PIECE != null){if (autonnumber == 1){m_RIGHT2PIECE.schedule();}autostep = 11;}
    if (m_RIGHTPLACEBALANCE != null){if (autonnumber == 2){m_RIGHTPLACEBALANCE.schedule();}autostep = 11;}
    if (m_LEFT2PIECE != null){if (autonnumber == 3){m_LEFT2PIECE.schedule();}autostep = 11;}
    if (m_LEFTPLACEBALANCE != null){if (autonnumber == 4){m_LEFTPLACEBALANCE.schedule();}autostep = 11;}
    }
    if (autostep == 11 && autoTimer.get()>.5) {
      m_index = 3;      
    }

    if (autostep == 11 && !m_CENTERAUTO.isScheduled()){autostep = 30;}
    if (autostep == 11 && !m_RIGHT2PIECE.isScheduled()){autostep = 12;m_Telescope.set(ControlMode.Position,-12000);}
    if (autostep == 11 && !m_RIGHTPLACEBALANCE.isScheduled()){autostep = 30;}
    if (autostep == 11 && !m_LEFT2PIECE.isScheduled()){autostep = 12;m_Telescope.set(ControlMode.Position,-12000);}  
    if (autostep == 11 && !m_LEFTPLACEBALANCE.isScheduled()){autostep = 30;}  
    
    
    if (autostep == 12){
      m_Telescope.set(ControlMode.Position,-12000);
      m_IntakeSolenoid.set(false);
      autostep = 13;
       }
    if (autostep == 13 && m_Telescope.getSelectedSensorPosition()<-9000 ){
      m_gripperSolenoid.set(false);
      autoTimer.restart();
      autostep = 14;
      
       }

    if (autostep == 14 && autoTimer.get()>.3){
      m_Telescope.set(ControlMode.Position,0);
      m_IntakeMotor.set(0);
      autostep = 15;
       }   

   if (autostep == 15 && m_Telescope.getSelectedSensorPosition()>-2000){
    m_index = 5; 
    if((Math.abs(kSetpointsMeters[5]-position))<.75){
    autostep=16;
    }
   } 

   if (autostep == 16 ){
    m_Telescope.set(ControlMode.Position,kTelescopearray[5]);
     m_index = 5;
     autostep = 17;
    }

  if (autostep == 17){
    m_Telescope.set(ControlMode.Position,kTelescopearray[5]);
     m_index = 5;
     
     autostep = 18;
    }
    
    if (autostep == 18 && m_Telescope.getSelectedSensorPosition()<-44000){
      m_index = 5;
      m_gripperSolenoid.set(true);
      autostep = 19;
      autoTimer.restart();
     }
     
     if (autostep == 19 && autoTimer.get()>.75){
      m_index = 5;
      m_gripperSolenoid.set(true);
      m_Telescope.set(ControlMode.Position,0);
     }
   
     if (autostep == 30){
      autonautobalance = true;
      autostep = 30;
    } 
   
   SmartDashboard.putNumber("autostep", autostep);

  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.

    m_CENTERAUTO.cancel();
    m_LEFT2PIECE.cancel();
    m_LEFTPLACEBALANCE.cancel();
    m_RIGHT2PIECE.cancel();
    m_RIGHTPLACEBALANCE.cancel();

    isarminmanuel = true;
    homingstep = 0;
    m_telescopehome = 0;
    m_index = 3;
    mrflippystate = false;
    autonautobalance = false;

   // s_swerve.resetModulesToAbsolute();
  }
  
  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

 //steps
 //0 initialize
 //1 compare if already at switch, if yes automatically zero, if not go onto next step
 //2 Run motor backwards until limit switch hit
 //3 stop motor and set motor position to 0
 //4 automatic mode, set motor positions
 //5 manaul mode, run motor by voltage
 //stickDeadband = Constants.stickDeadband;
 /*if(extenedTelescope = true){
  stickDeadband = 0;
  }
  else {
    stickDeadband = 0.05;

  }*/
 brakSolenoid.set(true);
 //SmartDashboard.putNumber("telescope encoder", m_Telescope.getSelectedSensorPosition());
//0
if (homingstep == 0){
  homingstep = 1;
}

//1
if (homingstep == 1 
  && !armhomingswitchbase.get() 
  && (m_potentiometer.get()+armoffsetamount>3.1) 
  && (m_potentiometer.get()+armoffsetamount< 3.6))
  {
  homingstep = 2;}
  else if (homingstep == 1){homingstep = 2;
}

//2
if (homingstep == 2 && !armhomingswitch.get()){
  homingstep = 5;}
  else if (homingstep == 2){homingstep = 3;
}

//3
if (homingstep == 3 && !armhomingswitch.get()){
  homingstep = 5;
}
  else if (homingstep ==3){ 
  m_Telescope.set(.25); 
}

//4
if (homingstep == 4){
  m_Telescope.set(0);
  m_Telescope.setSelectedSensorPosition(-7700);
  m_telescopehome = 1;
  homingstep = 6;
}
//5
if (homingstep == 5){
  m_Telescope.set(0);
  m_Telescope.setSelectedSensorPosition(-5);
  m_telescopehome = 1;
  homingstep = 6;
}

 //6 automated control
  if (homingstep == 6){
   // m_pidController.setReference(telescopecommandposition, CANSparkMax.ControlType.kPosition);
      m_Telescope.set(ControlMode.Position, telescopecommandposition);
      //m_Telescope.set(ControlMode.Position, 0, 0, kDefaultPeriod);(ControlMode.Position, telescopecommandposition);
    }

  //10 manual
  if (homingstep == 10){
  //manual  
  m_telescopehome = 0;
  telescopemanual = (((m_Driver2.getLeftTriggerAxis() + (-1 * m_Driver2.getRightTriggerAxis())) * .5)+.05);
  m_Telescope.set(ControlMode.PercentOutput,telescopemanual);
  }


//re home arm telescope
 if( m_Driver2.getRawButton(7)){
 homingstep = 0;
 m_telescopehome = 0;
 //m_Telescope.set(0);
 //m_encoder.setPosition(0);
 }


//set to manual
 if( m_Driver2.getRawButton(8)){
  homingstep = 10;
 }
 
//clamp game piece
/*if (m_Driver2.getLeftBumperPressed()) {
  if (gripperopen) {
     // Current state is true so turn off
     gripperopen = false;
   
     m_gripperSolenoid.set(false);
  } else {
     // Current state is false so turn on
     gripperopen = true;
     m_gripperSolenoid.set(true);
  }
}
 */
if (m_Driver2.getLeftBumperPressed()) {
  m_gripperSolenoid.set(false);}
if (/*m_Driver2.getRightBumperPressed()||*/m_Flight.getRawButtonPressed(10)||m_Flight.getRawButtonPressed(11)){
  m_gripperSolenoid.set(true);
  m_timer.restart();
}  
if (m_Driver2.getRightBumperPressed()){
  m_gripperSolenoid.set(true);
 // m_timer.restart();
} 


if (m_timer.get() > .6 && m_timer.get() < .65) {
  // Drive forwards half speed, make sure to turn input squaring off
  extenedTelescope = false;
}





//mr flippy
if (m_Flight.getRawButtonPressed(15) && m_Driver2.getRawButton(9)) {
  if (mrflippystate) {
     // Current state is true so turn off
     mrflippystate = false;
     mrflippySolenoid.set(true);
     mrflippySolenoidinvesre.set(false);
  } else {
     // Current state is false so turn on
    mrflippystate = true;
    mrflippySolenoid.set(false);
    mrflippySolenoidinvesre.set(true);
  }
}



//telescope arm
if (m_Driver2.getXButtonPressed() ) {
  if (extenedTelescope) {
     // Current state is true so turn off
     extenedTelescope = false;
  } else {
     // Current state is false so turn on
    extenedTelescope = true;
  }
}
// auto arm home
/*if (m_Driver2.getBButtonPressed() && telescopesafe == false) {
 kSetpointsMeters[m_index] = 4;
 telescopesafe = true;
 
}*/

if(extenedTelescope){
  telescopecommandposition = kTelescopearray[m_index];
  }else {

    telescopecommandposition = -10.0;
  }

 if (Math.abs(kSetpointsMeters[m_index]-m_potentiometer.get()+armoffsetamount)>.4){
    extenedTelescope = false;
    telescopecommandposition = 0.0;
 }
 SmartDashboard.putNumber("arm error",(Math.abs(kSetpointsMeters[m_index]-m_potentiometer.get()+armoffsetamount)));
 SmartDashboard.putNumber("arm command position",kSetpointsMeters[m_index]);

 //SmartDashboard.putNumber("arm position", m_potentiometer.get());
 SmartDashboard.putNumber("homing step", homingstep);
 //}


 
  /*  double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);
*/


 if(m_potentiometer.get() < .75
 || m_potentiometer.get() > 4.8
 || homingstep == 10
 || homingstep != 6){
 isarminmanuel = true;
 }else {isarminmanuel = false;
  }


  double position = m_potentiometer.get()+armoffsetamount;

  // Run the PID Controller
  double pidOut = m_armpidController.calculate(position);
  
if(!isarminmanuel /*&& m_Telescope.getSelectedSensorPosition()!=0*/){
m_armpidController.setSetpoint(kSetpointsMeters[m_index]);
m_rotate.set(pidOut);}
else { if(m_Driver2.getLeftY() <.05 && m_Driver2.getLeftY() > -.05){
  m_rotate.set(0);
}else
m_rotate.set((m_Driver2.getLeftY()*.6));
}
SmartDashboard.putNumber("telescope encoder", m_Telescope.getSelectedSensorPosition());
SmartDashboard.putBoolean("arm in manual", isarminmanuel);
telescopeencoderpos = m_Telescope.getSelectedSensorPosition();
// home
if(m_Driver2.getBButtonPressed()){
  extenedTelescope = false;
}
if(telescopeencoderpos> -1000){
telescopesafe = true;
} else telescopesafe = false; 



  m_IntakeSolenoid.set(m_Flight.getRawButton(kSolenoidButton));

  if (m_Flight.getRawButton(1)){
    reverseintake = true;
  } else {
    reverseintake = false;
  }

    if (m_Flight.getRawButton(kSolenoidButton)) {
      if(reverseintake){
        m_IntakeMotor.set(.6);
        Blinken.set(-.31);
        m_Driver2.setRumble(RumbleType.kBothRumble, 1);
        //m_convayor.set(((1+m_Flight.getRawAxis(6))/2));
      }else{
      m_IntakeMotor.set(-.65); 
      Blinken.set(-.89);
      m_Driver2.setRumble(RumbleType.kBothRumble, 0);
      //m_IntakeMotor.set(-((1+m_Flight.getRawAxis(6))/2)); 
      //m_convayor.set(-((1+m_Flight.getRawAxis(6))/2));
     
      }
    //scaling adjustment so it runs from 0-1, instead of -1 +1
 } else {
    m_IntakeMotor.set(0);
    //m_convayor.set(0);
 }
//TESTING AXIS 6 VALUE
SmartDashboard.putNumber("test joy axis", m_Flight.getRawAxis(5));
    SmartDashboard.putNumber("Intake Speed", -((1+m_Flight.getRawAxis(6))/2));

// prevent from counting if arm is telescoped
if(telescopeencoderpos> -1000){
  if(m_Driver2.getRightStickButtonPressed()){
m_index = 2;
}
if(m_Driver2.getBButton() && telescopesafe == true){
  m_index = 3;
  armrotated = false;
  armmid = false;
  armhigh = false; 
  }
if(m_Driver2.getYButtonPressed()&& m_index < 5){
  m_index = (m_index + 1 );
 }
if(m_Driver2.getAButtonPressed() && m_index > 0){
  m_index = (m_index - 1 );  
 }
}


}//Need to add command for arm to stay in retracted position

 





  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}

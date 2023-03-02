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
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
//import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
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
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static CTREConfigs ctreConfigs;
  
   // Joysticks
  private final Joystick m_Flight = new Joystick(0);
  private final XboxController m_Driver2 = new XboxController(1);

  // solenoids
  private final Solenoid m_IntakeSolenoid = new Solenoid(31,PneumaticsModuleType.REVPH, 1);
  private Solenoid m_gripperSolenoid = new Solenoid(31,PneumaticsModuleType.REVPH, 0);
  private final Solenoid brakSolenoid = new Solenoid(31,PneumaticsModuleType.REVPH, 3 );
 // Motors

// Analog Input
//public AnalogInput m_armAnalogInput = new AnalogInput(0);
//led
private PWMSparkMax Blinken = new PWMSparkMax(0);

 //Digital Inputs
public DigitalInput armhomingswitch = new DigitalInput(0);
public DigitalInput armhomingswitchbase = new DigitalInput(1);
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
private static final int kSolenoidButton = 2;
// Telescope Stuff
public SparkMaxPIDController m_pidController;
private RelativeEncoder m_encoder;
public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
public int m_telescopehome;
public double telescopeencoderpos;
public int homingstep;
public Double telescopecommandposition;
static final double[] kTelescopearray = {-45000,-23000,-13000,-13000,-23000,-45000};
//{-45000,-23000,-22000,-14000,-14000,-22000,-23000,-45000}
private boolean extenedTelescope;
private double telescopemanual;
private boolean telescopesafe;

//Arm Rotation Stuff
static final int kPotChannel = 0;  
static final double kFullHeightMeters = 5;
static final double[] kSetpointsMeters = {1.45,1.62,2.85,2.95,4.25,4.39};
//{1.45,1.53,2.1,2.85,2.98,3.55,4.25,4.39}
private static final double kPa = 1.7;
private static final double kIa = 0.0;
private static final double kDa = 0.0;

public final PIDController m_armpidController = new PIDController(kPa,kIa, kDa);  
private final AnalogPotentiometer m_potentiometer = 
  new AnalogPotentiometer(kPotChannel, 5);
 private int m_index;
private double targetpos = 0;
private  boolean  isarminmanuel;
private boolean armrotated;
private boolean armmid;
private boolean armhigh;

private boolean stopauto;
//private boolean armnotatcommandedpos;

private Integer autoroutine;
public Integer autostep;
private Command m_autonomousCommand;
private Command m_DBFL;
private Command m_DBFR;
private RobotContainer m_robotContainer;

/**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    //s_swerve = new Swerve();
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
    //m_Telescope.configFactoryDefault();
    m_Telescope.setInverted(false);
    m_Telescope.setSensorPhase(true);

    m_Telescope.configNominalOutputForward(0);
		m_Telescope.configNominalOutputReverse(0);
		m_Telescope.configPeakOutputForward(1);
		m_Telescope.configPeakOutputReverse(-1);
    m_Telescope.configAllowableClosedloopError(0,0);
    telescopeencoderpos = 1500;
    		/* Config Position Closed Loop gains in slot0, tsypically kF stays zero. */
		
		m_Telescope.config_kP(0, .4);
		m_Telescope.config_kI(0, 0);
		m_Telescope.config_kD(0, 0);
    m_Telescope.config_kF(0, 0);
    m_rotate = new CANSparkMax(RotateID, MotorType.kBrushless);
    autostep = 0;
    stopauto = false;
    
		
    
  m_robotContainer = new RobotContainer(); 
 
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
   
    //motors
   m_IntakeMotor = new CANSparkMax(IntakeDeviceID, MotorType.kBrushed);
   reverseintake = false;
   //m_convayor = new CANSparkMax(ConvayorDeviceID, MotorType.kBrushed);
   reverseintake = false;
  

   // Resets
   m_IntakeMotor.restoreFactoryDefaults();
   //m_convayor.restoreFactoryDefaults();
   m_rotate.restoreFactoryDefaults(); 
   m_rotate.setIdleMode(IdleMode.kBrake);
   m_rotate.setInverted(true);
   m_rotate.setOpenLoopRampRate(.4);
   
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
 
    SmartDashboard.putBoolean("stop auto?", stopauto);
    SmartDashboard.getBoolean("stop auto?", false);
    if (m_index == 0 || m_index == 5){
      armhigh = true;
      Blinken.set(-.59);
    }
    if (m_index == 4 || m_index == 2){
      armmid = true;
      Blinken.set(-.41);
     }else{
     Blinken.set(-.99);
     }
 

    //Telecope Section
    /*if(m_telescopehome == 0){
      if(m_telescopehome == 0 && m_telescopehomingswitch.get()){
      m_Telescope.set(-.20);} 
      else {m_pidCont
        roller.setReference(telescopecommandposition, CANSparkMax.ControlType.kPosition);
          m_telescopehome = 1;}                
  }*/


  SmartDashboard.putNumber("telescopecommandposition", telescopecommandposition);      
  SmartDashboard.putNumber("m index", m_index);    
  SmartDashboard.putBoolean("arm manual mode", isarminmanuel);
  SmartDashboard.putBoolean("arm homing switch", armhomingswitch.get());
  SmartDashboard.putBoolean("arm homing switch base", armhomingswitchbase.get());  
  SmartDashboard.putNumber("Target Arm POS", targetpos);
  //SmartDashboard.putNumber("calculated arm pos", m_armpidController.calculate(position));
  SmartDashboard.putNumber("arm motor speed", m_rotate.getAppliedOutput());
  
  }

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
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    m_DBFL = m_robotContainer.getDBFL();
    m_DBFR = m_robotContainer.getDBFR();
    homingstep = 0;
    m_telescopehome = 0;
    
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
      && (m_potentiometer.get()>3.1) 
      && (m_potentiometer.get()< 3.6))
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
 double position = m_potentiometer.get(); 
 double pidOut = m_armpidController.calculate(position); 
if(m_telescopehome == 1){
  m_armpidController.setSetpoint(kSetpointsMeters[m_index]);
  m_rotate.set(pidOut);
}
 if (autostep == 0 && !stopauto){
  autostep = 1;
 }
// Run the PID Controller

 
 if (autostep == 1 && m_telescopehome==1){
  autostep = 2;
 }

 if (autostep == 2 ){
  m_index = 5;
  if((Math.abs(kSetpointsMeters[5]-position))<.15){autostep=3;
  }
}
  if (autostep == 3){
       m_Telescope.set(ControlMode.Position,kTelescopearray[5]);
   // m_armpidController.setSetpoint(kSetpointsMeters[5]);
   // m_rotate.set(pidOut);
    m_index = 5;
    autostep = 4;
   }

  if (autostep == 4 && m_Telescope.getSelectedSensorPosition()<-44000){
    m_index = 5;
    autostep = 5;
   }

   if (autostep == 5){
    m_index = 5;
    m_gripperSolenoid.set(true);
    Timer.delay(.7);
    autostep = 6;
   }
   
   if (autostep == 6){
    m_index = 5;
    m_gripperSolenoid.set(true);
    m_Telescope.set(ControlMode.Position,0);
    autostep = 7;
   }
   if (autostep == 7 && m_Telescope.getSelectedSensorPosition()>-1000){
    m_index = 5;
    m_gripperSolenoid.set(true);
    m_Telescope.set(ControlMode.Position,0);    
    autostep = 8;
   }

   if (autostep == 8){
    m_index = 3;
    m_Telescope.set(ControlMode.Position,0);    
    autostep = 9;
   }

   if (autostep == 9 && (Math.abs(kSetpointsMeters[3]-position))<.15){
    //m_IntakeMotor.set(-.5);
    //m_IntakeSolenoid.set(true);
    autostep = 10;
   }
 
    
    if (autostep == 10){
    if (m_autonomousCommand != null) {
      //m_autonomousCommand.schedule();
      m_DBFL.schedule();
      //m_DBFR.schedule();
      autostep = 11;
    }}

    if (autostep == 11 && !m_autonomousCommand.isScheduled()){
      //autostep = 12;
    } 
     
    if (autostep == 12){
      m_Telescope.set(ControlMode.Position,kTelescopearray[3]);
      m_IntakeSolenoid.set(false);
      autostep = 13;
       }
    if (autostep == 13 && m_Telescope.getSelectedSensorPosition()<-13000 ){
      m_gripperSolenoid.set(false);
      Timer.delay(.5);
      autostep = 14;
      m_IntakeMotor.set(0);
       }

    if (autostep == 14){
      m_Telescope.set(ControlMode.Position,0);
      autostep = 15;
       }   

   if (autostep == 15 && m_Telescope.getSelectedSensorPosition()>-1000){
    m_index = 5; 
    if((Math.abs(kSetpointsMeters[5]-position))<.15){
    autostep=16;
    }
   } 

   if (autostep == 16 ){
    //m_Telescope.set(ControlMode.Position,kTelescopearray[5]);
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
      Timer.delay(.7);
      autostep = 19;
     }
     
     if (autostep == 19){
      m_index = 5;
      m_gripperSolenoid.set(true);
      m_Telescope.set(ControlMode.Position,0);
     }
   
   
   SmartDashboard.putNumber("autostep", autostep);
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
      }
    isarminmanuel = true;
    homingstep = 0;
    m_telescopehome = 0;
    m_index = 3;
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

 brakSolenoid.set(true);
 //SmartDashboard.putNumber("telescope encoder", m_Telescope.getSelectedSensorPosition());
//0
if (homingstep == 0){
  homingstep = 1;
}

//1
if (homingstep == 1 
  && !armhomingswitchbase.get() 
  && (m_potentiometer.get()>3.1) 
  && (m_potentiometer.get()< 3.6))
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
  telescopemanual = (((m_Driver2.getLeftTriggerAxis() + (-1 * m_Driver2.getRightTriggerAxis())) * .8)+.05);
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
if (m_Driver2.getRightBumperPressed()){
  m_gripperSolenoid.set(true);
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
    telescopecommandposition = 0.0;
  }

 if (Math.abs(kSetpointsMeters[m_index]-m_potentiometer.get())>.4){
    extenedTelescope = false;
    telescopecommandposition = 0.0;
 }



 SmartDashboard.putNumber("arm position", m_potentiometer.get());
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


 if(m_potentiometer.get() < 1 
 || m_potentiometer.get() > 4.8 
 || homingstep == 10
 || homingstep != 6){
 isarminmanuel = true;
 }else {isarminmanuel = false;
  }


  double position = m_potentiometer.get();

  // Run the PID Controller
  double pidOut = m_armpidController.calculate(position);
  
if(!isarminmanuel){
m_armpidController.setSetpoint(kSetpointsMeters[m_index]);
m_rotate.set(pidOut);}
else { if(m_Driver2.getLeftY() <.1 && m_Driver2.getLeftY() > -.1){
  m_rotate.set(0);
}else
m_rotate.set((m_Driver2.getLeftY()*1));
}

telescopeencoderpos = m_Telescope.getSelectedSensorPosition();
// home
if(m_Driver2.getBButtonPressed()){
  extenedTelescope = false;
}
if(telescopeencoderpos> -1000){
telescopesafe = true;
} else telescopesafe = false; 


  m_IntakeSolenoid.set(m_Flight.getRawButton(kSolenoidButton));

  if (m_Flight.getRawAxis(3)>0){
    reverseintake = true;
  } else {
    reverseintake = false;
  }

    if (m_Flight.getRawButton(kSolenoidButton)) {
      if(reverseintake){
        m_IntakeMotor.set(((1+m_Flight.getRawAxis(6))/2));
        //m_convayor.set(((1+m_Flight.getRawAxis(6))/2));
      }else{
      m_IntakeMotor.set(-((1+m_Flight.getRawAxis(6))/2)); 
      //m_convayor.set(-((1+m_Flight.getRawAxis(6))/2));
     
      }
    //scaling adjustment so it runs from 0-1, instead of -1 +1
 } else {
    m_IntakeMotor.set(0);
    //m_convayor.set(0);
 }

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

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.robot;

//import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.robot.autos.*;
//import frc.robot.autoss.*;
//import frc.robot.robot.commands.autoss.*;
//import frc.robot.commands.*;
import frc.robot.robot.commands.*;
//import frc.robot.robot.Constants;
import frc.robot.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.InstantCommand;



/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Joystick m_flight = new Joystick(0);
    
  /* Drive Controls */
  private final int translationAxis = 2;
  private final int strafeAxis = 5;
  private final int rotationAxis = 0;

  /* m_flight Buttons */
  private final JoystickButton zeroGyro = new JoystickButton(m_flight,3);
  private final JoystickButton robotCentric = new JoystickButton(m_flight, 1);

  private final Swerve s_Swerve = new Swerve();
  //private final boolean robotCentrictogglestate;
  //private final boolean robotCentricstate;








  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
 // private final CommandXboxController m_driverController =
      //new CommandXboxController(1);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    s_Swerve.setDefaultCommand(
      new TeleopSwerve(
          s_Swerve, 
          () -> -m_flight.getRawAxis(translationAxis), 
          () -> -m_flight.getRawAxis(strafeAxis), 
          () -> -m_flight.getRawAxis(rotationAxis)*.6, 
          () -> robotCentric.getAsBoolean()
      )
  );


    // Configure the trigger bindings
    
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));   
        
       /*  if (robotCentric.onTrue()){
          if(robotCentrictogglestate){
            robotCentricstate = true;
            robotCentrictogglestate = false;
          }else {
            robotCentricstate = false;
            robotCentrictogglestate = true;
          }
        }*/

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
   // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    //return exampleAuto.trajectory(m_exampleSubsystem);
    return new exampleAuto(s_Swerve);
  }
  public Command getDBFL() {
    // An example command will be run in autonomous
    //return exampleAuto.trajectory(m_exampleSubsystem);
    return new DBFL(s_Swerve);
  }
  public Command getDBFR() {
    // An example command will be run in autonomous
    //return exampleAuto.trajectory(m_exampleSubsystem);
    return new DBFR(s_Swerve);
  }
}
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Autos;
import frc.robot.commands.drive.Alignments;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.subsystems.drive.VisionSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  CommandXboxController driver = new CommandXboxController(0);

  VisionSubsystem visionSub = new VisionSubsystem();
  SwerveSubsystem swerveSubsystem = new SwerveSubsystem(visionSub);
  RunCommand drive = new RunCommand(()->swerveSubsystem.joystickDrive(driver.getLeftX(),driver.getLeftY(),driver.getRightX()),swerveSubsystem);


  SendableChooser<Command> autoChooser = new SendableChooser<>();



  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {



  //  driver.rightTrigger(0.5).onTrue(ArmCommands.placeCubeL2orL3(arm));
//    driver.leftTrigger(0.5).onTrue(ArmCommands.retractCubeFromL2orL3(arm));

    swerveSubsystem.setDefaultCommand(drive);

//
    driver.b().whileTrue(new RunCommand(swerveSubsystem::autoBalanceForward,swerveSubsystem));

    driver.start().onTrue(new InstantCommand(swerveSubsystem::resetOdometry));

    driver.back().onTrue(new InstantCommand(swerveSubsystem::toggleFieldOriented));

    driver.x().whileTrue(new RunCommand(swerveSubsystem::xConfig,swerveSubsystem));

    driver.a().toggleOnTrue(Alignments.speakerAlignTest);



    // Configure the trigger bindings
    configureBindings();
    createAutos();
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

  private void createAutos(){
    autoChooser.setDefaultOption("no auto :'( ", null);
    autoChooser.addOption("test",Autos.testAuto());

    //autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData(autoChooser);

  }

  private void configureBindings() {
   // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
      return autoChooser.getSelected();
  }
}

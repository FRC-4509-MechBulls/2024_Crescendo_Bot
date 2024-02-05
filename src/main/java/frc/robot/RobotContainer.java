// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
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
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
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

  StateControllerSub stateController = new StateControllerSub(); //this MUST be created before any pathplanner commands

  CommandXboxController driver = new CommandXboxController(0);
  CommandXboxController operator = new CommandXboxController(1);

  VisionSubsystem visionSub = new VisionSubsystem();

  SendableChooser<Command> autoChooser = new SendableChooser<>();


  ArmSubsystem armSubsystem = new ArmSubsystem(stateController);

  ClimbSubsystem climbSubsystem = new ClimbSubsystem(stateController);

  SwerveSubsystem swerveSubsystem = new SwerveSubsystem(visionSub,stateController);

  RunCommand drive = new RunCommand(()->swerveSubsystem.joystickDrive(driver.getLeftX(),driver.getLeftY(),driver.getRightX()),swerveSubsystem);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    swerveSubsystem.setDefaultCommand(drive);

    // Configure the trigger bindings
    configureBindings();
    createAutos();
  }

  private void configureBindings() {
    // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
    driver.start().onTrue(new InstantCommand(swerveSubsystem::resetOdometry));
    driver.back().onTrue(new InstantCommand(swerveSubsystem::toggleFieldOriented));

    operator.a().onTrue(new InstantCommand(stateController::intakePressed));
    operator.b().onTrue(new InstantCommand(stateController::holdPressed));
    operator.x().onTrue(new InstantCommand(stateController::ejectPressed));
    operator.y().onTrue(new InstantCommand(stateController::readyToShootPressed));

    operator.leftBumper().onTrue(new InstantCommand(stateController::stowPressed));
    operator.rightBumper().onTrue(new InstantCommand(stateController::raiseClimbPressed));
    operator.rightTrigger(0.5).onTrue(new InstantCommand(stateController::climbPressed));

    operator.povUp().onTrue(new InstantCommand(stateController::sourcePressed));
    operator.povRight().onTrue(new InstantCommand(stateController::ampPressed));
    operator.povDown().onTrue(new InstantCommand(stateController::speakerPressed));
    operator.povLeft().onTrue(new InstantCommand(stateController::trapPressed));

    operator.leftTrigger(0.5).onTrue(new InstantCommand(stateController::shootPressed));



 // operator.a().onTrue(new InstantCommand(()->stateController.setArmState(StateControllerSub.ArmState.INTAKE)));

  }

  private void createAutos(){
    autoChooser.setDefaultOption("no auto :'( ", null);

    autoChooser.addOption("four-speaker",new PathPlannerAuto("four-speaker"));
    autoChooser.addOption("four-amp",new PathPlannerAuto("four-amp"));
    autoChooser.addOption("two-amp-speaker",new PathPlannerAuto("two-amp-two-speaker"));
   // autoChooser.addOption("test",Autos.testAuto());

    //autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData(autoChooser);

  }


  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
      return autoChooser.getSelected();
  }
}

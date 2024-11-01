// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.fasterxml.jackson.databind.util.Named;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.WaitForArmError;
import frc.robot.commands.drive.Alignments;
import frc.robot.commands.drive.NoteCreep;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.EFSubsystem;
import frc.robot.subsystems.PneumaticControlSub;
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
  CommandXboxController operator = new CommandXboxController(1);

  StateControllerSub stateController = new StateControllerSub(driver,operator); //this MUST be created before any pathplanner commands

  PneumaticControlSub pneumaticControlSub = new PneumaticControlSub(stateController);


  VisionSubsystem visionSub = new VisionSubsystem(stateController);

  SendableChooser<Command> autoChooser = new SendableChooser<>();


  ArmSubsystem armSubsystem = new ArmSubsystem(stateController,pneumaticControlSub);

  EFSubsystem efSub = new EFSubsystem(stateController);

  ClimbSubsystem climbSubsystem = new ClimbSubsystem(stateController,pneumaticControlSub);

  SwerveSubsystem swerveSubsystem = new SwerveSubsystem(visionSub,stateController);



  RunCommand drive = new RunCommand(()->swerveSubsystem.joystickDrive(driver.getLeftX(),driver.getLeftY(),driver.getRightX()),swerveSubsystem);
  InstantCommand turnRight = new InstantCommand(()->swerveSubsystem.joystickDrive(1,0,0), swerveSubsystem);
  InstantCommand turnUp = new InstantCommand(()->swerveSubsystem.joystickDrive(0,1,0), swerveSubsystem);
  InstantCommand turnDown = new InstantCommand(()->swerveSubsystem.joystickDrive(0,-1,0), swerveSubsystem);
  InstantCommand turnLeft = new InstantCommand(()->swerveSubsystem.joystickDrive(-1,0,0), swerveSubsystem);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    swerveSubsystem.setDefaultCommand(drive);

    NamedCommands.registerCommand("joystick0", new RunCommand(()->swerveSubsystem.joystickDrive(0,0,0),swerveSubsystem));
    NamedCommands.registerCommand("waitForArmError", new WaitForArmError(armSubsystem, Units.degreesToRadians(2),0.25));

    // Configure the trigger bindings
    configureBindings();
    createAutos();
  }

  private void configureBindings() {
    // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
    driver.start().onTrue(new InstantCommand(swerveSubsystem::resetOdometry));
    driver.back().onTrue(new InstantCommand(swerveSubsystem::toggleFieldOriented));

    driver.povUp().onTrue(turnUp);
    driver.povDown().onTrue(turnDown);
    driver.povLeft().onTrue(turnLeft);
    driver.povRight().onTrue(turnRight);

    driver.y().onTrue(new InstantCommand(stateController::raiseClimbPressed));
    driver.x().onTrue(new InstantCommand(stateController::climbPressed));
    driver.b().onTrue(new InstantCommand(stateController::resetPressed));

    driver.leftBumper().onTrue(new InstantCommand(()->stateController.setDuckMode(false)));
    driver.leftBumper().onFalse(new InstantCommand(()->stateController.setDuckMode(true)));

    driver.rightBumper().onTrue(new InstantCommand(stateController::scheduleAlignmentCommand,swerveSubsystem));
    driver.rightBumper().onFalse(swerveSubsystem.getDefaultCommand());

    operator.povUp().onTrue(new InstantCommand(stateController::sourcePressed));
    operator.povRight().onTrue(new InstantCommand(stateController::ampPressed));
    operator.povDown().onTrue(new InstantCommand(stateController::speakerPressed));

    operator.a().onTrue(new InstantCommand(stateController::intakePressed));
    operator.b().onTrue(new InstantCommand(stateController::holdPressed));
    operator.x().onTrue(new InstantCommand(stateController::ejectPressed));
    operator.y().onTrue(new InstantCommand(stateController::readyToShootPressed));

    operator.leftBumper().onTrue(new InstantCommand(stateController::resetPressed));
    operator.rightBumper().onTrue(new InstantCommand(()->stateController.setSpeakerBase(!stateController.getSpeakerBase())));

    operator.leftTrigger(0.5).onTrue(new InstantCommand(()->stateController.shootPressed()));

  //  driver.a().onTrue(new InstantCommand(stateController::intakePressed));
  //  driver.b().onTrue(new InstantCommand(stateController::holdPressed));
  //  driver.x().onTrue(new InstantCommand(stateController::ejectPressed));
  //  driver.y().onTrue(new InstantCommand(stateController::readyToShootPressed));

   // driver.a().onTrue(new InstantCommand(stateController::stowClimbPressed));



   // operator.leftBumper().onTrue(new InstantCommand(stateController::stowPressed));
   // operator.rightBumper().onTrue(new InstantCommand(stateController::raiseClimbPressed));
   // operator.rightTrigger(0.5).onTrue(new InstantCommand(stateController::climbPressed));



    //operator.povLeft().onTrue(new InstantCommand(stateController::trapPressed));

   // operator.leftTrigger(0.5).onTrue(new InstantCommand(stateController::shootPressed));
    

   // driver.rightBumper().onTrue(new InstantCommand(stateController::toggleClimbed));
  



   // driver.rightTrigger(0.5).onTrue(swerveSubsystem.getDefaultCommand());




  //  driver.povUp().onTrue(new InstantCommand(()->Alignments.trapTest(stateController, Rotation2d.fromRotations(0)).schedule()));
  //  driver.povLeft().onTrue(new InstantCommand(()->Alignments.trapTest(stateController, Rotation2d.fromRotations(1.0/3)).schedule()));
  //  driver.povRight().onTrue(new InstantCommand(()->Alignments.trapTest(stateController, Rotation2d.fromRotations(-1.0/3)).schedule()));


   //TODO driver.x().onTrue(new InstantCommand(stateController::shootPressed));


   // driver.leftTrigger(0.5).onTrue(new InstantCommand(()->stateController.setDuckMode(true)));
    //driver.leftTrigger(0.5).onFalse(new InstantCommand(()->stateController.setDuckMode(false)));


   // driver.rightTrigger(0.5).onTrue(new InstantCommand(()->stateController.setDuckMode(true)));
   // driver.rightTrigger(0.5).onFalse(new InstantCommand(()->stateController.setDuckMode(false)));

   //TODO driver.a().onTrue(new InstantCommand(stateController::toggleAlignWhenClose));

  //  driver.x().whileTrue(efSub.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
  //  driver.y().whileTrue(efSub.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
  //  driver.a().whileTrue(efSub.sysIdDynamic(SysIdRoutine.Direction.kForward));
  //  driver.b().whileTrue(efSub.sysIdDynamic(SysIdRoutine.Direction.kReverse));



 // operator.a().onTrue(new InstantCommand(()->stateController.setArmState(StateControllerSub.ArmState.INTAKE)));

  }

  private void createAutos(){
    autoChooser.setDefaultOption("no auto :'( ", null);

    autoChooser.addOption("CS-PL-01-00-02",new PathPlannerAuto("CS-PL-01-00-02"));
    autoChooser.addOption("SS-PL-12",new PathPlannerAuto("SS-PL-12"));
    autoChooser.addOption("CS-PL-01-00-02-12",new PathPlannerAuto("CS-PL-01-00-02-12"));
    autoChooser.addOption("AS-PL-00",new PathPlannerAuto("AS-PL-00"));
    autoChooser.addOption("SS-PL",new PathPlannerAuto("SS-PL"));
    autoChooser.addOption("wipe",new PathPlannerAuto("wipe"));
    autoChooser.addOption("CS-PL-01-02",new PathPlannerAuto("CS-PL-01-02"));




    //autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("autoChooser",autoChooser);

  }


  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
      return autoChooser.getSelected();
  }
}

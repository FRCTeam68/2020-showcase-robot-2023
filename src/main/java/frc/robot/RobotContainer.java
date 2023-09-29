/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;
import frc.robot.commands.ChangeIntakePos;
import frc.robot.commands.LiftAdd1Deg;
import frc.robot.commands.LiftMinus1Deg;
import frc.robot.commands.SetAgitator;
import frc.robot.commands.ShiftGears;
import frc.robot.commands.ShootLow;
import frc.robot.commands.ShootMedium;
import frc.robot.commands.SpinFeeder;
import frc.robot.commands.Zero;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.2
 */
// import frc.robot.commands.DriveWithJoysticks;
// import frc.robot.subsystems.DriveTrain;
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  XboxController xboxManipulator;

  JoystickButton xboxManipX;
  JoystickButton xboxManipCircle;
  JoystickButton xboxManipRS;
  JoystickButton xboxManipSquare;
  JoystickButton xboxManipTriangle;
  JoystickButton xboxManipLT;
  JoystickButton xboxManipRT;


  //Command shiftGear = new ShiftGears();

  private static RobotContainer robotContainer;

  public static RobotContainer getRobotContainer() {
    if (robotContainer == null) {
      robotContainer = new RobotContainer();
    }
    return robotContainer;
  }
  public Command getAutonomousCommand() {
    return null;
    //return new RunAutonStraightMeter(Robot.robotOdemetry, Robot.driveTrain);
  }
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    //xboxDriveRT.whenPressed(new ShiftGears());
    xboxManipX.whileHeld(new ShootLow());
    xboxManipCircle.whileHeld(new ShootMedium());
    xboxManipTriangle.whileHeld(new ShootLow());
    xboxManipRS.whenPressed(new ChangeIntakePos());
    xboxManipSquare.whileHeld(new Zero());
    //xboxManipX.whenReleased(new Zero());
    xboxManipCircle.whenReleased(new Zero());
    xboxManipTriangle.whenReleased(new Zero());
    //xboxManipLT.whenPressed(new LiftMinus1Deg());
    xboxManipRT.whileHeld(new SpinFeeder(.75));
    xboxManipRT.whenReleased(new SpinFeeder(0));

  }

  /** 
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    xboxManipulator = new XboxController(Constants.XBOX_MANIPULATE);

    xboxManipX = new JoystickButton(xboxManipulator, Constants.XBOX_MANIPULATE_X);
    xboxManipCircle = new JoystickButton(xboxManipulator, Constants.XBOX_MANIPULATE_CIRCLE);
    xboxManipRS = new JoystickButton(xboxManipulator, Constants.XBOX_MANIPULATE_SR);
    xboxManipSquare = new JoystickButton(xboxManipulator, Constants.XBOX_MANIPULATE_SQUARE);
    xboxManipTriangle = new JoystickButton(xboxManipulator, Constants.XBOX_MANIPULATE_TRIANGLE);
    xboxManipLT = new JoystickButton(xboxManipulator, Constants.XBOX_MANIPULATE_LT);
    xboxManipRT = new JoystickButton(xboxManipulator, Constants.XBOX_MANIPULATE_RT);
    

  }

  public double getLeftXboxJoystickValue() {
    double leftAxis;
    leftAxis = xboxManipulator.getLeftY();

    // Allow for up to 10% of joystick noises\
    leftAxis = (Math.abs(leftAxis) < 0.1) ? 0 : leftAxis;
    return leftAxis;
  }

  // Drivetrain Tank Drive Right
  public double getRightXboxJoystickValue() {
    double rightAxis;
    rightAxis = xboxManipulator.getRightY();
    // Allow for up to 10% of joystick noise
    rightAxis = (Math.abs(rightAxis) < 0.1) ? 0 : rightAxis;
    return rightAxis;
  }
  public double getRightXboxJoystickValueX() {
    double rightAxis;
    
    rightAxis = xboxManipulator.getRawAxis(4);
    
    // Allow for up to 10% of joystick noise
    rightAxis = (Math.abs(rightAxis) < 0.1) ? 0 : rightAxis;
    return rightAxis;
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @throws IOException
   */

}

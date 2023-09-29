/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.SetWhinchSpeed;

//import edu.wpi.first.wpilibj.Servo;
public class EndGame extends SubsystemBase {
  /**
   * Creates a new EndGame.
   */
  //private Servo releaseEndGame;

  public EndGame() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    CommandScheduler.getInstance().setDefaultCommand(Robot.endGame, new SetWhinchSpeed());
  }
  public void setWhinchSpeed(double speed){
  }
}

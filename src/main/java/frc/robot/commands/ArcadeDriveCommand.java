// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.DriveSubsystem;

public class ArcadeDriveCommand extends Command{
  private final DriveSubsystem m_drive;
  private final CommandXboxController m_controller;
  public ArcadeDriveCommand(DriveSubsystem drive, CommandXboxController controller){
    m_drive = drive;
    m_controller = controller;
    addRequirements(m_drive);
  }

  @Override
  public void initialize(){

  }
  @Override
  public void execute(){
    double fwd =-m_controller.getLeftY();
    double rot = -m_controller.getLeftX();
    m_drive.arcadeDrive(fwd, rot);
  }
 @Override
  public void end(boolean interrupted){

  }

  @Override
  public boolean isFinished(){
    return false;
  }
}
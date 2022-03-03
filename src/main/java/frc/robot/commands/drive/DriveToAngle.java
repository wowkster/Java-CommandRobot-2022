package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.MathUtil;

import frc.robot.Robot;

public class DriveToAngle extends CommandBase{
    
	//Declaring varibles
	private double angle;
    private final double kP = 0.015; 
    private final double kI = 0;
    private final double kD = 0;
    private double speed;
    private PIDController pidcontroller;
    public static double error; 
    
    //Takes a varible of what angle you want to turn to
    public DriveToAngle(double angle) {
        //Initialize varibles
        this.angle = angle;
        pidcontroller = new PIDController(kP, kI, kD);
        pidcontroller.setSetpoint(angle);
        pidcontroller.setTolerance(3);
        addRequirements(Robot.drivebase);
    }

    @Override
    public void initialize() {
        Robot.drivebase.resetEncoders();
        Robot.gyro.reset();
    }

    @Override
    public void execute() {
        

        // Max trun speed of 0.8 number not tested | negative angle beacsue Gyro is reversed 
        speed = MathUtil.clamp(pidcontroller.calculate(-Robot.gyro.getAngle()), -0.8, 0.8);

        //(kP * error), -(kP * error)
        Robot.drivebase.drive(speed, -speed);;
        

    }

    @Override
    public void end(boolean interrupted){
    }

    @Override
    public boolean isFinished() {
        //check if you are at the angle and returns that value
        return pidcontroller.atSetpoint();
    }

}
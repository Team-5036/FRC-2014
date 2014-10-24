/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
/*
_________ _______  _______  _______    _______  _______  ______    ______
\__   __/(  ____ \(  ___  )(       )  (  ____ \(  __   )/ ___  \  / ____ \                
   ) (   | (    \/| (   ) || () () |  | (    \/| (  )  |\/   \  \( (    \/                
   | |   | (__    | (___) || || || |  | (____  | | /   |   ___) /| (____                  
   | |   |  __)   |  ___  || |(_)| |  (_____ \ | (/ /) |  (___ ( |  ___ \                 
   | |   | (      | (   ) || |   | |        ) )|   / | |      ) \| (   ) )                
   | |   | (____/\| )   ( || )   ( |  /\____) )|  (__) |/\___/  /( (___) )                
   )_(   (_______/|/     \||/     \|  \______/ (_______)\______/  \_____/                 
                                                                                          
 _______  _______  ______   _______  ______   _______          _________ _        _______ 
(  ____ )(  ___  )(  ___ \ (  ___  )(  __  \ (  ____ \|\     /|\__   __/( \      (  ____ \
| (    )|| (   ) || (   ) )| (   ) || (  \  )| (    \/| )   ( |   ) (   | (      | (    \/
| (____)|| |   | || (__/ / | |   | || |   ) || (__    | |   | |   | |   | |      | (_____ 
|     __)| |   | ||  __ (  | |   | || |   | ||  __)   ( (   ) )   | |   | |      (_____  )
| (\ (   | |   | || (  \ \ | |   | || |   ) || (       \ \_/ /    | |   | |            ) |
| ) \ \__| (___) || )___) )| (___) || (__/  )| (____/\  \   /  ___) (___| (____/\/\____) |
|/   \__/(_______)|/ \___/ (_______)(______/ (_______/   \_/   \_______/(_______/\_______)

*/
package edu.wpi.first.wpilibj.Cerebellum;
/**
*@author Team 5036 - RoboDevils
*/

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.Timer;
import DevilVision.DevilVisionServer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Cerebellum extends IterativeRobot {
    
//                              STATE THE CONSTANTS
    /**Compressor PWM port on Relay Ports*/
    final int COMPRESSOR_RELAY_PORT = 2;                                        
    /**Pressure Switch(pneumatics) on Digital I/O*/
    final int PRESSURE_SWITCH_PORT = 14; 
    /**Air Compressor Start button*/
    final int AIRCOMPRESSOR_STOP = 9;
    /**AirCompresspor Stop Button*/
    final int AIRCOMPRESSOR_START = 10;
    /**Joystick 1*/
    final int JOYSTICK1 = 1;          
    /**Joystick - Kaj Left Axis*/
    final int KAJ_LEFT_AXIS = 2;
    /**Joystick - Kaj Right Axis*/
    final int KAJ_RIGHT_AXIS = 3;
    /**Front Left Victor PWM Port on PWM OUT*/
    final int FRONT_LEFT_SC = 1;
    /**Front Right Victor PWM Port on PWM OUT*/
    final int FRONT_RIGHT_SC = 2;
    /**Back Left Victor PWM Port on PWM OUT*/
    final int BACK_LEFT_SC = 3; 
    /**Back Right Victor PWM Port on PWM OUT*/
    final int BACK_RIGHT_SC = 4;
    /** DriveTrain Orientation Button1 (switches front and back)*/
    final int DRIVETRAIN_ORIENTATION_SWITCH1 = 11;
    /** DriveTrain Orientation Button2 (switches front and back)*/
    final int DRIVETRAIN_ORIENTATION_SWITCH2 = 12;
    /**Double Solenoid Port on Solenoid Breakout Channel(Blocker)*/
    final int SOLENOID_1_EXTRACT = 1;
    /**Double Solenoid Port on Solenoid Breakout Channel(Blocker)*/
    final int SOLENOID_1_RETRACT = 2;
    /**Double Solenoid Port on Solenoid Breakout Channel(Shooter)*/
    final int SOLENOID_2_EXTRACT = 3;
    /**Double Solenoid Port on Solenoid Breakout Channel(Shooter)*/
    final int SOLENOID_2_RETRACT = 4;
    /**Double Solenoid Port on Solenoid Breakout Channel(Gates)*/
    final int SOLENOID_3_EXTRACT = 5;
    /**Double Solenoid Port on Solenoid Breakout Channel(Gates)*/
    final int SOLENOID_3_RETRACT = 6;
    /**Joystick Button for Piston1 (Blocker)*/
    final int PISTON1_EXTRACT_BUTTON = 3;
    /**Joystick Button for Piston1 (Blocker)*/
    final int PISTON1_RETRACT_BUTTON = 4;
    /**Joystick Button for Piston1 (Shooter)*/
    final int PISTON2_EXTRACT_BUTTON = 2;
    /**Joystick Button for Piston1 (Gates)*/
    final int PISTON3_EXTRACT_BUTTON = 5;
    /**Joystick Button for Piston1 (Gates)*/
    final int PISTON3_RETRACT_BUTTON = 6;
    
//                              STATE THE VARIABLES
    /**Timer for delays to prevent lag that comes from the code looping too much and Autonomous Actions*/
    Timer timer;
    /**Air Compressor for Pneumatic Pressure*/
    Compressor airCompressor;
    /**Joystick for receiving input from driver*/
    Joystick driveStick;
    /**RobotDrive is a preprogrammed class that automatically inverses left side from the right side*/
    RobotDrive robotDrive;
    /**Speed Controller for CIM*/
    Victor frontLeftSC;
    /**Speed Controller for CIM*/
    Victor frontRightSC;
    /**Speed Controller for CIM*/
    Victor backLeftSC;
    /**Speed Controller for CIM*/
    Victor backRightSC;
    /**DoubleSolenoid is a valve that is responsible for two positions open or closed(Blocker)*/
    DoubleSolenoid piston1;
    /**DoubleSolenoid is a valve that is responsible for two positions open or closed(Shooter)*/
    DoubleSolenoid piston2;
    /**DoubleSolenoid is a valve that is responsible for two positions open or closed(Gates)*/
    DoubleSolenoid piston3;

    /**
     * This is a function that is defined and programmed in another package,
     * It is used for the ability to autonomously control the robot without 
     * touching touching the joystick using the computer's Web-camera
     */
    public static DevilVisionServer devilVision;
    /**This is a boolean variable that is used to invert the drive train*/
    boolean inter = false;

public Cerebellum() {
//                      DEFINE THE VARIABLES
    this.timer = new Timer();
    this.airCompressor = new Compressor(PRESSURE_SWITCH_PORT, PRESSURE_SWITCH_PORT);
    this.driveStick = new Joystick(JOYSTICK1);
    /**Using the Victors in the RobotDrive class allows you to use both victors and RobotDrive*/
    this.robotDrive = new RobotDrive(frontLeftSC, backLeftSC, frontRightSC, backRightSC);
    this.frontLeftSC = new Victor(FRONT_LEFT_SC);
    this.frontRightSC = new Victor(FRONT_RIGHT_SC);
    this.backLeftSC = new Victor(BACK_LEFT_SC);
    this.backRightSC = new Victor(BACK_RIGHT_SC);
    this.piston1 = new DoubleSolenoid(SOLENOID_1_EXTRACT, SOLENOID_1_RETRACT);
    this.piston2 = new DoubleSolenoid(SOLENOID_2_EXTRACT, SOLENOID_2_RETRACT);
    this.piston3 = new DoubleSolenoid(SOLENOID_3_EXTRACT, SOLENOID_3_RETRACT);
    
    Cerebellum.devilVision = DevilVisionServer.getInstance();
}//Cerebellum End Bracket

/**
* This function is run when the robot is first started up and should be
* used for any initialization code.
*/
public void robotInit() {  
    /**
     * Watchdog is an important safety device that automatically disables the 
     * robot if it does not receive input*/
    getWatchdog () .setEnabled(true);
    getWatchdog() .setExpiration(0.01);
    airCompressor.start();
    devilVision.start();
    robotDrive.setExpiration(0.1);
    
robotDrive.drive(0,0);    
System.out.println("RobotInit Completed");
}//RobotInit End Bracket

/**
 * This function is called periodically every time the robot is disabled
 */
public void disabledPeriodic() {
    getWatchdog() .feed();
    while (isDisabled()) {
        getWatchdog() .feed();
        Timer.delay(0.003);
        airCompressor.start();
        robotDrive.drive(0,0);
    }//Disabled While Statement End Bracket
robotDrive.drive(0,0);
if (isAutonomous()) System.out.println("AutoDisabled");
if (isOperatorControl()) System.out.println("TeleopDisabled");
if (isTest()) System.out.println("TestDisabled");
}//Disabled End Bracket

/**
 * This function is called periodically during autonomous
 */
public void autonomousPeriodic() {
    getWatchdog() .feed();
    while(isEnabled() && isAutonomous()) {
        getWatchdog() .feed();
        Timer.delay(0.003);
//                      DEVILVISION BLOCKER AUTONOMOUS
        robotDrive.drive(0, 0);
        piston1.set(DoubleSolenoid.Value.kForward);
        if(devilVision.getLeftStatus() && !devilVision.getRightStatus()) {
            robotDrive.drive(1,0);
        }
        else if(!devilVision.getLeftStatus() && devilVision.getRightStatus()) {
            robotDrive.drive(-1,0);
        }
        else if(devilVision.getLeftStatus() && devilVision.getRightStatus()) {
            piston1.set(DoubleSolenoid.Value.kReverse);
        }
        else{
            robotDrive.drive(0,0);
        }
//                      DEVILVISON LOW GOAL AUTONOMOUS
/*
        robotDrive.drive(0, 0);
        if(devilVision.getLeftStatus() && !devilVision.getRightStatus()) {
            robotDrive.drive(0.3,0);
	    piston3.set(DoubleSolenoid.Value.kForward);
        }
        else if(!devilVision.getLeftStatus() && devilVision.getRightStatus()) {
            robotDrive.drive(0,0);
	    piston2.set(DoubleSolenoid.Value.kForward)
        }
        else if(devilVision.getLeftStatus() && devilVision.getRightStatus()) {
            robotDrive.drive(-0.5,0);
        }
        else
            robotDrive.drive(-1,0);
        }
    */
    }//Autonomous While Statement End Bracket
robotDrive.drive(0,0);
System.out.println("Goliath autonomousPeriodic() completed.");    
}//Autonomous End Bracket

/**
 * This function is called periodically during operator control
 */
public void teleopPeriodic() {
    getWatchdog() .feed();
    while(isEnabled() && isOperatorControl()) {
        getWatchdog() .feed();
        Timer.delay(0.003);
//                      ROBOT MOBILITY - ROBOTDRIVE
	double kajLeftAxis = (driveStick.getRawAxis(KAJ_LEFT_AXIS));
    		/**Get joystick Y-axis and multiply it with x^3 function. Responsible for forward and backward movement*/
        	double kajLeft = 0.6*((kajLeftAxis)*(kajLeftAxis)*(kajLeftAxis)) + 0.4*(kajLeftAxis);
    	double kajRightAxis = (driveStick.getRawAxis(KAJ_RIGHT_AXIS));
    		/**Get joystick Z-axis and multiply it with x^3 function. Responsible for all turning movement*/
        	double kajRight = 0.6*((kajRightAxis)*(kajRightAxis)*(kajRightAxis)) + 0.4*(kajRightAxis);
        robotDrive.arcadeDrive(-kajLeft, -kajRight);    
        /*if(inter == false){
            if((driveStick.getRawButton(DRIVETRAIN_ORIENTATION_SWITCH1))||(driveStick.getRawButton(DRIVETRAIN_ORIENTATION_SWITCH2))){
                    inter = true;
                    robotDrive.arcadeDrive(kajLeft, kajRight);
            }
        }
        else if(inter == true) {
            if((driveStick.getRawButton(DRIVETRAIN_ORIENTATION_SWITCH1))||(driveStick.getRawButton(DRIVETRAIN_ORIENTATION_SWITCH2))){
                    inter =false;
                    robotDrive.arcadeDrive(-kajLeft, -kajRight);
            }
        }*/
//                      ROBOT MOBILITY - VICTORS
//                      ROBOT PNUEMATICS - COMPRESSOR
        if(driveStick.getRawButton(AIRCOMPRESSOR_STOP)){
            airCompressor.stop();
        }
        else if(driveStick.getRawButton(AIRCOMPRESSOR_START)){
            airCompressor.start();
        }
//                      ROBOT PNUEMATICS - PISTON1
        if(driveStick.getRawButton(PISTON1_EXTRACT_BUTTON)){
            piston1.set(DoubleSolenoid.Value.kForward);
        }
        else if(driveStick.getRawButton(PISTON1_RETRACT_BUTTON)){
            piston1.set(DoubleSolenoid.Value.kReverse);
        }
//                      ROBOT PNUEMATICS - PISTON2
        if(driveStick.getRawButton(PISTON2_EXTRACT_BUTTON)){
            piston2.set(DoubleSolenoid.Value.kForward);
        }
        else{
            piston2.set(DoubleSolenoid.Value.kReverse);
        }
//                      ROBOT PNUEMATICS - PISTON 3
        if(driveStick.getRawButton(PISTON3_EXTRACT_BUTTON)){
            piston3.set(DoubleSolenoid.Value.kForward);
        }
        else if(driveStick.getRawButton(PISTON3_EXTRACT_BUTTON)){
            piston3.set(DoubleSolenoid.Value.kReverse);
        }
    }//Teleop While Statement End Bracket
robotDrive.drive(0,0);
System.out.println("Goliath teleopPeriodic() completed.");
}//Teleop End Bracket
    
/**
* This function is called periodically during test mode
*/
public void testPeriodic() {
    getWatchdog() .setEnabled(false);
    while (isEnabled() && isTest()){
        LiveWindow.setEnabled(true);
    }//Test While Statement End Bracket
robotDrive.drive(0,0);
System.out.println("Goliath testPeriodic() completed.");
}//Test End Bracket


}//Interative Robot End Bracket

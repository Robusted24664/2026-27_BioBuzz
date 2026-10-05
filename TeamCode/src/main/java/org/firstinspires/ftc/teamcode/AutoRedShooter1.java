package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.RoRobot.DriveParameters;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled

@Autonomous(name = "Red Shooter 1", group = "auto")
public class AutoRedShooter1 extends LinearOpMode {
 public RoRobot robot = new RoRobot();
    
    @Override
    public void runOpMode() throws InterruptedException {
        robot.initializeAuto(hardwareMap, (LinearOpMode)this);

        DriveParameters slowForward = robot.getDriveParametersForSlowForward();
        DriveParameters mediumForward = robot.getDriveParametersForMediumForward();
        DriveParameters fastForward = robot.getDriveParametersForFastForward();
        
        robot.setRed();

        
        while (opModeInInit()) {
            robot.updatePose();
            robot.reportPositionData();
        }
        
        waitForStart();
        double start_time = getRuntime();
        
        double velocityMultiplier = 6000.0*0.85*28.0/60.0;
        double targetVelocity = 1060;
        double targetPower = targetVelocity/velocityMultiplier;

        
        robot.flywheelSetPower(targetPower);
        robot.firingStateMaintain();
        robot.hoodPos(0.8346);
        

        
        robot.setBrakes();
        
        robot.strafeToPoseAndStopD(33.2,36.8,42.6, mediumForward);
        
        while (opModeIsActive()) {
            double flywheel_velocity = robot.flywheel.getVelocity();
            telemetry.addData("Flywheel Velocity:", flywheel_velocity);
            telemetry.addData("target flywheel velocity:", targetVelocity);
            telemetry.addData("Time:", getRuntime() - start_time);

            telemetry.update();
            if (flywheel_velocity > targetVelocity) {
                break;
                
            }
        }
        
        robot.flywheelBoost();
        robot.firingStateFire();
        
        sleep (1500);
        
        //collect first line
        robot.flywheelSetPower(targetPower);

        robot.firingStateIntake();
        robot.hoodPos(0.8346);

        robot.strafeToPoseAndStopD(30.75,13.40,0, fastForward); // allign first line
        robot.strafeToPoseAndStopD(54.5,13.40,0, mediumForward); // intake 3 ball
        robot.strafeToPoseAndStopD(35.0,13.40,90, fastForward); // move back, turn 90 deg
        
        robot.strafeToPoseAndStopD(35.0,30.0,90, fastForward); // move close to fire position
        robot.strafeToPoseAndStopD(33.2,36.8,42.6, slowForward); // move to fire pos
        
        robot.flywheelBoost();
        robot.firingStateFire();
        sleep (1500);
        
        //collect second line

        robot.flywheelSetPower(targetPower);

        robot.firingStateIntake();
        robot.hoodPos(0.8346);

        robot.strafeToPoseAndStopD(30.43,-9.64,0, fastForward); // allign second line
        robot.strafeToPoseAndStopD(61.25,-9.64,0, mediumForward); // intake 3 ball
        robot.strafeToPoseAndStopD(35.0,-9.64,90.0, fastForward); // move back, turn 90 deg
        
        robot.strafeToPoseAndStopD(35.0,30.0,90, fastForward); // move close to fire position
        robot.strafeToPoseAndStopD(33.2,36.8,42.6, slowForward); // move to fire pos
        
        robot.flywheelBoost();
        robot.firingStateFire();
        sleep (1500);
        
        //collect third line

        robot.flywheelSetPower(targetPower);

        robot.firingStateIntake();
        robot.hoodPos(0.8346);

        robot.strafeToPoseAndStopD(29.7,-34.8,0, fastForward); // allign third line
        robot.strafeToPoseAndStopD(61.25,-34.8,0, mediumForward); // intake 3 ball
        robot.strafeToPoseAndStopD(35.0,-34.8,90.0, fastForward); // move back, turn 90 deg
        
        
        robot.strafeToPoseAndStopD(35.0,30.0,90, fastForward); // move close to fire position
        robot.strafeToPoseAndStopD(33.2,36.8,42.6, slowForward); // move to fire pos
        
        robot.flywheelBoost();
        robot.firingStateFire();
        sleep (1500);

    }
}

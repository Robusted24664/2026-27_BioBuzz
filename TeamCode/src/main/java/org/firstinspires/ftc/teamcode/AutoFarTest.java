package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.RoRobot.DriveParameters;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled

@Autonomous(name = "Auto Far Shooter 1 Strafe", group = "auto")
public class AutoFarTest extends LinearOpMode {
    public RoRobot robot = new RoRobot();
 
    public double start_time = 0;

 
    public void waitForFlywheelVelocity(double targetVelocity) {
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
    }
    
    public void fireSequence(double targetVelocity, double targetPower, double waitingPower) {
        robot.flywheelSetPower(targetPower);
        waitForFlywheelVelocity(targetVelocity);
        robot.flywheelBoost();
        robot.firingStateFire();
        sleep (1500);
        robot.flywheelSetPower(waitingPower);
    }
    
    @Override
    public void runOpMode() throws InterruptedException {
        robot.initializeAuto(hardwareMap, (LinearOpMode)this);

        DriveParameters slowForward = robot.getDriveParametersForSlowForward();
        DriveParameters mediumForward = robot.getDriveParametersForMediumForward();
        DriveParameters fastForward = robot.getDriveParametersForFastForward();
        DriveParameters mediumFastForward = robot.getDriveParametersForMediumFastForward();

        
        robot.setRed();

        
        while (opModeInInit()) {
            robot.updatePose();
            robot.reportPositionData();
        }
        
        waitForStart();
        start_time = getRuntime();
        
        double velocityMultiplier = 6000.0*0.85*28.0/60.0;
        double targetVelocity = 1250;
        double targetPower = targetVelocity/velocityMultiplier;
        
        double waitingPower = targetPower*0.9;

        
        robot.flywheelSetPower(targetPower);
        robot.firingStateMaintain();
        robot.hoodPos(0.3346);
        

        
        robot.setBrakes();
        
        
        fireSequence(targetVelocity, targetPower, waitingPower);
        
        //collect first line


    }
}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.RoRobot.DriveParameters;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled

@Autonomous(name = "Red Shooter 1 inner", group = "auto")
public class AutoRedShooter1inner extends LinearOpMode {
 public RoRobot robot = new RoRobot();
    
    @Override
    public void runOpMode() throws InterruptedException {
        robot.initializeAuto(hardwareMap, (LinearOpMode)this);

        DriveParameters slowForward = robot.getDriveParametersForSlowForward();
        DriveParameters mediumForward = robot.getDriveParametersForMediumForward();
        DriveParameters fastForward = robot.getDriveParametersForFastForward();
        
        int waitTime = 0;
        
        robot.setRed();

        
        while (opModeInInit()) {
            waitTime += gamepad1.rightBumperWasPressed() && waitTime < 20 ? 1 : 0;
            waitTime -= gamepad1.leftBumperWasPressed() && waitTime > 0 ? 1 : 0;
            telemetry.addLine("Right Bumper to add 1 Second");
            telemetry.addLine("Left Bumper to subtract 1 Second");
            telemetry.addData("waitTime:", waitTime);
            robot.updatePose();
            robot.reportPositionData();
            
        }
        
        waitForStart();
        double start_time = getRuntime();
        
        robot.flywheelSetPower(0.52);
        
        robot.setBrakes();

        sleep(waitTime*1000);

        robot.strafeToPoseAndStopD(13.5,43.7,23.125, mediumForward);
        
        while (opModeIsActive() && getRuntime() < start_time + 5) {
            double flywheel_velocity = robot.flywheel.getVelocity();
            telemetry.addData("Flywheel Velocity:", flywheel_velocity);
            telemetry.update();
        }
        
        sleep(3500);
        
        robot.autoFireSequence();
        sleep(1400-750);
        
        robot.flywheelSetPower(0.53);
        
        robot.autoFireSequence();
        sleep(1400-750);
        robot.autoFireSequence();
        sleep(1400-750);
        robot.autoFireSequence();
        
        
        
        //sleep(10000);

    }
}

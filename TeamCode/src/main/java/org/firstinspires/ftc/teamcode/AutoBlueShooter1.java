package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.RoRobot.DriveParameters;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled

@Autonomous(name = "Blue Shooter 1", group = "auto")
public class AutoBlueShooter1 extends LinearOpMode {
 public RoRobot robot = new RoRobot();
    
    @Override
    public void runOpMode() throws InterruptedException {
        robot.initializeAuto(hardwareMap, (LinearOpMode)this);

        DriveParameters slowForward = robot.getDriveParametersForSlowForward();
        DriveParameters mediumForward = robot.getDriveParametersForMediumForward();
        DriveParameters fastForward = robot.getDriveParametersForFastForward();
        
        
        robot.setBlue();
        
        while (opModeInInit()) {
            robot.updatePose();
            robot.reportPositionData();
        }
        
        waitForStart();
        double start_time = getRuntime();
        
        robot.flywheelSetPower(0.53);
        
        robot.setBrakes();

        
        robot.strafeToPoseAndStopD(-20.4,12.1,180-56+4, mediumForward);
        
        while (opModeIsActive() && getRuntime() < start_time + 5) {
            double flywheel_velocity = robot.flywheel.getVelocity();
            telemetry.addData("Flywheel Velocity:", flywheel_velocity);
            telemetry.update();
        }
        
        
        robot.autoFireSequence();
        sleep(1400-750);
        robot.autoFireSequence();
        sleep(1400-750);
        robot.autoFireSequence();
        sleep(1400-750);
        robot.autoFireSequence();
        
        robot.strafeToPoseAndStopD(-20.4,0.0,180-54+4, mediumForward);
        
        
        //sleep(10000);

    }
}

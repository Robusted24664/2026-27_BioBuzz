package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.RoRobot.DriveParameters;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;


@Disabled
@Autonomous(name = "Red Shooter 2 Rapid", group = "auto")
public class AutoRedShooter2Rapid extends LinearOpMode {
 public RoRobot robot = new RoRobot();
    
    @Override
    public void runOpMode() throws InterruptedException {
        robot.initializeAuto(hardwareMap, (LinearOpMode)this);

        DriveParameters slowForward = robot.getDriveParametersForSlowForward();
        DriveParameters mediumForward = robot.getDriveParametersForMediumForward();
        DriveParameters fastForward = robot.getDriveParametersForFastForward();
        DriveParameters turboForward = robot.getDriveParametersForTurbo();
        
        
        robot.setRed();
        
        while (opModeInInit()) {
            /*
            telemetry.addLine("Press A or Cross to initialize odometry");
            telemetry.addLine("Press B or Circle to lock the start position");
            telemetry.addLine("");
            if (gamepad1.aWasPressed()) {
                robot.initializeOdo();
                robot.setPoseD(36.8, 31.5, 90);
            } 
            if (gamepad1.bWasPressed()) {
                robot.startPose = robot.poseEst;
            }
            */
            robot.updatePose();
            robot.reportPositionData();
        }
        
        //START
        waitForStart();
        
        /*
        if (robot.startPose.isCloseTo(RoPose.getTrivial()) == false) {
            robot.setPose(robot.startPose);
        }
        */
        
     
        robot.setBrakes();

         if (robot.poseEst.position.y < 0.0) {
            if (robot.poseEst.heading < -0.75 * Math.PI || robot.poseEst.heading > 0.75 * Math.PI) {
                RoPose TargetPose = robot.poseEst.thenForward(-10);
                robot.strafeToPoseAndStop(TargetPose, mediumForward);
                robot.strafeToPoseAndStopD(15,-40,90, mediumForward);
            }
            robot.strafeToPoseAndStopD(14, -11, 90, fastForward);
        }
        
        
        double start_time = getRuntime();
        robot.flywheelSetPower(0.52);
           
        robot.strafeToPoseAndStopD(20.4, 12.1, 56, mediumForward);
        
        while (opModeIsActive() && getRuntime() < start_time + 3) {
            double flywheel_velocity = robot.flywheel.getVelocity();
            telemetry.addData("Flywheel Velocity:", flywheel_velocity);
            telemetry.update();
        }
        
        robot.flywheelSetPower(0.54);
        
        robot.autoFireSequenceA();
        robot.autoFireSequenceA();
        robot.autoFireSequenceC();
        sleep(300);
        robot.intakeReverse();
        robot.flywheelSetPower(0.55);
        /*
        robot.autoFireSequenceA();
        robot.autoFireSequenceC();
        robot.flywheelSetPower(0.55);
        sleep(100);
        robot.autoFireSequenceB();
        */

        ////////////////////////////////////
        // UTILITY: Collect First Line
        ////////////////////////////////////
        robot.strafeToPoseAndStopD(29.7,9.5+1,0.0, turboForward);
        robot.gateClose();
        robot.intakeOn();
        robot.strafeToPoseAndStopD(54,9.5+1,0.0, slowForward);
        robot.flywheelSetPower(0.55);
        robot.strafeToPoseAndStopD(20.4,12.1,56, fastForward);

        robot.flywheelSetPower(0.57);
        robot.autoFireSequenceA();
        robot.autoFireSequenceA();
        robot.autoFireSequenceC();
        sleep(300);
        robot.intakeReverse();
        robot.flywheelSetPower(0.55);
        /*
        robot.autoFireSequenceA();
        robot.autoFireSequenceC();
        robot.flywheelSetPower(0.55);
        sleep(100);
        robot.autoFireSequenceB();
        */
        
        ////////////////////////////////////
        // UTILITY: Collect Second Line
        ////////////////////////////////////
        
    
        robot.strafeToPoseAndStopD(29.7,9.5-24+1,0.0, turboForward);
        robot.gateClose();
        robot.intakeOn();
        robot.strafeToPoseAndStopD(59,9.5-24+1,0.0, slowForward);
        robot.strafeToPoseAndStopD(35,9.5-24+1,0.0, turboForward);
        robot.flywheelSetPower(0.55);
        robot.strafeToPoseAndStopD(20.4,12.1,56, fastForward);

        robot.flywheelSetPower(0.57);
        robot.autoFireSequenceA();
        robot.autoFireSequenceA();
        robot.autoFireSequenceC();
        sleep(300);
        robot.intakeReverse();
        robot.flywheelSetPower(0.55);
        /*
        robot.autoFireSequenceA();
        robot.autoFireSequenceC();
        robot.flywheelSetPower(0.55);
        sleep(100);
        robot.autoFireSequenceB();
        */
        
        ////////////////////////////////////
        // UTILITY: Collect Third Line
        ////////////////////////////////////
        robot.strafeToPoseAndStopD(29.7,9.5-24-24+1,0.0, turboForward);
        robot.gateClose();
        robot.intakeOn();
        robot.strafeToPoseAndStopD(59,9.5-24-24+1,0.0, slowForward);
        robot.flywheelSetPower(0.55);
        robot.strafeToPoseAndStopD(20.4,12.1,56, fastForward);
        
        robot.flywheelSetPower(0.57);
        robot.autoFireSequenceA();
        robot.autoFireSequenceA();
        robot.autoFireSequenceC();
        sleep(300);
        robot.intakeReverse();
        robot.flywheelSetPower(0.55);
        /*
        robot.autoFireSequenceA();
        robot.autoFireSequenceC();
        robot.flywheelSetPower(0.55);
        sleep(100);
        robot.autoFireSequenceB();
        */

        
        ////////////////////////////////////
        // UTILITY: Move to lever
        ////////////////////////////////////
        robot.gateClose();
        robot.strafeToPoseAndStopD(40,-6,180, turboForward);

    }
}

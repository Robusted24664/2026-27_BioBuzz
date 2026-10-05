package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.RoRobot.DriveParameters;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled

@TeleOp(name = "TestRapid", group = "auto")
public class AutotestRapid extends LinearOpMode {
 public RoRobot robot = new RoRobot();
    
    @Override
    public void runOpMode() throws InterruptedException {
        robot.initializeAuto(hardwareMap, (LinearOpMode)this);

        DriveParameters slowForward = robot.getDriveParametersForSlowForward();
        DriveParameters mediumForward = robot.getDriveParametersForMediumForward();
        DriveParameters fastForward = robot.getDriveParametersForFastForward();
        DriveParameters turboForward = robot.getDriveParametersForTurbo();
        
        robot.setPoseD(36.8, 31.5, 90);

        //robot.setPoseD(0, 0, 0);
        
        while (opModeInInit()) {
            robot.updatePose();
            robot.reportPositionData();
        }
        
        waitForStart();
        
        double start_time = getRuntime();
        
        robot.flywheelSetPower(0.55);
        
        robot.setBrakes();
        
        if (robot.poseEst.position.y < 0.0) {
            if (robot.poseEst.heading < -0.75 * Math.PI || robot.poseEst.heading > 0.75 * Math.PI) {
                RoPose TargetPose = robot.poseEst.thenForward(-10);
                robot.strafeToPoseAndStop(TargetPose, mediumForward);
                robot.strafeToPoseAndStopD(15,-40,90, mediumForward);
            }
            robot.strafeToPoseAndStopD(14,-11,90, fastForward);
        }
        
        robot.strafeToPoseAndStopD(20.4,12.1,56, mediumForward);
        
        //LOOP
        
        while (opModeIsActive()) {
            myLoop();
        }
    }    
        
    public void myLoop() {

        if (gamepad1.rightBumperWasPressed()) {
            //Now we will transition to the next state
            
            robot.flywheelSetPower(0.56);

            //sleep(500);
            robot.autoFireSequenceA();
        
            robot.autoFireSequenceA();
            
            robot.autoFireSequenceB();
            sleep(500);
        
            robot.autoFireSequenceA();

            robot.autoFireSequenceB();
            robot.flywheelSetPower(0.55);

            

            }
        }
    }

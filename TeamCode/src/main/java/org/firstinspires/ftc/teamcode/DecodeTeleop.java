package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;


import org.firstinspires.ftc.teamcode.RoRobot.DriveParameters;






@TeleOp(name = "Decode Teleop", group = "AAFirst Group")


public class DecodeTeleop extends LinearOpMode {
    public RoRobot robot = new RoRobot();

    ////////////////////////////////////
    // State info
    ////////////////////////////////////




    public double driveMultiplier = RoRobot.LIFT_DOWN_MULTIPLIER;




    public double timeWhenFired = 0;
    
    public boolean isAutoAimOn = false;


    DriveParameters slowForward = robot.getDriveParametersForSlowForward();
    DriveParameters mediumForward = robot.getDriveParametersForMediumForward();
    DriveParameters fastForward = robot.getDriveParametersForFastForward();
    DriveParameters turboForward = robot.getDriveParametersForTurbo();
    DriveParameters autoDrive = turboForward;

    RoPose centerRed  = RoPose.RoPoseD(0,0,45);
    RoPose centerBlue = RoPose.RoPoseD(0,0,180-45);

    RoPose farRed  = RoPose.RoPoseD(0,-48,60);
    RoPose farBlue = RoPose.RoPoseD(0,-48,180-60);

    RoPose humanRed  = RoPose.RoPoseD(-48,-48,180-0);
    RoPose humanBlue = RoPose.RoPoseD(48,-48,0);
    
    @Override
    public void runOpMode() throws InterruptedException {


        ////////////////////////////////////
        // OPMODE: Initialize
        ////////////////////////////////////


        robot.initializeTeleOp(hardwareMap, (LinearOpMode)this);
        robot.ledOff();

        ////////////////////////////////////
        // OPMODE: Start
        ////////////////////////////////////


        waitForStart();
        robot.flywheelOn();
        driveMultiplier = RoRobot.LIFT_DOWN_MULTIPLIER;
        //robot.parkingStickUp();
        robot.liftDown();




        while (opModeIsActive()) {
            myLoop();
        }
    }


    ////////////////////////////////////
    // OPMODE: loop
    ////////////////////////////////////


    public int loopCount = 0;

    public void myLoop() {
        telemetry.addLine("Right trigger for intake");
        telemetry.addLine("Left trigger for reverse intake");
        telemetry.addLine("Right bumper to stop intake");


        ////////////////////////////////////
        // update firing data every 3 cycles - this may need to be tuned
        loopCount += 1;
        if (loopCount % 3 == 0) {
            robot.updateVelocityAndHood();
        }


        ////////////////////////////////////
        // Intake


        if (gamepad1.right_trigger > 0.5 || gamepad2.right_trigger > 0.5) {
            //turn intake on
            robot.firingStateIntake();
        }
        if (gamepad1.left_trigger > 0.3 || gamepad2.left_trigger > 0.3){
            // outtake
            robot.firingStateOuttake();
        }


        ////////////////////////////////////
        // Firing


        if (gamepad1.rightBumperWasPressed()) {
            isAutoAimOn = false;
            robot.firingStateFire();
            timeWhenFired = getRuntime();
        }


        if (robot.firingState == RoRobot.FIRING_STATE.FIRE) {
            if (getRuntime() - timeWhenFired > RoRobot.FIRING_DURATION) {
                robot.firingStateMaintain();
            }
        }

        if (gamepad2.leftBumperWasPressed()) {
            robot.firingStateMaintain();
        }




        ////////////////////////////////////
        // Lift


        if (gamepad1.dpad_down) {
            robot.liftUp();
            driveMultiplier = RoRobot.LIFT_UP_MULTIPLIER;
        }
        if (gamepad1.dpad_up) {
            robot.liftDown();
            driveMultiplier = RoRobot.LIFT_DOWN_MULTIPLIER;
        }


        if (gamepad1.dpad_right) {
            driveMultiplier = RoRobot.PARKING_SLOW_MULTIPLIER;
        }
        if (gamepad1.dpad_left) {
            driveMultiplier = RoRobot.LIFT_DOWN_MULTIPLIER;
        }


        ////////////////////////////////////
        // Auto-Aim
        // Over-rides the right joystick if auto-aim is on


        double turningPower = -gamepad1.right_stick_x * driveMultiplier;
        double minPower = 0.2;
        double minAngle = 1;
        double angleMultiplier = 0.05;

        if (Math.abs(gamepad1.right_stick_x) > 0.3) {
            isAutoAimOn = false;
        }
        
        if (gamepad1.leftBumperWasPressed()) {
            isAutoAimOn = true;
        }

        if (isAutoAimOn) {
            robot.updatePose();
            robot.reportPositionDataWithoutUpdate();
            double deltaHeading = robot.deltaHeadingToFaceTargetD(RoRobot.targetX, RoRobot.targetY);
            telemetry.addData("Delta Heading", deltaHeading);


            double absDeltaHeading = Math.abs(deltaHeading);
            double signDeltaHeading = Math.signum(deltaHeading);
            if (absDeltaHeading > minAngle) {
                turningPower = minPower + angleMultiplier * absDeltaHeading;
            } else {
                turningPower = 0;
            }
            turningPower = Range.clip (turningPower * signDeltaHeading, -1.0, 1.0);
        }
        
        ////////////////////////////////////
        // ResetOdo
        // Resets odo at the red and blue goals

        if (gamepad2.x && gamepad2.a) {
            robot.resetBlue();
            robot.greenOn();
            sleep(250);
            robot.ledOff();
        }
        
        if (gamepad2.b && gamepad2.a) {
            robot.resetRed();
            robot.redOn();
            sleep(250);
            robot.ledOff();
        }
        
        ////////////////////////////////////
        // PresetPositions
        // move to different parts of the field
    
        
        if (gamepad1.x) {
            //move to near triangle, center of field
            if (robot.isAutoRed) {
                robot.stepStrafeTowardPose(centerRed, autoDrive);
            } else {
                robot.stepStrafeTowardPose(centerBlue, autoDrive);
            }
            return;
        } else if (gamepad1.a) {
            //move to far triangle, near audience
            if (robot.isAutoRed) {
                robot.stepStrafeTowardPose(farRed, autoDrive);
            } else {
                robot.stepStrafeTowardPose(farBlue, autoDrive);
            }
            return;
        } else if (gamepad1.b) {
            //our human player, color specific
            if (robot.isAutoRed) {
                robot.stepStrafeTowardPose(humanRed, autoDrive);
            } else {
                robot.stepStrafeTowardPose(humanBlue, autoDrive);
            }
            return;
        } else if (gamepad1.y) {
            //opponents human player
            if (robot.isAutoRed) {
                robot.stepStrafeTowardPose(humanBlue, autoDrive);
            } else {
                robot.stepStrafeTowardPose(humanRed, autoDrive);
            }
            return;
        }
        

        ////////////////////////////////////
        // Driving
        drive(-gamepad1.left_stick_y * driveMultiplier,
                -gamepad1.left_stick_x * driveMultiplier,
                turningPower,
                gamepad1.left_stick_button);

        //if (gamepad1.y && !robot.poseEst.isWithin(targetGoal, 4)){
        //    robot.strafeToPoseAndStopD(20.4,12.1,56, slowForward);
        //}
        
        telemetry.addData("Hood Position", robot.hoodPosition);
        telemetry.addData("Target Velocity", robot.targetVelocity);
        //telemetry.addData("Dist to Corner", robot.distanceToTarget());


        telemetry.update();


    }


    public void drive(double forward, double left, double rotateccw, boolean turbo) {
        double maxSpeed = (turbo) ? 1.0 : 0.55;
        robot.strafeDrive(forward*maxSpeed, left*maxSpeed, rotateccw*maxSpeed);
        telemetry.addData("Turbo:", turbo);
    }
}



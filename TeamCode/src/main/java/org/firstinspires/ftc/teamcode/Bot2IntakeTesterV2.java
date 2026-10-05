/*
Copyright 2026 FIRST Tech Challenge Team 24664

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


import java.text.SimpleDateFormat;
import java.util.Date;

/**
 * This file contains a minimal example of an iterative (Non-Linear) "OpMode". An OpMode is a
 * 'program' that runs in either the autonomous or the TeleOp period of an FTC match. The names
 * of OpModes appear on the menu of the FTC Driver Station. When an selection is made from the
 * menu, the corresponding OpMode class is instantiated on the Robot Controller and executed.
 *
 * Remove the @Disabled annotation on the next line or two (if present) to add this OpMode to the
 * Driver Station OpMode list, or add a @Disabled annotation to prevent this OpMode from being
 * added to the Driver Station.
 */

@TeleOp(name = "Bot2 Intake Tester V2", group = "Test")

public class Bot2IntakeTesterV2 extends OpMode {
        // This declares the motors needed
    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor backLeftDrive;
    DcMotor backRightDrive;
    DcMotor intakeFront;
    DcMotor intakeLift;
    DcMotorEx flywheel;
    DcMotorEx flywheel2;


    private Servo gate = null;
    private Servo kicker = null;
    double flapperPower = 0;  
    double transferPower = 0;
    private Servo leftStopper = null;
    private Servo rightStopper = null;
    private Servo hood = null;
    
    public double hoodPos = 0.5;



    public static final double NEW_P = 20;
    public static final double NEW_I = 0.9;
    public static final double NEW_D = 5.0;
    public static final double NEW_F = 0.0;
    
    

    DigitalChannel transferBallDetector;
    DigitalChannel intakeBallDetector;
    public double lastNoIntakeBallTime = 0;
    
    public static final double GATE_OPEN_POSITION = 0.5785;
    public static final double GATE_CLOSED_POSITION = 0.4783;
    public static final double KICKER_TOP_POSITION = 0.2484;
    public static final double KICKER_BOTTOM_POSITION = 0.68;
    
    public static final double LEFT_STOPPER_OPEN_POSITION = 0.61;
    public static final double RIGHT_STOPPER_OPEN_POSITION = 0.64;
    public static final double LEFT_STOPPER_CLOSED_POSITION = 0.25;
    public static final double RIGHT_STOPPER_CLOSED_POSITION = 0.29;
    
    public static final double TELEOP_FIRE_BOOST_RATIO = 2.0;
    public static final double INTAKEBALLTHRESHOLD = 0.25;
    public static final double SLOWINTAKEPOWER = 0.33;
    
    public static final double FIRING_DURATION = 2.0;
    
    public static final double WAITING_PERCENTAGE = 0.95;


    double velocityMultiplier = 6000.0*0.85*28.0/60.0;
    double targetVelocity = 1050;
    double targetPower = targetVelocity/velocityMultiplier;
    double waitingPower = targetPower*WAITING_PERCENTAGE;
    
    public double flywheel_speed = targetVelocity;


    public enum FiringState {
        OFF,
        INTAKE_SLOW,
        INTAKE,
        OUTTAKE,
        FIRE
    }

    FiringState firingstate = FiringState.OFF;
    
    public void setTargetVelocity(double newTargetVelocity) {
        targetVelocity = newTargetVelocity;
        targetPower = targetVelocity/velocityMultiplier;
        waitingPower = targetPower*WAITING_PERCENTAGE;
    }
    
    public void flywheelSetPower(double power) {
        flywheel.setPower(power);
        flywheel2.setPower(power);
    }
    
    public void flywheelBoost() {
        double oldPower = flywheel.getPower();
        flywheelSetPower(oldPower*TELEOP_FIRE_BOOST_RATIO);
    }
    
    public void waitForFlywheelVelocity(double targetVelocity) {
        while (true) {
            double flywheel_velocity = flywheel.getVelocity();
            //telemetry.addData("Flywheel Velocity:", flywheel_velocity);
            //telemetry.addData("target flywheel velocity:", targetVelocity);

            //telemetry.update();
            if (flywheel_velocity > targetVelocity) {
                break;
                
            }
        }
    }
    
    
    public double timeWhenFired = 0;

    
    public void fireSequence() {
        flywheelSetPower(targetPower*TELEOP_FIRE_BOOST_RATIO);
        waitForFlywheelVelocity(targetVelocity);
        //flywheelBoost();
        //robot.firingStateFire(); //the following is the same
        leftStopper.setPosition(LEFT_STOPPER_OPEN_POSITION);
        rightStopper.setPosition(RIGHT_STOPPER_OPEN_POSITION);
        flapperPower = 1;
        transferPower = 1;
        //end
    }

    // This declares the IMU needed to get the current direction the robot is facing
    IMU imu;

    @Override
    public void init() {
        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRight");
        backLeftDrive = hardwareMap.get(DcMotor.class, "backLeft");
        backRightDrive = hardwareMap.get(DcMotor.class, "backRight");
        
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheel2 = hardwareMap.get(DcMotorEx.class, "flywheel2");

        intakeFront = hardwareMap.get(DcMotor.class, "intakeFront");
        intakeLift = hardwareMap.get(DcMotor.class, "intakeLift");
        
        gate = hardwareMap.get(Servo.class, "gate");
        kicker = hardwareMap.get(Servo.class, "kicker");
        
        leftStopper = hardwareMap.get(Servo.class, "leftStopper");
        rightStopper = hardwareMap.get(Servo.class, "rightStopper");
        
        hood = hardwareMap.get(Servo.class, "hood");


        transferBallDetector = hardwareMap.get(DigitalChannel.class, "transferBallDetector");
        intakeBallDetector = hardwareMap.get(DigitalChannel.class, "intakeBallDetector");
        lastNoIntakeBallTime = getRuntime();
        
        // We set the left motors in reverse which is needed for drive trains where the left
        // motors are opposite to the right ones.
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        
        
        intakeFront.setDirection(DcMotor.Direction.REVERSE);
        intakeLift.setDirection(DcMotor.Direction.FORWARD);
        
        flywheel.setDirection(DcMotorEx.Direction.FORWARD);
        flywheel2.setDirection(DcMotorEx.Direction.REVERSE);

    
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        PIDFCoefficients pidfNew = new PIDFCoefficients(NEW_P,NEW_I, NEW_D, NEW_F);
        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfNew);

        transferBallDetector.setMode(DigitalChannel.Mode.INPUT);
        intakeBallDetector.setMode(DigitalChannel.Mode.INPUT);
    

        // This uses RUN_USING_ENCODER to be more accurate.   If you don't have the encoder
        // wires, you should remove these
        /*
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        */

        imu = hardwareMap.get(IMU.class, "imu");
        // This needs to be changed to match the orientation on your robot
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        RevHubOrientationOnRobot orientationOnRobot = new
                RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }

    @Override
    public void loop() {
        
        boolean transferState = transferBallDetector.getState();
        boolean intakeState = intakeBallDetector.getState();
        double flywheel_velocity = flywheel.getVelocity();
        

        if (gamepad1.left_bumper){
            // Stops
            firingstate = FiringState.INTAKE_SLOW;
            flapperPower = SLOWINTAKEPOWER;
            transferPower = SLOWINTAKEPOWER;
            leftStopper.setPosition(LEFT_STOPPER_CLOSED_POSITION);
            rightStopper.setPosition(RIGHT_STOPPER_CLOSED_POSITION);
        }
            
        if (gamepad1.right_trigger > 0.5) {
            firingstate = FiringState.INTAKE;
            leftStopper.setPosition(LEFT_STOPPER_CLOSED_POSITION);
            rightStopper.setPosition(RIGHT_STOPPER_CLOSED_POSITION);
            // Intakes the ball in
            flapperPower = 1;
            transferPower = 1;
        }
        
        
        if (gamepad1.left_trigger > 0.5){
            firingstate = FiringState.OUTTAKE;
            // Outakes the ball
            flapperPower = -1;
            transferPower = -1;
            leftStopper.setPosition(LEFT_STOPPER_OPEN_POSITION);
            rightStopper.setPosition(RIGHT_STOPPER_OPEN_POSITION);
        }
        
        if (gamepad1.right_bumper) {
            firingstate = FiringState.FIRE;
            fireSequence();
            timeWhenFired = getRuntime();
        } 
        
        if (getRuntime() - timeWhenFired > FIRING_DURATION && firingstate == FiringState.FIRE) {
            firingstate = FiringState.INTAKE_SLOW;
            flapperPower = SLOWINTAKEPOWER;
            transferPower = SLOWINTAKEPOWER;
            leftStopper.setPosition(LEFT_STOPPER_CLOSED_POSITION);
            rightStopper.setPosition(RIGHT_STOPPER_CLOSED_POSITION);
            flywheelSetPower(waitingPower);

        }
        
        if (gamepad1.dpad_up) {
            flywheel_speed += 1.0;
            setTargetVelocity(flywheel_speed);
            flywheelSetPower(waitingPower);
        } 
        if (gamepad1.dpad_down) {
            flywheel_speed -= 1.0;
            setTargetVelocity(flywheel_speed);
            flywheelSetPower(waitingPower);
        }
        flywheel_speed = Range.clip(flywheel_speed, -5000,5000);
        
        if (transferState == true && firingstate == FiringState.INTAKE) {
            transferPower = 0;
        }
        
        
        double timeSinceNoIntakeBall = getRuntime() - lastNoIntakeBallTime;
        if (intakeState == false) {
            lastNoIntakeBallTime = getRuntime();
        } else if (firingstate == FiringState.INTAKE && timeSinceNoIntakeBall > INTAKEBALLTHRESHOLD){
            flapperPower = SLOWINTAKEPOWER;
        }
        
        hoodPos += gamepad2.left_stick_y / 1000;
        hoodPos = Range.clip(hoodPos, 0.18, 0.99);
        hood.setPosition(hoodPos);
         
         
        driveFieldRelative(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        intakeFront.setPower(flapperPower);
        intakeLift.setPower(transferPower * 1);

        telemetry.addLine("Moving the right joystick left and right turns the robot");
        telemetry.addData("Intake Power:", flapperPower);
        telemetry.addData("Transfer Power:", transferPower);
        telemetry.addData("Transfer Ball Detector ", transferState);
        telemetry.addData("Intake Ball Detector ", intakeState);

        telemetry.addData("Flywheel Power:", flywheel_speed);
        telemetry.addData("FiringState:", firingstate);
        telemetry.addData("Flywheel Velocity:", flywheel_velocity);
        telemetry.addData("hood Position", hoodPos);


    }

    // This routine drives the robot field relative
    private void driveFieldRelative(double forward, double right, double rotate) {
        // First, convert direction being asked to drive to polar coordinates
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);

        // Second, rotate angle by the angle the robot is pointing
        theta = AngleUnit.normalizeRadians(theta -
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
        theta=0;

        // Third, convert back to cartesian
        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        // Finally, call the drive method with robot relative forward and right amounts
        drive(forward, right, rotate);
    }

    // Thanks to FTC16072 for sharing this code!!
    public void drive(double forward, double right, double rotate) {
        // This calculates the power needed for each wheel based on the amount of forward,
        // strafe right, and rotate
        double frontLeftPower  = + forward + right + rotate;
        double frontRightPower = + forward - right - rotate;
        double backLeftPower   = + forward - right + rotate;
        double backRightPower  = + forward + right - rotate;


        double maxPower = 1.0;
        double maxSpeed = 0.85;  // make this slower for outreaches

        // This is needed to make sure we don't pass > 1.0 to any wheel
        // It allows us to keep all of the motors in proportion to what they should
        // be and not get clipped
        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));

        // We multiply by maxSpeed so that it can be set lower for outreaches
        // When a young child is driving the robot, we may not want to allow full
        // speed.
        frontLeftDrive.setPower(maxSpeed * (frontLeftPower / maxPower));
        frontRightDrive.setPower(maxSpeed * (frontRightPower / maxPower));
        backLeftDrive.setPower(maxSpeed * (backLeftPower / maxPower));
        backRightDrive.setPower(maxSpeed * (backRightPower / maxPower));
    }
}

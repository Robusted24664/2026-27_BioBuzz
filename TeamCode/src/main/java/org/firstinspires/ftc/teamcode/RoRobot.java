package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import java.util.ArrayList;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import java.util.Locale;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import android.util.Log;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;


////////////////////////////
//
// NOTE: In this file, vDiff = vR - vL, so positive vDiff means turning to the left.
// Turning to the left means increasing the angle in the standard x/y coordinate system.
//
// NOTE: There are two valid ways to install mecanum wheels.  If the mecanum driving
//       isn't working try toggling the value of IS_MECANUM_XDRIVE
//
////////////////////////////




public class RoRobot {


    ////////////////////////////////////
    // Hardware interface
    ////////////////////////////////////


    public DcMotor frontLeft = null;
    public DcMotor backLeft = null;
    public DcMotor backRight = null;
    public DcMotor frontRight = null;


    public DcMotorEx flywheel = null;
    public DcMotorEx flywheel2 = null;




    public DcMotor intakeFront = null;
    public DcMotor intakeLift = null;


    public Servo gate = null;
    public Servo kicker = null;

    public Servo leftStopper = null;
    public Servo rightStopper = null;

    private Servo hood = null;




    public GoBildaPinpointDriver odo = null;

    //public Servo parkingStick = null;

    private Servo left1Lift = null;
    private Servo right1Lift = null;
    private Servo left2Lift = null;
    private Servo right2Lift = null;

    //Digital sensors



    DigitalChannel transferBallDetector;
    DigitalChannel intakeBallDetector;

    LED leftRed;
    LED leftGreen;


    LED rightRed;
    LED rightGreen;




    ////////////////////////////////////
    // Important Constants
    ////////////////////////////////////


    // controlling the gate for shooting
    public static final double GATE_OPEN_POSITION = 0.5785;
    public static final double GATE_CLOSED_POSITION = 0.4783;
    public static final double GATE_CLOSE_DELAY = 0.15; // seconds

    // Stopper positions, to let balls in and stop balls
    public static final double LEFT_STOPPER_OPEN_POSITION = 0.61;
    public static final double RIGHT_STOPPER_OPEN_POSITION = 0.64;
    public static final double LEFT_STOPPER_CLOSED_POSITION = 0.25;
    public static final double RIGHT_STOPPER_CLOSED_POSITION = 0.29;


    // controlling the kicker for shooting
    public static final double KICKER_TOP_POSITION = 0.2484;
    public static final double KICKER_BOTTOM_POSITION = 0.68;
    public static final double KICKER_CLOSE_DELAY = 0.75; // seconds




    // speed of the flywheel
    public static final double SHOOTING_POWER = 0.55;


    // adjust for weight distribution to improve strafing
    public static final double BACK_DRIVE_MULTIPLIER = 1.1;
    public static final double FRONT_DRIVE_MULTIPLIER = 1.0;


    // Which type of mecanum setup is it?
    public static final boolean IS_MECANUM_XDRIVE = true;

    // How do we tell when the robot is stuck?
    public static final int DRIVING_OBSTRUCTION_CYCLES = 50;
    public static final double DRIVING_OBSTRUCTION_DISTANCE = 1.0;

    // How do we tell when the third ball is in the intake?
    public static final double INTAKEBALLTHRESHOLD = 0.25;


    // Stop firing after certain amount of time, switch state?
    public static final double FIRING_DURATION = 1.0;


    public static final double FIRE_BOOST_RATIO = 1.05;
    public static final double TELEOP_FIRE_BOOST_RATIO = 2;
    public static final double WAITING_PERCENTAGE = 0.90;
    
    public static boolean isAutoRed = true;






    double velocityMultiplier = 6000.0*0.85*28.0/60.0;
    double targetVelocity = 1050;
    double targetPower = targetVelocity/velocityMultiplier;
    double waitingPower = targetPower*WAITING_PERCENTAGE;
    double hoodPosition = 0.5;

    public void setTargetVelocity(double newTargetVelocity) {
        targetVelocity = newTargetVelocity;
        targetPower = targetVelocity/velocityMultiplier;
        waitingPower = targetPower*WAITING_PERCENTAGE;
        flywheelSetPower(waitingPower);
    }







   /*
   //Controls of Parking Stick
   public static final double PARKING_STICK_UP = 0.1662;
   public static final double PARKING_STICK_HOVER = 0.5925;
   public static final double PARKING_STICK_DOWN = 0.6316;
   */


    public static final double LIFT_DOWN_MULTIPLIER = 1.0;
    //public static final double PARKING_STICK_HOVER_MULTIPLIER = 0.4;
    //public static final double PARKING_STICK_DOWN_MULTIPLIER = 0.2;
    public static final double PARKING_SLOW_MULTIPLIER = 0.4;

    public static final double LIFT_UP_MULTIPLIER = 0.0;


    //Improved PID values
    public static final double NEW_P = 20.0;
    public static final double NEW_I = 0.9;
    public static final double NEW_D = 5.0;
    public static final double NEW_F = 0.0;

    //Lift Values
    public static final double LIFT_DOWN = 0.17;
    public static final double LIFT_UP_RIGHT = 0.9;
    public static final double LIFT_UP_LEFT = 0.7;




    ////////////////////////////////////
    // State info
    ////////////////////////////////////

    //Where to aim the robot in Decode Teleop, Set in Auto, Used in Teleop
    public static double targetX = 72;
    public static double targetY = 72; //Defaults to red goal

    ////////////////////////////////////
    // Robot position information
    public RoPose poseEst = RoPose.getTrivial();
    public static boolean isPoseBaseInitialized = false;
    public static RoPose poseBase = RoPose.getTrivial();
    public static double TRACK_WIDTH = 14.50; // inches
    ArrayList<RoPose> poseList = new ArrayList<RoPose>();
    public RoPose startPose = RoPose.getTrivial();


    ////////////////////////////////////
    // parent opmode
    public static LinearOpMode opMode = null;


    ////////////////////////////////////
    // driving parameters
    public enum DRIVE_DIRECTION {
        FORWARD,
        REVERSE
    }


    // default driving parameters:
    public static class DriveParameters {
        public double vCMax = 0.5;
        public double vDiffMax = 0.4;
        public double slowDistance = 15;
        public double slowAngleD = 10;
        public double threshold = 2;
        public double thresholdD = 2;
        public DRIVE_DIRECTION direction = DRIVE_DIRECTION.FORWARD;
    }


    ////////////////////////////////////
    // UTILITY: Kicker movement
    ////////////////////////////////////


    public void kickerUp() {
        kicker.setPosition(KICKER_TOP_POSITION);
    }


    public void kickerDown() {
        kicker.setPosition(KICKER_BOTTOM_POSITION);
    }


    ////////////////////////////////////
    // UTILITY: Parking Stick movement
    ////////////////////////////////////
   /*
   public void parkingStickUp() {
       parkingStick.setPosition(PARKING_STICK_UP);
   }


   public void parkingStickDown() {
       parkingStick.setPosition(PARKING_STICK_DOWN);
   }


   public void parkingStickHover() {
       parkingStick.setPosition(PARKING_STICK_HOVER);
   }
   */

    ////////////////////////////////////
    // UTILITY: Lift Movement
    ////////////////////////////////////


    public void liftUp() {
        left1Lift.setPosition(LIFT_UP_LEFT);
        right1Lift.setPosition(LIFT_UP_RIGHT);
        left2Lift.setPosition(LIFT_UP_LEFT);
        right2Lift.setPosition(LIFT_UP_RIGHT);
    }


    public void liftDown() {
        left1Lift.setPosition(LIFT_DOWN);
        right1Lift.setPosition(LIFT_DOWN);
        left2Lift.setPosition(LIFT_DOWN);
        right2Lift.setPosition(LIFT_DOWN);
    }




    ////////////////////////////////////
    // UTILITY: Gate movement
    ////////////////////////////////////


    public void gateOpen() {
        gate.setPosition(GATE_OPEN_POSITION);
    }


    public void gateClose() {
        gate.setPosition(GATE_CLOSED_POSITION);
    }


    ////////////////////////////////////
    // UTILITY: Flywheel control
    ////////////////////////////////////
    public void flywheelSetPower(double power) {
        flywheel.setPower(power);
        flywheel2.setPower(power);


    }


    public void flywheelOn() {
        flywheel.setPower(SHOOTING_POWER);
        flywheel2.setPower(SHOOTING_POWER);


    }


    public void flywheelOff() {
        flywheel.setPower(0);
        flywheel2.setPower(0);


    }

    public double flywheelVelocity() {
        return flywheel.getVelocity();
    }

    public void flywheelBoost() {
        flywheelSetPower(targetPower*FIRE_BOOST_RATIO);
    }


    public void teleopFlywheelBoost() {
        flywheelSetPower(targetPower*TELEOP_FIRE_BOOST_RATIO);
    }

    public void waitForFlywheelVelocity(double targetVelocity) {
        while (opMode.opModeIsActive()) {
            double flywheel_velocity = flywheel.getVelocity();
            if (flywheel_velocity > targetVelocity) {
                break;
            }
        }
    }

    public void updateVelocityAndHood() {
        updatePose();


        double dist = distanceToTarget();
        double newHoodPosition = 2.57 - 0.0576*dist + 0.000403*dist*dist;
        double newTargetVelocity = 733 + 1.13*dist + 0.0225*dist*dist   + 60;
        if (dist > 75) {
            newHoodPosition = .5 - (dist-75)/100.;
        }
        if (dist > 110) {
            newHoodPosition = 0.18;   // far firing zone is a special case
        }
        double nearThresh = 110;
        if (dist < nearThresh) {
            // we seem to need to add more power for close shooting
            newTargetVelocity += 100 + 0.75*(nearThresh-dist);
        }
        hoodPos(newHoodPosition);
        setTargetVelocity(newTargetVelocity);
    }





    ////////////////////////////////////
    // UTILITY: Stopper control
    ////////////////////////////////////


    public void stopperClose() {
        leftStopper.setPosition(LEFT_STOPPER_CLOSED_POSITION);
        rightStopper.setPosition(RIGHT_STOPPER_CLOSED_POSITION);
    }


    public void stopperOpen() {
        leftStopper.setPosition(LEFT_STOPPER_OPEN_POSITION);
        rightStopper.setPosition(RIGHT_STOPPER_OPEN_POSITION);
    }


    ////////////////////////////////////
    // UTILITY: Hood control
    ////////////////////////////////////

    public void hoodPos(double pos) {
        hoodPosition = pos;
        hood.setPosition(pos);
    }



    ////////////////////////////////////
    // UTILITY: Intake control
    ////////////////////////////////////

    public enum FIRING_STATE {
        OFF,
        MAINTAIN,
        INTAKE,
        OUTTAKE,
        FIRE
    }

    ArrayList<Double> forwardPowerHistory = new ArrayList<Double>();
    public FIRING_STATE firingState = FIRING_STATE.OFF;

    public void firingStateOff() {
        firingState = FIRING_STATE.OFF;
        intakeOff();
        transferOff();
        stopperClose();
        //flywheelOff();
    }

    public void firingStateIntake() {
        firingState = FIRING_STATE.INTAKE;
        intakeOn();
        transferOn();
        stopperClose();
        //flywheelOn();
    }

    public void firingStateMaintain() {
        firingState = FIRING_STATE.MAINTAIN;
        intakeSlow();
        stopperClose();
        greenOn();
        setTargetVelocity(targetVelocity);
        //flywheelOn();
    }

    public void firingStateOuttake() {
        firingState = FIRING_STATE.OUTTAKE;
        intakeReverse();
        stopperOpen();
        greenOn();
        //flywheelOff();
    }

    public void firingStateFire() {
        firingState = FIRING_STATE.FIRE;
        teleopFlywheelBoost();
        waitForFlywheelVelocity(targetVelocity);
        stopperOpen();
        intakeOn();
        transferOn();
    }
    
    public void autoFiringStateFire() {
        firingState = FIRING_STATE.FIRE;
        stopperOpen();
        intakeOn();
        transferOn();
    }

    private boolean isBackingUp() {
        int cycleThreshold = DRIVING_OBSTRUCTION_CYCLES;
        int maxIndex = forwardPowerHistory.size() - 1;
        if (maxIndex < cycleThreshold) {
            //Its too early to make a decision if its obstructed
            return false;
        }
        for (int i = 0; i < cycleThreshold; i++) {
            double forwardI = forwardPowerHistory.get(maxIndex - i);
            if (forwardI > -0.1) {
                // we were not going backward i steps ago
                return false;
            }
        }
        return true;
    }

    double lastNoIntakeBallTime = 0;
    public boolean isThirdBall() {
        if (intakeBallDetector.getState() == false) {
            lastNoIntakeBallTime = opMode.getRuntime();
            return false;
        }
        double timeSinceNoIntakeBall = opMode.getRuntime() - lastNoIntakeBallTime;
        if (timeSinceNoIntakeBall > INTAKEBALLTHRESHOLD) {
            return true;
        }
        return false;
    }

    public void updateBallDetector(double forward) {
        forwardPowerHistory.add(forward);
        if (firingState == FIRING_STATE.INTAKE) {
            boolean isBallInTransfer = transferBallDetector.getState();
            if (isBallInTransfer) {
                transferOff();
            }
            if (isThirdBall()) {
                intakeSlow();
                redOn();
            } else if (isBackingUp()) {
                intakeSlow();
            } else {
                intakeOn();
            }
        }
    }


    public void intakeOn() {
        intakeFront.setPower(1.0);
    }
    public void transferOn() {
        intakeLift.setPower(1.0);
    }

    public void transferOff() {
        intakeLift.setPower(0.0);
    }


    public void intakeOff() {
        intakeFront.setPower(0.0);
    }


    public void intakeReverse() {
        intakeFront.setPower(-1.0);
        intakeLift.setPower(-1.0);
    }

    public void intakeSlow() {
        intakeFront.setPower(0.3);
    }

    public void intakeSetPower(double flapperPower) {
        intakeFront.setPower(flapperPower);
        intakeLift.setPower(flapperPower * 0.5);
    }

    ////////////////////////////////////
    // UTILITY: LED control
    ////////////////////////////////////

    public enum LED_COLOR {
        RED,
        GREEN,
        AMBER,
        OFF,
        UNDEFINED
    }
    public LED_COLOR ledColor = LED_COLOR.UNDEFINED;

    public void redOn() {
        if (ledColor != LED_COLOR.RED) {
            leftRed.on();
            rightRed.on();

            leftGreen.off();
            rightGreen.off();
            ledColor = LED_COLOR.RED;
        }

    }

    public void greenOn() {
        if (ledColor != LED_COLOR.GREEN) {
            leftRed.off();
            rightRed.off();

            leftGreen.on();
            rightGreen.on();
            ledColor = LED_COLOR.GREEN;
        }
    }

    public void amberOn() {
        if (ledColor != LED_COLOR.AMBER) {
            leftRed.on();
            rightRed.on();

            leftGreen.on();
            rightGreen.on();
            ledColor = LED_COLOR.AMBER;
        }
    }

    public void ledOff() {
        if (ledColor != LED_COLOR.OFF) {
            leftRed.off();
            rightRed.off();

            leftGreen.off();
            rightGreen.off();
            ledColor = LED_COLOR.OFF;
        }
    }


    ////////////////////////////////////
    // UTILITY: Firing in Auto
    ////////////////////////////////////


    public void autoFireSequenceA() {
        // sleep times are in milliseconds
        long gateAfterStart = Math.round(1000 * GATE_CLOSE_DELAY);
        gateOpen();
        opMode.sleep(gateAfterStart);
        kickerUp();
        //opMode.sleep(gateAfterStart);
    }

    public void autoFireSequenceB() {
        // sleep times are in milliseconds
        long gateAfterStart = Math.round(1000 * GATE_CLOSE_DELAY);
        long kickerAfterStart = Math.round(1000 * KICKER_CLOSE_DELAY);
        long kickerAfterGate = kickerAfterStart - gateAfterStart;
        opMode.sleep(kickerAfterStart);
        gateClose();
        kickerDown();
    }

    public void autoFireSequenceC() {
        // sleep times are in milliseconds
        long gateAfterStart = Math.round(1000 * GATE_CLOSE_DELAY);
        long kickerAfterStart = Math.round(1000 * KICKER_CLOSE_DELAY);
        long kickerAfterGate = kickerAfterStart - gateAfterStart;
        opMode.sleep(kickerAfterStart);
        //gateClose();
        kickerDown();
    }

    public void autoFireSequence() {
        // sleep times are in milliseconds
        long gateAfterStart = Math.round(1000 * GATE_CLOSE_DELAY);
        long kickerAfterStart = Math.round(1000 * KICKER_CLOSE_DELAY);
        long kickerAfterGate = kickerAfterStart - gateAfterStart;
        gateOpen();
        opMode.sleep(gateAfterStart);
        gateClose();
        kickerUp();
        opMode.sleep(kickerAfterGate);
        kickerDown();
    }


    ////////////////////////////////////
    // UTILITY: Standard driving parameters
    ////////////////////////////////////
    ///
    /// Note: these parameters probably need to be tweaked every year


    public DriveParameters getDriveParametersForSlowForward() {
        DriveParameters slowForward = new DriveParameters();
        slowForward.vCMax = 0.25;
        slowForward.vDiffMax = 0.35;
        slowForward.slowDistance = 0;
        slowForward.threshold = 1;
        slowForward.direction = DRIVE_DIRECTION.FORWARD;
        return slowForward;
    }


    public DriveParameters getDriveParametersForMediumForward() {
        DriveParameters mediumForward = new DriveParameters();
        mediumForward.vCMax = 0.6;
        mediumForward.vDiffMax = 0.5;
        mediumForward.slowDistance = 8;
        mediumForward.threshold = 3;
        mediumForward.direction = DRIVE_DIRECTION.FORWARD;
        return mediumForward;
    }

    public DriveParameters getDriveParametersForMediumFastForward() {
        DriveParameters mediumFastForward = new DriveParameters();
        mediumFastForward.vCMax = 0.65;
        mediumFastForward.vDiffMax = 0.6;
        mediumFastForward.slowDistance = 6;
        mediumFastForward.threshold = 3.5;
        mediumFastForward.direction = DRIVE_DIRECTION.FORWARD;
        return mediumFastForward;
    }


    public DriveParameters getDriveParametersForFastForward() {
        DriveParameters fastForward = new DriveParameters();
        fastForward.vCMax = 0.7;
        fastForward.vDiffMax = 0.65;
        fastForward.slowDistance = 5;
        fastForward.threshold = 4;
        fastForward.direction = DRIVE_DIRECTION.FORWARD;
        return fastForward;
    }


    public DriveParameters getDriveParametersForTurbo() {
        DriveParameters turbo = new DriveParameters();
        turbo.vCMax = 2;
        turbo.vDiffMax = 1.5;
        turbo.slowDistance = 8;
        turbo.threshold = 6;
        turbo.direction = DRIVE_DIRECTION.FORWARD;
        return turbo;
    }


    ////////////////////////////////////////////////////////////////////////
    // #####################################################################
    ////////////////////////////////////////////////////////////////////////
    //
    // Constructor
    //
    ////////////////////////////////////////////////////////////////////////
    // #####################################################################
    ////////////////////////////////////////////////////////////////////////
    //
    // Android Studio was complaining about there being no constructor.
    // So now there is a constructor, but it does nothing.
    //
    ////////////////////////////////////


    public RoRobot() {
        // do nothing
    }


    ////////////////////////////////////////////////////////////////////////
    // #####################################################################
    ////////////////////////////////////////////////////////////////////////
    //
    // Robot Initialization
    //
    ////////////////////////////////////////////////////////////////////////
    // #####################################################################
    ////////////////////////////////////////////////////////////////////////
    //
    // Call ONE of the initializers exactly ONCE at the beginning of the opmode.
    // Note: Must be a LinearOpMode
    //
    ////////////////////////////////////


    /*********************
     * initializeAuto
     * - initialization for auto opmodes
     * .
     * Inputs:
     * hardwareMap - hardwareMap from the parent opMode
     * opMode - parent opMode
     *********************/


    public void initializeAuto(HardwareMap hardwareMap, LinearOpMode opMode) {
        initializeGeneric(hardwareMap, opMode);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


        opMode.sleep(500);
        odo.resetPosAndIMU();
        opMode.telemetry.addLine("Initializing Odo");
        opMode.telemetry.update();
        opMode.sleep(1000);
        gateClose();
        kickerDown();
        liftDown();
        //parkingStick.setPosition(PARKING_STICK_UP);
    }

    public void initializeAutoNotOdo(HardwareMap hardwareMap, LinearOpMode opMode) {
        initializeGeneric(hardwareMap, opMode);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        gateClose();
        kickerDown();
        liftDown();


        //parkingStick.setPosition(PARKING_STICK_UP);
    }

    public void initializeOdo() {
        opMode.sleep(500);
        odo.resetPosAndIMU();
        opMode.telemetry.addLine("Initializing Odo");
        opMode.telemetry.update();
        opMode.sleep(500);
    }

    public void setBrakes() {
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }


    /*********************
     * initializeTeleOp
     * - initialization for teleop opmodes
     * .
     * Inputs:
     * hardwareMap - hardwareMap from the parent opMode
     * opMode - parent opMode
     *********************/


    public void initializeTeleOp(HardwareMap hardwareMap, LinearOpMode opMode) {
        initializeGeneric(hardwareMap, opMode);
    }


    /*********************
     * initializeGeneric
     * - Generic initialization for all opmodes
     * .
     * Inputs:
     * hardwareMap - hardwareMap from the parent opMode
     * opMode - parent opMode
     *********************/


    public void initializeGeneric(HardwareMap hardwareMap, LinearOpMode parentOpMode) {


        ////////////////////////////////////
        // save the parent opmode for later use


        opMode = parentOpMode;


        ////////////////////////////////////
        // Initialize the motors variables


        frontLeft = getAndInitDriveMotor(hardwareMap, "frontLeft", true);
        backLeft = getAndInitDriveMotor(hardwareMap, "backLeft", true);
        frontRight = getAndInitDriveMotor(hardwareMap, "frontRight", false);
        backRight = getAndInitDriveMotor(hardwareMap, "backRight", false);


        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheel.setDirection(DcMotor.Direction.FORWARD);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        flywheel2 = hardwareMap.get(DcMotorEx.class, "flywheel2");
        flywheel2.setDirection(DcMotor.Direction.REVERSE);
        flywheel2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        intakeFront = hardwareMap.get(DcMotor.class, "intakeFront");
        intakeFront.setDirection(DcMotor.Direction.REVERSE);


        intakeLift = hardwareMap.get(DcMotor.class, "intakeLift");
        intakeLift.setDirection(DcMotor.Direction.FORWARD);





        //parkingStick = hardwareMap.get(Servo.class, "parkingStick");



        ////////////////////////////////////
        // Initialize the servos


        gate = hardwareMap.get(Servo.class, "gate");
        kicker = hardwareMap.get(Servo.class, "kicker");

        leftStopper = hardwareMap.get(Servo.class, "leftStopper");
        rightStopper = hardwareMap.get(Servo.class, "rightStopper");

        hood = hardwareMap.get(Servo.class, "hood");



        left1Lift = hardwareMap.get(Servo.class, "left1Lift");
        right1Lift = hardwareMap.get(Servo.class, "right1Lift");
        left2Lift = hardwareMap.get(Servo.class, "left2Lift");
        right2Lift = hardwareMap.get(Servo.class, "right2Lift");



        transferBallDetector = hardwareMap.get(DigitalChannel.class, "transferBallDetector");
        intakeBallDetector = hardwareMap.get(DigitalChannel.class, "intakeBallDetector");

        transferBallDetector.setMode(DigitalChannel.Mode.INPUT);
        intakeBallDetector.setMode(DigitalChannel.Mode.INPUT);

        leftRed = hardwareMap.get(LED.class, "leftRed");
        leftGreen = hardwareMap.get(LED.class, "leftGreen");
        rightRed = hardwareMap.get(LED.class, "rightRed");
        rightGreen = hardwareMap.get(LED.class, "rightGreen");

        ledOff();


        ////////////////////////////////////
        // Initialize the pinpoint odometry computer


        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(-85, -138, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);

        //Improved PIDF
        PIDFCoefficients pidfNew = new PIDFCoefficients(NEW_P,NEW_I, NEW_D, NEW_F);
        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfNew);
    }


    ////////////////////////////////////
    // INITIALIZATION UTILITY
    ////////////////////////////////////


    /*********************
     * getAndInitDriveMotor
     * - retrieve and initialization a drive motor
     * .
     * Inputs:
     * hardwareMap - hardwareMap from the parent opMode
     * name - name for the motor in the hardware map
     * .
     * Note: all drive motors are initialized in the same way
     *********************/


    private DcMotor getAndInitDriveMotor(HardwareMap hardwareMap, String name, boolean reverse) {
        DcMotor motor = hardwareMap.get(DcMotor.class, name);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // No encoders on drive motors
        if (reverse) {
            motor.setDirection(DcMotor.Direction.REVERSE);
        } else {
            motor.setDirection(DcMotor.Direction.FORWARD);
        }

        return motor;
    }


    ////////////////////////////////////////////////////////////////////////
    // #####################################################################
    ////////////////////////////////////////////////////////////////////////
    //
    // RED/BLUE SETUP
    //
    ////////////////////////////////////////////////////////////////////////
    // #####################################################################
    ////////////////////////////////////////////////////////////////////////

    public void setBlue() {
        isAutoRed = false;
        setPoseD(-36.25, 31.5, 180-90);
        targetX = -72.0;
        targetY = 72.0;
    }
    public void setRed() {
        isAutoRed = true;
        setPoseD(36.8, 31.5, 90);
        targetX = 72.0;
        targetY = 72.0;
    }
    
    public void resetBlue() {
        isAutoRed = false;
        setPoseD(-51.49, 49.028, 180-38.616);
        targetX = -72.0;
        targetY = 72.0;
    }
    
    public void resetRed() {
        isAutoRed = true;
        setPoseD(51.49, 49.028, 38.616);
        targetX = 72.0;
        targetY = 72.0;
    }
    
    

    ////////////////////////////////////////////////////////////////////////
    // #####################################################################
    ////////////////////////////////////////////////////////////////////////
    //
    // REPORTING / TELEMETRY
    //
    ////////////////////////////////////////////////////////////////////////
    // #####################################################################
    ////////////////////////////////////////////////////////////////////////


    /*********************
     * reportPositionData
     * - Tell the driver station where you think you are
     * .
     * Note: Put whatever you think would be helpful here. This is
     * called during various of the loops that are intended for
     * auto
     *********************/


    public void reportPositionData(boolean updateAtEnd) {
        /*
         * gets the current Position (x & y in mm, and heading in degrees) of the robot,
         * and prints it.
         */
        String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", poseEst.position.x, poseEst.position.y,
                poseEst.heading * 180 / Math.PI);
        opMode.telemetry.addData("Position", data);
        opMode.telemetry.addData("Status", odo.getDeviceStatus());


        opMode.telemetry.addData("Pinpoint Frequency", odo.getFrequency()); // prints/gets the current refresh rate of
        // the Pinpoint
        if (updateAtEnd) {
            opMode.telemetry.update();
        }
    }


    public void reportPositionData() {
        boolean updateAtEnd = true;
        reportPositionData(updateAtEnd);
    }

    public void reportPositionDataWithoutUpdate() {
        boolean updateAtEnd = false;
        reportPositionData(updateAtEnd);
    }

    ////////////////////////////////////////////////////////////////////////
    // #####################################################################
    ////////////////////////////////////////////////////////////////////////
    //
    // ODOMETRY
    //
    ////////////////////////////////////////////////////////////////////////
    // #####################################################################
    ////////////////////////////////////////////////////////////////////////
    //
    // Odometry -- keeping track of where we are.
    //
    // This is VERY important for all the higher-level driving functions
    //
    ////////////////////////////////////


    /*********************
     * setPoseD
     * - tells the robot where it currently is on the field
     * (with heading in degrees)
     * .
     * Inputs:
     * x - x position on the field (in inches)
     * y - y position on the field (in inches)
     * headingD - heading (in degrees)
     *********************/


    public void setPoseD(double x, double y, double headingD) {
        // distances in inches
        RoPose currentPose = RoPose.RoPoseD(x, y, headingD);
        setPose(currentPose);
    }

    public void setPose(RoPose currentPose) {
        odo.update();
        Pose2D goBildaPos = odo.getPosition();
        double X = goBildaPos.getX(DistanceUnit.INCH);
        double Y = goBildaPos.getY(DistanceUnit.INCH);
        double heading = goBildaPos.getHeading(AngleUnit.RADIANS);
        RoPose relPoseEst = new RoPose(X, Y, heading);
        poseBase = currentPose.then(relPoseEst.inverse());
        isPoseBaseInitialized = true;
        updatePose();
    }


    /*********************
     * updatePose
     * - tells the robot to get updated pose information from the
     * pinpoint odometry computer
     * .
     * Note: This sets poseEst, which is then used by all the
     * higher-level driving functions as the "current" or "most
     * recent" estimate of the robot's pose.
     *********************/


    public void updatePose() {
        odo.update();
        Pose2D goBildaPos = odo.getPosition();
        double x = goBildaPos.getX(DistanceUnit.INCH);
        double y = goBildaPos.getY(DistanceUnit.INCH);
        double heading = goBildaPos.getHeading(AngleUnit.RADIANS);
        RoPose relPoseEst = new RoPose(x, y, heading);
        poseEst = poseBase.then(relPoseEst);
        poseList.add(poseEst);
    }


    ////////////////////////////////////////////////////////////////////////
    // #####################################################################
    ////////////////////////////////////////////////////////////////////////
    //
    // BASIC TANK DRIVE
    //
    ////////////////////////////////////////////////////////////////////////
    // #####################################################################
    ////////////////////////////////////////////////////////////////////////
    //
    // This is used by the higher-level modified Pure Pursuit functions
    // IMPORTANT: This is also used for turning!
    //
    ////////////////////////////////////


    /*********************
     * setLeftRightDrivePower
     * - set the left and right motor powers
     * .
     * Inputs:
     * leftPower - requested forward velocity on left side of the robot
     * rightPower - requested forward velocity on left side of the robot
     * .
     * Note: This function checks to see if either "power" is outside
     * the standard FTC range of (-1,1). If so, then it *scales*
     * the velocities so that the bigger one is still in range. If
     * you just *clipped* the velocities, then the ratio between
     * left and right powers would change, and that would change the
     * rate of turning!
     *********************/
    public void setLeftRightDrivePower(double leftPower, double rightPower) {
        // find max absolute value if it is over 1.0
        double maxAbsoluteValue = 1.0;
        maxAbsoluteValue = Math.max(maxAbsoluteValue, Math.abs(leftPower));
        maxAbsoluteValue = Math.max(maxAbsoluteValue, Math.abs(rightPower));


        // scale down
        leftPower /= maxAbsoluteValue;
        rightPower /= maxAbsoluteValue;


        // set power levels
        frontLeft.setPower(leftPower);
        backLeft.setPower(leftPower);
        backRight.setPower(rightPower);
        frontRight.setPower(rightPower);
    }


    /*********************
     * setForwardTurnDrivePower
     * - set the average forward power and the right/left power difference
     * .
     * Inputs:
     * vC - requested forward velocity at center of robot
     * vDiff - requested velocity difference between the two wheels
     *         = vR-vL: positive means turning left / pos angle / counter-clockwise
     *********************/
    public void setForwardTurnDrivePower(double vC, double vDiff) {
        double vR = vC + (vDiff / 2);
        double vL = vC - (vDiff / 2);
        setLeftRightDrivePower(vL, vR);
    }


    ////////////////////////////////////////////////////////////////////////
    // #####################################################################
    ////////////////////////////////////////////////////////////////////////
    //
    // BASIC MECANUM DRIVE
    //
    ////////////////////////////////////////////////////////////////////////
    // #####################################################################
    ////////////////////////////////////////////////////////////////////////
    //
    // This is used in all the "strafe" functions
    //
    ////////////////////////////////////


    /*********************
     * strafeDrive
     * - set the drive motors according to standard mecanum inputs
     * .
     * Inputs:
     * forward - average forward speed (positive = forward)
     * left - average left/right strafing speed (positive = left)
     * rotateccw - turning speed (positive = left)
     *********************/



    public void strafeDrive(double forward, double left, double rotateccw) {
        double strafeSign = (IS_MECANUM_XDRIVE) ? 1.0 : -1.0;
        updateBallDetector(forward);


        double backLeftPower   =  forward - rotateccw + left*strafeSign;
        double backRightPower  =  forward + rotateccw - left*strafeSign;
        double frontLeftPower  =  forward - rotateccw - left*strafeSign;
        double frontRightPower =  forward + rotateccw + left*strafeSign;


        // adjust for weight distribution to improve strafing
       /*
       backRightPower  *= BACK_DRIVE_MULTIPLIER;
       backLeftPower   *= BACK_DRIVE_MULTIPLIER;
       frontRightPower *= FRONT_DRIVE_MULTIPLIER;
       frontLeftPower  *= FRONT_DRIVE_MULTIPLIER;
*/
        // find max absolute value if it is over 1.0
        double maxAbsoluteValue = 1.0;
        maxAbsoluteValue = Math.max(maxAbsoluteValue, Math.abs(backLeftPower));
        maxAbsoluteValue = Math.max(maxAbsoluteValue, Math.abs(backRightPower));
        maxAbsoluteValue = Math.max(maxAbsoluteValue, Math.abs(frontLeftPower));
        maxAbsoluteValue = Math.max(maxAbsoluteValue, Math.abs(frontRightPower));


        // scale down so that maxAbsoluteValue becomes 1.0
        double multiplier = 1.0 / maxAbsoluteValue;
        backLeft.setPower(backLeftPower * multiplier);
        backRight.setPower(backRightPower * multiplier);
        frontLeft.setPower(frontLeftPower * multiplier);
        frontRight.setPower(frontRightPower * multiplier);
    }


    ////////////////////////////////////////////////////////////////////////
    // #####################################################################
    ////////////////////////////////////////////////////////////////////////
    //
    // TURNING
    //
    ////////////////////////////////////////////////////////////////////////
    // #####################################################################
    ////////////////////////////////////////////////////////////////////////
    //
    // These functions are for turning in place.
    //
    // The strafeToPoseXXX functions can do similar things, but these
    // functions do a better job of controlling the turn.
    //
    ////////////////////////////////////


    ////////////////////////////////////
    //
    // TURNING: IN RADIANS
    //
    ////////////////////////////////////

    public double headingToFaceTargetD(double targetX, double targetY) {
        double robotX = poseEst.position.x;
        double robotY = poseEst.position.y;
        double headingToFaceTarget = Math.atan2(targetY - robotY, targetX - robotX);
        double headingToFaceTargetInDegrees = headingToFaceTarget /Math.PI * 180;
        return headingToFaceTargetInDegrees;
    }

    public double deltaHeadingToFaceTargetD(double targetX, double targetY) {
        double headingToFaceTargetInDegrees = headingToFaceTargetD(targetX, targetY);
        double robotHeadingD = poseEst.heading /Math.PI * 180;
        double delta = headingToFaceTargetInDegrees - robotHeadingD;
        while (delta > 180) {
            delta -= 360;
        }
        while (delta < -180) {
            delta += 360;
        }
        return delta;
    }


    public double distanceToTarget() {
        RoVector2D target = new RoVector2D(targetX,targetY);
        double dist = poseEst.position.sub(target).length();
        return dist;
    }






    /*********************
     * turnToHeading
     * - turn in place to a heading specified in RADIANS
     * .
     * Inputs:
     * heading - target heading (in radians)
     * vDiffMax - largest vDiff to allow
     * slowAngle - start slowing down once robot heading is this close (in radians)
     * threshold - stop once robot heading is this close (in radians)
     * timeoutS - timeout in seconds
     * .
     * Notes:
     * .
     * - After slowAngle is met, robot attempts to slow down at a
     * constant deceleration while continuing to turn toward the
     * target heading. The goal is to make a smooth stop without
     * over-turning.
     * .
     * - HOWEVER, when the robot gets to the threshold this function
     * returns WITHOUT turning off the motors, assuming that that
     * next function will take over control of the robot motion.
     * This is to avoid jerky start-stop movements. If the robot
     * absolutely needs to stop at the end, use
     * turnToHeadingAndStop.
     * .
     *********************/


    public void turnToHeading(
            double heading,
            double vDiffMax,
            double slowAngle,
            double threshold,
            double timeoutS) {
        ElapsedTime timer = new ElapsedTime();
        while (opMode.opModeIsActive() && timer.seconds() < timeoutS) {
            opMode.telemetry.addData("time", timer.seconds());
            opMode.telemetry.update();


            updatePose();
            double dHeading = RoPose.normalizeAngle(heading - poseEst.heading);
            Log.d("turnToHeading", "target heading: " + heading);
            Log.d("turnToHeading", "pose heading: " + poseEst.heading);
            Log.d("turnToHeading", "dHeading: " + dHeading);
            if (Math.abs(dHeading) < threshold) {
                Log.d("turnToHeading", "turn complete");
                break;
            }
            double ratioSqrt = Math.sqrt(Math.abs(dHeading / slowAngle));
            double clippedSqrt = Range.clip(ratioSqrt, 0.05 / vDiffMax, 1);
            double vDiffMin = 0.4; // value of 0.3 was too small and the robot got stuck one turn
            double vDiff = vDiffMax * clippedSqrt * Math.signum(dHeading);
            if (Math.abs(vDiff) < vDiffMin) {
                vDiff = vDiffMin * Math.signum(dHeading);
            }
            String message = "Turning! ";
            message += " poseEst: " + poseEst;
            message += String.format(", (vDiff)=(%.2f)", vDiff);
            message += String.format(", (dHeading, ratioSqrt, clippedSqrt)=(%.2f,%.2f,%.2f)", dHeading, ratioSqrt,
                    clippedSqrt);
            Log.d("turnToHeading", message);
            setForwardTurnDrivePower(0, vDiff);


            reportPositionData();
        }
    }


    /*********************
     * turnToHeading
     * - As above, except not timeoutS parameter (defaults to 90 seconds)
     *********************/


    public void turnToHeading(
            double heading,
            double vDiffMax,
            double slowAngle,
            double threshold) {
        double timeoutS = 90; // this is an excessively long timeout (in seconds)
        turnToHeading(heading, vDiffMax, slowAngle, threshold, timeoutS);
    }


    /*********************
     * turnToHeadingAndStop
     * - turn in place to a heading specified in RADIANS
     * - then STOP
     * .
     * Inputs:
     * headingD - target heading (in radians)
     * vDiffMax - largest vDiff to allow
     * slowAngleD - start slowing down once robot heading is this close (in radians)
     * thresholdD - stop once robot heading is this close (in radians)
     *********************/


    public void turnToHeadingAndStop(
            double heading,
            double vDiffMax,
            double slowAngle,
            double threshold) {
        turnToHeading(heading, vDiffMax, slowAngle, threshold);
        setLeftRightDrivePower(0, 0);
    }


    ////////////////////////////////////
    //
    // TURNING: IN DEGREES
    //
    ////////////////////////////////////


    /*********************
     * turnToHeadingD
     * - turn in place to a heading specified in DEGREES
     * .
     * Inputs:
     * headingD - target heading (in degrees)
     * vDiffMax - largest vDiff to allow
     * slowAngleD - start slowing down once robot heading is this close (in degrees)
     * thresholdD - stop once robot heading is this close (in degrees)
     * .
     * Note: This is essentially the same as turnToHeading, but all
     * angles are specified in degrees.
     *********************/


    public void turnToHeadingD(
            double headingD,
            double vDiffMax,
            double slowAngleD,
            double thresholdD) {
        double heading = headingD * Math.PI / 180;
        double slowAngle = slowAngleD * Math.PI / 180;
        double threshold = thresholdD * Math.PI / 180;
        turnToHeading(heading, vDiffMax, slowAngle, threshold);
    }


    /*********************
     * turnToHeadingD
     * - turn in place to a heading specified in DEGREES, using DRIVE PARAMETERS
     * .
     * Inputs:
     * headingD - target heading (in degrees)
     * params - preset drive parameters
     *********************/


    public void turnToHeadingD(
            double headingD,
            DriveParameters params) {
        turnToHeadingD(headingD, params.vDiffMax, params.slowAngleD, params.thresholdD);
    }


    /*********************
     * turnToHeadingDAndStop
     * - turn in place to a heading specified in DEGREES
     * - then STOP
     * .
     * Inputs:
     * headingD - target heading (in degrees)
     * vDiffMax - largest vDiff to allow
     * slowAngleD - start slowing down once robot heading is this close (in degrees)
     * thresholdD - stop once robot heading is this close (in degrees)
     * .
     * Note: This is essentially the same as turnToHeadingD, but the
     * motors are told to stop at the end.
     *********************/


    public void turnToHeadingDAndStop(
            double headingD,
            double vDiffMax,
            double slowAngleD,
            double thresholdD) {
        turnToHeadingD(headingD, vDiffMax, slowAngleD, thresholdD);
        setLeftRightDrivePower(0, 0);
    }


    /*********************
     * turnToHeadingDAndStop
     * - turn in place to a heading specified in DEGREES, using DRIVE PARAMETERS
     * - then STOP
     * .
     * Inputs:
     * headingD - target heading (in degrees)
     * params - preset drive parameters
     *********************/


    public void turnToHeadingDAndStop(
            double headingD,
            DriveParameters params) {
        turnToHeadingD(headingD, params);
        setLeftRightDrivePower(0, 0);
    }


    ////////////////////////////////////////////////////////////////////////
    // #####################################################################
    ////////////////////////////////////////////////////////////////////////
    //
    // HIGH-LEVEL STRAFING
    //
    ////////////////////////////////////////////////////////////////////////
    // #####################################################################
    ////////////////////////////////////////////////////////////////////////
    //
    // These are high-level functions for driving to a specified pose
    // using strafing / mecanum primitives.
    //
    // Basically, you tell it what pose to go to and the robot goes
    // there.
    //
    // In normal usage, you would tell it to go to a number of
    // waypoints in sequence without stopping, then stop at the end
    // when you get to the final destination.
    //
    // The robot will head straight for its destination, but it will
    // also turn toward the final target heading while it is driving!
    // The effect is not like anything a human would do.
    //
    // It can also be slow if you end up driving a long distance while
    // the robot is not turned in the direction it is driving. A
    // better option is often to tell the robot to strafe drive almost
    // all the way to the end with a target heading that lines up with
    // the direction the robot needs to go, then only tell it to turn
    // to the final heading in the short distance from that waypoint
    // until the end. With a bit of work, we could write a function
    // to do that automatically, but that isn't written yet so for now
    // we have to set the waypoints by hand.
    //
    ////////////////////////////////////


    ////////////////////////////////////
    //
    // STRAFING: INNER LOOP CONTENTS (you probably don't call this directly)
    //
    ////////////////////////////////////


    /*********************
     * stepTowardPose
     * .
     * - tells the motors what to do at this loop iteration in order
     * to follow a straight line toward the target x,y position
     * while also turning toward the target heading
     * .
     * Inputs:
     * target - target pose
     * vCMax - max allowed center-of-robot velocity
     * vDiffMax - max allowed left/right velocity difference
     * .
     * Returns:
     * robotPoseInTargetPOV -- where the robot is from the point of
     * view of the target. This is returned as a convenience to
     * the calling loop, as it may be helpful for deciding when to
     * do things such as stop.
     * .
     * Notes:
     * .
     * - Given current robot pose, determines how current motors
     * should be set in order to eventually get to target pose
     * .
     * - This function just tells the robot what to do at this
     * moment in time. In order to actually end up at (or near)
     * the target, you will need to call this function repeatedly
     * in a loop and decide when to stop.
     * .
     *********************/


    public RoPose stepStrafeTowardPose(
            RoPose target,
            double vCMax,
            double vDiffMax) {
        updatePose();
        RoPose drivingPose = this.poseEst;
        // getting the target's pose in robot coordinates.
        RoPose relPose = drivingPose.getRelativeRoPoseFor(target);
        RoPose robotPoseInTargetPOV = relPose.inverse();
        // setting the coordinates, make typing easier
        double x = relPose.position.x;
        double y = relPose.position.y;
        double heading = relPose.heading; // heading in radians


        double tenDegrees = 10 * Math.PI / 180;
        double weight = 10 / tenDegrees; // this is the weight that makes 10 deg = 1 inch
        double relLength = relPose.length(weight);
        double weightedHeading = heading * weight;


        double vX = x / relLength * vCMax; // forward velocity (on robot's x-axis)
        double vDiff = weightedHeading / relLength * vDiffMax; // turning speed
        double vY = y / relLength * vCMax; // left strafe velocity (on robot's y-axis)


        String message = "StrafeDriving! ";
        message += " poseEst: " + poseEst;
        message += " relPose: " + relPose;
        message += String.format(", (vX, vY, vDiff)=(%.2f,%.2f,%.2f)", vX, vY, vDiff);
        Log.d("stepStrafeTowardPose", message);


        strafeDrive(vX, vY, vDiff);


        reportPositionData();


        return robotPoseInTargetPOV;
    }


    /*********************
     * stepStrafeTowardPose
     * - same as above, but using DRIVE PARAMETERS
     * .
     * Inputs:
     * target - target pose
     * params - preset drive parameters
     *********************/


    public void stepStrafeTowardPose(
            RoPose target,
            DriveParameters params) {
        stepStrafeTowardPose(target, params.vCMax, params.vDiffMax);
    }


    ////////////////////////////////////
    //
    // STRAFING: GO TO POSE AND STOP
    //
    ////////////////////////////////////


    /*********************
     * strafeToPoseAndStop
     * - Sets up a loop to repeatedly call stepStrafeTowardPose
     * - takes care of slowing down and stopping when the robot
     * gets near the target
     * .
     * Inputs:
     * target - target pose
     * vCMax - max allowed center-of-robot velocity
     * vDiffMax - max allowed left/right velocity difference
     * slowDistance - distance to start slowing before stop
     * threshold - stop when robot gets this close
     * .
     *********************/


    public boolean obstructionTest(int cycleThreshold) {
        if (poseList.size() <= cycleThreshold) {
            //Its too early to make a decision if its obstructed
            return false;
        }
        int maxIndex = poseList.size() - 1;
        for (int i = 0; i < DRIVING_OBSTRUCTION_CYCLES; i++) {
            RoPose poseI = poseList.get(maxIndex - i);
            if (poseI.isWithin(poseEst, DRIVING_OBSTRUCTION_DISTANCE) == false) {
                //we moved so we are not obstructed, therefore returning false
                return false;                }
        }
        //Did not move on any recent cycles, obstruction is true
        return true;
    }


    public void strafeToPoseAndStop(
            RoPose target,
            double vCMax,
            double vDiffMax,
            double slowDistance,
            double threshold) {
        int cycleThreshold = poseList.size() + DRIVING_OBSTRUCTION_CYCLES;
        double vC = vCMax;
        while (opMode.opModeIsActive()) {
            if (obstructionTest(cycleThreshold) == true) {
                break;
            }
            RoPose robotPoseInTargetPOV = stepStrafeTowardPose(target, vC, vDiffMax);
            double tenDegrees = 10 * Math.PI / 180;
            double weight = 10 / tenDegrees; // this is the weight that makes 10 deg = 1 inch
            double relLength = robotPoseInTargetPOV.length(weight);
            double ratioSqrt = Math.sqrt(Math.abs(relLength / slowDistance));
            double clippedSqrt = Range.clip(ratioSqrt, 0.05 / vCMax, 1);
            vC = vCMax * clippedSqrt;
            if (relLength < threshold) {
                break;
            }
        }
        setLeftRightDrivePower(0, 0);
    }


    /*********************
     * strafeToPoseAndStop
     * - same as above, but using DRIVE PARAMETERS
     * .
     * Inputs:
     * target - target pose
     * params - preset drive parameters
     *********************/


    public void strafeToPoseAndStop(
            RoPose target,
            DriveParameters driveParams) {
        strafeToPoseAndStop(target,
                driveParams.vCMax,
                driveParams.vDiffMax,
                driveParams.slowDistance,
                driveParams.threshold);
    }

    /*********************
     * strafeToPoseAndStopD
     * - same as above, but using DRIVE PARAMETERS and DEGREES
     * .
     * Inputs:
     * target_x - x coordinate of target pose
     * target_y - y coordinate of target pose
     * targetHeadingD - target heading (in degrees)
     * params - preset drive parameters
     *********************/


    public void strafeToPoseAndStopD(
            double target_x,
            double target_y,
            double targetHeadingD,
            DriveParameters driveParams) {
        RoPose target = RoPose.RoPoseD(target_x, target_y, targetHeadingD);
        strafeToPoseAndStop(target, driveParams);
    }


    ////////////////////////////////////
    //
    // STRAFING: GO TO POSE AND CONTINUE
    //
    ////////////////////////////////////


    /*********************
     * strafeToPoseAndContinue
     * - Sets up a loop to repeatedly call stepStrafeTowardPose
     * - Does *NOT* slow down and stop at the end
     * .
     * Inputs:
     * target - target pose
     * vCMax - max allowed center-of-robot velocity
     * vDiffMax - max allowed left/right velocity difference
     * threshold - stop when robot gets this close
     * .
     * Note: When robot gets near target this function simply returns
     * with the robot still moving. This assumes the next function
     * in the auto sets a new target to drive to, and that eventually
     * one of them will stop.
     *********************/


    public void strafeToPoseAndContinue(
            RoPose target,
            double vCMax,
            double vDiffMax,
            double threshold) {
        int cycleThreshold = poseList.size() + DRIVING_OBSTRUCTION_CYCLES;
        while (opMode.opModeIsActive()) { // to do!!! check if op mode is active.
            if (obstructionTest(cycleThreshold) == true) {
                break;
            }
            RoPose robotPoseInTargetPOV = stepStrafeTowardPose(target, vCMax, vDiffMax);
            double tenDegrees = 10 * Math.PI / 180;
            double weight = 10 / tenDegrees; // this is the weight that makes 10 deg = 1 inch
            double relLength = robotPoseInTargetPOV.length(weight);
            if (relLength < threshold) {
                break;
            }
        }
    }


    /*********************
     * strafeToPoseAndContinue
     * - same as above, but using DRIVE PARAMETERS
     * .
     * Inputs:
     * target - target pose
     * params - preset drive parameters
     *********************/


    public void strafeToPoseAndContinue(
            RoPose target,
            DriveParameters driveParams) {
        strafeToPoseAndContinue(target,
                driveParams.vCMax,
                driveParams.vDiffMax,
                driveParams.threshold);
    }




    /*********************
     * strafeToPoseAndContinueD
     * .
     * - Sets up a loop to repeatedly call stepStrafeTowardPose
     * .
     * - Does *NOT* slow down and stop at the end
     * .
     * - this is basically the same as strafeToPoseAndContinue, but
     * angles are in DEGREES. Since the Pose class uses radians,
     * this ends up meaning that the target pose is specified as
     * three parameters instead of packaged into a Pose.
     * .
     * Inputs:
     * target_x - x coordinate of target pose
     * target_y - y coordinate of target pose
     * targetHeadingD - target heading (in degrees)
     * vCMax - max allowed center-of-robot velocity
     * vDiffMax - max allowed left/right velocity difference
     * threshold - stop when robot gets this close
     * .
     * Note: When robot gets near target this function simply returns
     * with the robot still moving. This assumes the next function
     * in the auto sets a new target to drive to, and that eventually
     * one of them will stop.
     *********************/


    public void strafeToPoseAndContinueD(
            double target_x,
            double target_y,
            double targetHeadingD,
            double vCMax,
            double vDiffMax,
            double threshold) {
        RoPose target = RoPose.RoPoseD(target_x, target_y, targetHeadingD);
        strafeToPoseAndContinue(target, vCMax, vDiffMax, threshold);
    }


    /*********************
     * strafeToPoseAndContinue
     * - same as above, but using DRIVE PARAMETERS
     * .
     * Inputs:
     * target_x - x coordinate of target pose
     * target_y - y coordinate of target pose
     * targetHeadingD - target heading (in degrees)
     * params - preset drive parameters
     *********************/


    public void strafeToPoseAndContinueD(
            double target_x,
            double target_y,
            double targetHeadingD,
            DriveParameters driveParams) {
        RoPose target = RoPose.RoPoseD(target_x, target_y, targetHeadingD);
        strafeToPoseAndContinue(target, driveParams);
    }


    ////////////////////////////////////////////////////////////////////////
    // #####################################################################
    ////////////////////////////////////////////////////////////////////////
    //
    // MODIFIED PURE PURSUIT
    //
    ////////////////////////////////////////////////////////////////////////
    // #####################################################################
    ////////////////////////////////////////////////////////////////////////
    //
    // These are high-level functions for driving to a specified pose
    // using only tank-drive primitives.
    //
    // Basically, you tell it what pose to go to and the robot goes
    // there using only driving forward (or backward if you tell it
    // to), and turning either in place or while it drives. NO
    // STRAFING.
    //
    // In normal usage, you would tell it to go to a number of
    // waypoints in sequence without stopping, then stop at the end
    // when you get to the final destination.
    //
    // This makes for smooth, curling paths, and if the the waypoints
    // are chosen carefully, it can be noticeably faster than using
    // the equivalent Strafing function to get to the same point. On
    // the other hand, strafing paths are easier to set up, so you may
    // not feel the need to use these functions.
    //
    // Note: These functions were originally developed for
    // Centerstage. That year the RoBusted robot was a tank drive
    // and hand no odometry wheels (!), so it was necessary to
    // develop these more elaborate driving solutions in order to
    // have a good auto.
    //
    ////////////////////////////////////


    ////////////////////////////////////
    //
    // PURE PURSUIT: INNER LOOP CONTENTS (you probably don't call this directly)
    //
    ////////////////////////////////////


    /*********************
     * stepTowardPose
     * - tells the motors what to do at this loop iteration in order to follow the
     * path
     * .
     * Inputs:
     * target - target pose
     * vCMax - max allowed center-of-robot velocity
     * vDiffMax - max allowed left/right velocity difference
     * alignmentThreshold - if heading is too far off target, just turn robot
     * direction - try to match pose by driving forward or in reverse? (They are
     * different!)
     * .
     * Returns:
     * robotPoseInTargetPOV -- where the robot is from the point of
     * view of the target. This is returned as a convenience to
     * the calling loop, as it may be helpful for deciding when to
     * do things such as stop.
     * .
     * Notes:
     * .
     * - Given current robot pose, determines how current motors
     * should be set in order to eventually get to target,
     * following a modified pure pursuit algorithm.
     * .
     * - However, near the end of the path, achieving something
     * close to the right heading is often more important than
     * achieving the right (x,y) position. Once robot is within
     * alignmentThreshold of the target, extra weight is given to
     * the heading and less to the (x,y) position.
     * .
     * - In either case, this function just tells the robot what to
     * do at this moment in time. In order to actually follow the
     * pure-pursuit path, you will need to call this function
     * repeatedly in a loop and decide when to stop.
     *********************/


    public RoPose stepTowardPose(
            RoPose target,
            double vCMax,
            double vDiffMax,
            double alignmentThreshold,
            DRIVE_DIRECTION direction) {
        updatePose();
        RoPose drivingPose = this.poseEst;
        if (direction == DRIVE_DIRECTION.REVERSE) {
            drivingPose = drivingPose.thenRotate(Math.PI);
            Log.d("stepTowardPose", "in reverse");
        } else {
            Log.d("stepTowardPose", "going forward");
        }
        // getting the target's pose in robot coordinates.
        RoPose relPose = drivingPose.getRelativeRoPoseFor(target);
        RoPose robotPoseInTargetPOV = relPose.inverse();
        // setting the coordinates, make typing easier
        double x = relPose.position.x;
        double y = relPose.position.y;
        double targetDist = Math.sqrt(x * x + y * y);
        double trackWidth = TRACK_WIDTH;
        double pointDist = Math.min(targetDist / 2, targetDist - trackWidth);
        RoPose pointPose = relPose.thenForward(-pointDist);
        double pointX = pointPose.position.x;
        double pointY = pointPose.position.y;
        double vC = 0;
        double vDiff;
        if (2 * pointX < Math.abs(pointY)) {
            // turn
            vDiff = vDiffMax * Math.signum(pointY);
            String message = "Turning! ";
            message += " poseEst: " + poseEst;
            message += " relPose: " + relPose;
            message += String.format(", (vC, vDiff)=(%.2f,%.2f)", vC, vDiff);
            Log.d("stepTowardPose", message);


        } else {
            // drive
            vC = vCMax;
            if (robotPoseInTargetPOV.position.x > -alignmentThreshold) {
                double headingScale = 7.5;
                vDiff = relPose.heading * vC * headingScale;
            } else {
                vDiff = (vC * 2 * pointY * trackWidth) / (pointX * pointX + pointY * pointY);
            }
            String message = "Driving! ";
            message += " poseEst: " + poseEst;
            message += " relPose: " + relPose;
            message += String.format(", (vC, vDiff)=(%.2f,%.2f)", vC, vDiff);
            message += String.format(", (pointDist, pointX, pointY)=(%.2f,%.2f,%.2f)", pointDist, pointX, pointY);
            Log.d("stepTowardPose", message);
        }
        if (direction == DRIVE_DIRECTION.FORWARD) {
            setForwardTurnDrivePower(vC, vDiff);
        } else {
            setForwardTurnDrivePower(-vC, vDiff);
        }


        reportPositionData();


        return robotPoseInTargetPOV;
    }


    ////////////////////////////////////
    //
    // PURE PURSUIT: GO TO POSE AND STOP
    //
    ////////////////////////////////////


    /*********************
     * goToPoseAndStop
     * - Sets up a loop to repeatedly call stepTowardPose
     * - takes care of slowing down and stopping when the robot
     * gets near the target
     * .
     * Inputs:
     * target - target pose
     * vCMax - max allowed center-of-robot velocity
     * vDiffMax - max allowed left/right velocity difference
     * slowDistance - distance to start slowing before stop
     * threshold - stop when robot gets this close
     * direction - try to match pose by driving forward or in reverse? (They are
     * different!)
     * .
     *********************/


    public void goToPoseAndStop(
            RoPose target,
            double vCMax,
            double vDiffMax,
            double slowDistance,
            double threshold,
            DRIVE_DIRECTION direction) {
        double vC = vCMax;
        while (opMode.opModeIsActive()) {
            RoPose robotPoseInTargetPOV = stepTowardPose(target, vC, vDiffMax, slowDistance / 2, direction);
            double x = robotPoseInTargetPOV.position.x;
            double y = robotPoseInTargetPOV.position.y;
            double targetDist = Math.sqrt(x * x + y * y);
            double ratioSqrt = Math.sqrt(Math.abs(targetDist / slowDistance));
            double clippedSqrt = Range.clip(ratioSqrt, 0.05 / vCMax, 1);
            vC = vCMax * clippedSqrt;
            if (x > -threshold) {
                break;
            }
        }
        setLeftRightDrivePower(0, 0);
    }


    /*********************
     * goToPoseAndStop
     * - same as above, but using DRIVE PARAMETERS
     * .
     * Inputs:
     * target - target pose
     * params - preset drive parameters
     *********************/


    public void goToPoseAndStop(
            RoPose target,
            DriveParameters driveParams) {
        goToPoseAndStop(target,
                driveParams.vCMax,
                driveParams.vDiffMax,
                driveParams.slowDistance,
                driveParams.threshold,
                driveParams.direction);
    }


    /*********************
     * goToPoseAndStopD
     * .
     * - Sets up a loop to repeatedly call stepTowardPose
     * .
     * - takes care of slowing down and stopping when the robot gets
     * near the target
     * .
     * - this is basically the same as goToPoseAndStop, but angles
     * are in DEGREES. Since the Pose class uses radians, this
     * ends up meaning that the target pose is specified as three
     * parameters instead of packaged into a Pose.
     * .
     * Inputs:
     * target_x - x coordinate of target pose
     * target_y - y coordinate of target pose
     * targetHeadingD - target heading (in degrees)
     * vCMax - max allowed center-of-robot velocity
     * vDiffMax - max allowed left/right velocity difference
     * slowDistance - distance to start slowing before stop
     * threshold - stop when robot gets this close
     * direction - try to match pose by driving forward or in reverse? (They are
     * different!)
     * .
     *********************/


    public void goToPoseAndStopD(
            double target_x,
            double target_y,
            double targetHeadingD,
            double vCMax,
            double vDiffMax,
            double slowDistance,
            double threshold,
            DRIVE_DIRECTION direction) {
        RoPose target = RoPose.RoPoseD(target_x, target_y, targetHeadingD);
        goToPoseAndStop(target, vCMax, vDiffMax, slowDistance, threshold, direction);
    }


    /*********************
     * goToPoseAndStopD
     * - same as above, but using DRIVE PARAMETERS
     * .
     * Inputs:
     * target_x - x coordinate of target pose
     * target_y - y coordinate of target pose
     * targetHeadingD - target heading (in degrees)
     * params - preset drive parameters
     *********************/


    public void goToPoseAndStopD(
            double target_x,
            double target_y,
            double targetHeadingD,
            DriveParameters driveParams) {
        RoPose target = RoPose.RoPoseD(target_x, target_y, targetHeadingD);
        goToPoseAndStop(target, driveParams);
    }


    ////////////////////////////////////
    //
    // PURE PURSUIT: GO TO POSE AND CONTINUE
    //
    ////////////////////////////////////


    /*********************
     * goToPoseAndContinue
     * - Sets up a loop to repeatedly call stepTowardPose
     * - Does *NOT* slow down and stop at the end
     * .
     * Inputs:
     * target - target pose
     * vCMax - max allowed center-of-robot velocity
     * vDiffMax - max allowed left/right velocity difference
     * threshold - stop when robot gets this close
     * direction - try to match pose by driving forward or in reverse? (They are
     * different!)
     * .
     * Note: When robot gets near target this function simply returns
     * with the robot still moving. This assumes the next function
     * in the auto sets a new target to drive to, and that eventually
     * one of them will stop.
     *********************/


    public void goToPoseAndContinue(
            RoPose target,
            double vCMax,
            double vDiffMax,
            double threshold,
            DRIVE_DIRECTION direction) {
        while (opMode.opModeIsActive()) { // to do!!! check if op mode is active.
            RoPose robotPoseInTargetPOV = stepTowardPose(target, vCMax, vDiffMax, threshold, direction);
            double x = robotPoseInTargetPOV.position.x;
            if (x > -threshold) {
                break;
            }
        }
    }


    /*********************
     * goToPoseAndContinue
     * - same as above, but using DRIVE PARAMETERS
     * .
     * Inputs:
     * target - target pose
     * params - preset drive parameters
     *********************/


    public void goToPoseAndContinue(
            RoPose target,
            DriveParameters driveParams) {
        goToPoseAndContinue(target,
                driveParams.vCMax,
                driveParams.vDiffMax,
                driveParams.threshold,
                driveParams.direction);
    }


    /*********************
     * goToPoseAndContinueD
     * .
     * - Sets up a loop to repeatedly call stepTowardPose
     * .
     * - Does *NOT* slow down and stop at the end
     * .
     * - this is basically the same as goToPoseAndContinue, but
     * angles are in DEGREES. Since the Pose class uses radians,
     * this ends up meaning that the target pose is specified as
     * three parameters instead of packaged into a Pose.
     * .
     * Inputs:
     * target_x - x coordinate of target pose
     * target_y - y coordinate of target pose
     * targetHeadingD - target heading (in degrees)
     * vCMax - max allowed center-of-robot velocity
     * vDiffMax - max allowed left/right velocity difference
     * threshold - stop when robot gets this close
     * direction - try to match pose by driving forward or in reverse? (They are
     * different!)
     * .
     * Note: When robot gets near target this function simply returns
     * with the robot still moving. This assumes the next function
     * in the auto sets a new target to drive to, and that eventually
     * one of them will stop.
     *********************/


    public void goToPoseAndContinueD(
            double target_x,
            double target_y,
            double targetHeadingD,
            double vCMax,
            double vDiffMax,
            double threshold,
            DRIVE_DIRECTION direction) {
        RoPose target = RoPose.RoPoseD(target_x, target_y, targetHeadingD);
        goToPoseAndContinue(target, vCMax, vDiffMax, threshold, direction);
    }


    /*********************
     * goToPoseAndContinueD
     * - same as above, but using DRIVE PARAMETERS
     * .
     * Inputs:
     * target_x - x coordinate of target pose
     * target_y - y coordinate of target pose
     * targetHeadingD - target heading (in degrees)
     * params - preset drive parameters
     *********************/


    public void goToPoseAndContinueD(
            double target_x,
            double target_y,
            double targetHeadingD,
            DriveParameters driveParams) {
        RoPose target = RoPose.RoPoseD(target_x, target_y, targetHeadingD);
        goToPoseAndContinue(target, driveParams);
    }


}


package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import android.util.Log;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import java.util.Locale;



@TeleOp(name = "Test: Flywheel", group = "Testing")

public class FlywheelTester extends LinearOpMode {
    public RoRobot robot = new RoRobot();
    public String target = "NO TARGET SET!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!";


    ////////////////////////////////////
    // State info
    ////////////////////////////////////

    public enum GateState {
        BASE,
        WAIT_FOR_GATE,
        WAIT_FOR_KICKER,
    }
    GateState gateState = GateState.BASE;

    public double time_when_pressed = 0;
    public double flywheel_speed = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.initializeTeleOp(hardwareMap, (LinearOpMode)this);

        telemetry.addLine("Successful Initialization");
        telemetry.update();
        
        waitForStart(); 
         while (opModeIsActive()) {
            myLoop();
        }
    }
    
    
    ////////////////////////////////////
    // OPMODE: loop
    ////////////////////////////////////

    public void myLoop() {
        telemetry.addLine("This is the flywheel tester");
        telemetry.addLine("Right joystick: change flywheel speed");
        telemetry.addLine("Press A to set the speed to 0");
        telemetry.addLine("Press Right Bumper to Fire");
        telemetry.addLine("");
        telemetry.addLine("To Initialize Position:");
        telemetry.addLine("Press X or Square for Red");
        telemetry.addLine("Press Y or Triangle for Blue");
        
        robot.intakeOn();
        // If you press the A button, then you reset the speed to be zero
        if (gamepad1.a) {
            flywheel_speed = 0;
        }
        flywheel_speed += -gamepad1.right_stick_y/5000;
        flywheel_speed = Range.clip(flywheel_speed, -1,1);
        robot.flywheelSetPower(flywheel_speed);


        double flywheel_velocity = robot.flywheelVelocity();
        
         if (gamepad1.xWasPressed()) {
            robot.setPoseD(36.8, 31.5, 90);
            target = "RED*RED*RED*RED*RED*RED*RED";
        }
        
        if (gamepad1.yWasPressed()) {
            robot.setPoseD(-36.25, 31.5, 180-90);
            target = "BLUE * BLUE * BLUE * BLUE * BLUE * BLUE";
        }
        

        ////////////////////////////////////
        // Shooting Process


        if (gateState == GateState.BASE) {
            if (gamepad1.rightBumperWasPressed()) {
                //Now we will transition to the next state
                time_when_pressed = getRuntime();
                robot.gateOpen();
                gateState = GateState.WAIT_FOR_GATE;
            }
        } else if (gateState == GateState.WAIT_FOR_GATE) {
            if (getRuntime() > time_when_pressed + robot.GATE_CLOSE_DELAY) {
                robot.gateClose();
                robot.kickerUp();
                gateState = GateState.WAIT_FOR_KICKER;
            }
        } else if (gateState == GateState.WAIT_FOR_KICKER) {
            if (getRuntime() > time_when_pressed + robot.KICKER_CLOSE_DELAY) {
                robot.kickerDown();
                gateState = GateState.BASE;
            }
        }

        ////////////////////////////////////
        // Telemetry Update

        double heading =  robot.poseEst.heading * 180 / Math.PI;
        
        telemetry.addLine("");
        telemetry.addData("Target:", target);
        telemetry.addLine("");
        robot.updatePose();
        String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", 
                robot.poseEst.position.x, 
                robot.poseEst.position.y,
                heading);
        telemetry.addData("Position", data);
        telemetry.addData("Flywheel Power:", flywheel_speed);
        telemetry.addData("Flywheel Velocity:", flywheel_velocity);
        telemetry.update();
        Log.d("FlywheelTester", "heading: " + heading);
    }
}
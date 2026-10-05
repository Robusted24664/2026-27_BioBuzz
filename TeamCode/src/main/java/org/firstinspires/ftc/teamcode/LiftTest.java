package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.robot.RobotState;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.Range;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="LiftTest", group="Testing")
public class LiftTest extends LinearOpMode {

    // Declare OpMode members.
    private Servo left1Lift = null;
    private Servo right1Lift = null;
    private Servo left2Lift = null;
    private Servo right2Lift = null;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables.
        left1Lift = hardwareMap.get(Servo.class, "left1Lift");
        right1Lift = hardwareMap.get(Servo.class, "right1Lift");
        left2Lift = hardwareMap.get(Servo.class, "left2Lift");
        right2Lift = hardwareMap.get(Servo.class, "right2Lift");

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        double left1LiftPos = 0.23;
        double right1LiftPos = 0.23;
        double left2LiftPos = 0.23;
        double right2LiftPos = 0.23;

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            
        if (gamepad1.xWasPressed()) {
            left1LiftPos = 0.23;
            right1LiftPos = 0.23;
            left2LiftPos = 0.23;
            right2LiftPos = 0.23;
        }
        
        if (gamepad1.yWasPressed()) {
            left1LiftPos = 0.7;
            right1LiftPos = 0.7;
            left2LiftPos = 0.7;
            right2LiftPos = 0.7;
        }
        
        left1LiftPos += gamepad1.left_stick_y / 5000;
        right1LiftPos += gamepad1.right_stick_y / 5000;
            
        left1LiftPos = Range.clip(left1LiftPos, 0.0, 1.0);
        right1LiftPos = Range.clip(right1LiftPos, 0.0 , 1.0);
        
        left1Lift.setPosition(left1LiftPos);
        right1Lift.setPosition(right1LiftPos);
        left2Lift.setPosition(left1LiftPos);
        right2Lift.setPosition(right1LiftPos);


            
            
            
            // Send telemetry data to the driver station
            telemetry.addData("leftLift Position Position", left1LiftPos);
            telemetry.addData("rightLift Position", right1LiftPos);


            telemetry.update();
        }
    }
}

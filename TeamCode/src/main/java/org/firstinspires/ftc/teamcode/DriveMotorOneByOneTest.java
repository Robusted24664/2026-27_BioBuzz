package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.RoRobot.DriveParameters;


@TeleOp(name = "DriveMotorOneByOneTest", group = "test")
public class DriveMotorOneByOneTest extends LinearOpMode {
    public RoRobot robot = new RoRobot();
    
    @Override
    public void runOpMode() throws InterruptedException {
        robot.initializeAuto(hardwareMap, (LinearOpMode)this);

        DriveParameters slowForward = robot.getDriveParametersForSlowForward();
        DriveParameters mediumForward = robot.getDriveParametersForMediumForward();
        DriveParameters fastForward = robot.getDriveParametersForFastForward();
        
        robot.setPoseD(0, 0, 0);
        
        robot.reportPositionData();

        waitForStart();
        
        telemetry.addLine("backLeft");
        telemetry.update();
        robot.backLeft.setPower(1);
        sleep(5000);
        robot.backLeft.setPower(0);

        telemetry.addLine("backRight");
        telemetry.update();
        robot.backRight.setPower(1);
        sleep(5000);
        robot.backRight.setPower(0);
        
        
        telemetry.addLine("frontLeft");
        telemetry.update();
        robot.frontLeft.setPower(1);
        sleep(5000);
        robot.frontLeft.setPower(0);

        telemetry.addLine("frontRight");
        telemetry.update();
        robot.frontRight.setPower(1);
        sleep(5000);
        robot.frontRight.setPower(0);

        
        //robot.strafeToPoseAndStopD(32,10,0, mediumForward);
        
        // Driving
        //while (opModeIsActive()) {

       // robot.strafeDrive(-gamepad1.left_stick_y, 
         //       gamepad1.left_stick_x, 
            //    gamepad1.right_stick_x); 
        //}
        sleep(1000);

    }
}
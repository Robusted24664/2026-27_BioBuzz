package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.RoRobot.DriveParameters;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled

@Autonomous(name = "Blue Shooter 2", group = "auto")
public class AutoBlueShooter2 extends LinearOpMode {
    public RoRobot robot = new RoRobot();
    
    @Override
    public void runOpMode() throws InterruptedException {
        robot.initializeAuto(hardwareMap, (LinearOpMode)this);

        DriveParameters slowForward = robot.getDriveParametersForSlowForward();
        DriveParameters mediumForward = robot.getDriveParametersForMediumForward();
        DriveParameters fastForward = robot.getDriveParametersForFastForward();
        DriveParameters turboForward = robot.getDriveParametersForTurbo();
        
        
        robot.setBlue();
        
        while (opModeInInit()) {
            robot.updatePose();
            robot.reportPositionData();
        }
        
        waitForStart();
        double start_time = getRuntime();
        
        robot.flywheelSetPower(0.52);
        
        robot.setBrakes();
        
        if (robot.poseEst.position.y < 0.0) {
            if (robot.poseEst.heading > -0.25 * Math.PI && robot.poseEst.heading < 0.25 * Math.PI) {
                RoPose TargetPose = robot.poseEst.thenForward(-10);
                robot.strafeToPoseAndStop(TargetPose, mediumForward);
                robot.strafeToPoseAndStopD(-15,-40,180-90, mediumForward);
            }
            robot.strafeToPoseAndStopD(-14,-11,180-90, fastForward);
        }
        
        robot.strafeToPoseAndStopD(-20.4,12.1,180-56+4+4+4, mediumForward);
        
        while (opModeIsActive() && getRuntime() < start_time + 3) {
            double flywheel_velocity = robot.flywheel.getVelocity();
            telemetry.addData("Flywheel Velocity:", flywheel_velocity);
            telemetry.update();
        }
        
        
        robot.autoFireSequence();
        //sleep(1200-750);
        
        robot.flywheelSetPower(0.53);
        
        robot.autoFireSequence();
        //sleep(1200-750);
        robot.autoFireSequence();
        //sleep(1000-750);
        robot.autoFireSequence();
        
        ////////////////////////////////////
        // UTILITY: Collect First Line
        ////////////////////////////////////
        robot.strafeToPoseAndStopD(-29.7,9.5+4,180-0.0, turboForward);
        robot.intakeOn();
        robot.strafeToPoseAndStopD(-53-3,9.5+4,180-0.0, slowForward);
        robot.flywheelSetPower(0.52);

        
        robot.strafeToPoseAndStopD(-20.4,12.1,180-56+4+2, fastForward);
        robot.flywheelSetPower(0.5);

        robot.autoFireSequence();
        sleep(1200-750);
        robot.autoFireSequence();
        sleep(1200-750);
        robot.autoFireSequence();
        sleep(1000-750);
        robot.autoFireSequence();
        
        ////////////////////////////////////
        // UTILITY: Collect Second Line
        ////////////////////////////////////
        robot.strafeToPoseAndStopD(-29.7,9.5-24+4-2.5,180-0.0, turboForward);
        robot.intakeOn();
        robot.strafeToPoseAndStopD(-55-4,9.5-24+4-2.5,180-0.0, slowForward);
        
        robot.strafeToPoseAndStopD(-40,9.5-24+4-2.5,180-0.0, turboForward);

        robot.flywheelSetPower(0.52);

        
        robot.strafeToPoseAndStopD(-20.4,12.1,180-56+4+2, fastForward);
        robot.flywheelSetPower(0.5);

        robot.autoFireSequence();
        sleep(1200-750);
        robot.autoFireSequence();
        sleep(1200-750);
        robot.autoFireSequence();
        sleep(1000-750);
        robot.autoFireSequence();
        
        ////////////////////////////////////
        // UTILITY: Collect Third Line
        ////////////////////////////////////
        robot.strafeToPoseAndStopD(-29.7,9.5-24-24+4,180-0.0, turboForward);
        robot.intakeOn();
        robot.strafeToPoseAndStopD(-55-4,9.5-24-24+4,180-0.0, slowForward);
        
        ////////////////////////////////////
        // UTILITY: Move to lever
        ////////////////////////////////////
        robot.strafeToPoseAndStopD(-40,-6+4,180-180, turboForward);
        
    }
}

package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveOdometry;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;
import edu.wpi.first.math.geometry;

import java.util.List;
import java.util.concurrent.TimeUnit;

@TeleOp(name="Berk AprilTag")

public class ApriltagAutoDrive extends LinearOpMode
{
    
    final double DESIRED_DISTANCE = 10.0; //inch

    
    final double m_SpeedKp  =  0.02;   
    final double m_StrafeKp =  0.015;   
    final double m_TurnKp   =  0.01;  

    final double MAX_AUTO_SPEED = 0.5;   
    final double MAX_AUTO_STRAFE= 0.5;   
    final double MAX_AUTO_TURN  = 0.3;   

    private DcMotor leftFrontDrive   = null;  
    private DcMotor rightFrontDrive  = null;  
    private DcMotor leftBackDrive    = null;  
    private DcMotor rightBackDrive   = null;  
    private IMU imu = null;
    public m_Pose = null;
    
    private static final boolean USE_WEBCAM = true; 
    private static final int DESIRED_TAG_ID = 10;     
    private VisionPortal visionPortal;               
    private AprilTagProcessor aprilTag;
    private AprilTagDetection desiredTag = null;

    
    
    @Override public void runOpMode()
    {
        boolean targetFound = false;    
        double  drive = 0;        
        double  strafe = 0;        
        double  turn = 0;        

        initAprilTag();

        leftFrontDrive  = hardwareMap.get(DcMotor.class, "leftFront");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFront");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "leftBack");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBack");

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        double m_botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        
        Translation2d m_frontLeftLocation = new Translation2d(0.75,0.75);
        Translation2d m_frontRightLocation = new Translation2d(0.75,0.75);
        Translation2d m_backLeftLocation = new Translation2d(0.75,0.75);
        Translation2d m_backRightLocation = new Translation2d(0.75,0.75);
    
        MecanumDriveKinematics m_Kinematics = new MecanumDriveKinematics(m_frontLeftLocation,m_frontRightLocation,m_backLeftLocation,m_backRightLocation);
    
        MecanumDriveOdometry m_Odometry = new MecanumDriveOdometry(m_Kinematics, m_botHeading, new Pose(0.0,0.0,new Rotation2d()));
        
        if (USE_WEBCAM)
            setExposure(6, 250);

        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();

        while (opModeIsActive())
        {
            targetFound = false;
            desiredTag  = null;
            MecanumDriveWheelSpeeds wheelSpeeds = new MecanumDriveWheelSpeeds(leftFrontDrive.getCurrentPosition(), rightFrontDrive.getCurrentPosition(),leftBackDrive.getCurrentPosition(), rightBackDrive.getCurrentPosition());
            Rotation2d gyroAngle = Rotation2d.fromDegrees(m_gyro.getAngle());

            m_Pose = m_odometry.update(gyroAngle, wheelSpeeds);

            if (gamepad1.options) {
                imu.resetYaw();
            }
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null) {
                    if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                        targetFound = true;
                        desiredTag = detection;
                        break;
                    } else {
                        telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                    }
                } else {
                    telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
                }
            }

            if (targetFound) {
                telemetry.addData("\n>","HOLD Left-Bumper to Drive to Target\n");
                telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
                telemetry.addData("Range",  "%5.1f inches", desiredTag.ftcPose.range);
                telemetry.addData("Bearing","%3.0f degrees", desiredTag.ftcPose.bearing);
                telemetry.addData("Yaw","%3.0f degrees", desiredTag.ftcPose.yaw);
                telemetry.addData("Pose", "%3.0f", m_Pose.getPose());
            } else {
                telemetry.addData("Pose", "%3.0f", m_Pose.getPose());
                telemetry.addData("\n>","Drive using joysticks to find valid target\n");
            }

            if (gamepad1.left_bumper && targetFound) {

                double  rangeError      = (DESIRED_DISTANCE- desiredTag.ftcPose.range);
                double  headingError    = desiredTag.ftcPose.bearing;
                double  yawError        = desiredTag.ftcPose.yaw;

                drive  = Range.clip(rangeError * m_SpeedKp, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                turn   = Range.clip(-headingError * m_TurnKp, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
                strafe = Range.clip(yawError * m_StrafeKp, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

                telemetry.addData("Auto","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            } else {

                drive  = -gamepad1.left_stick_x  / 1.0;
                strafe = gamepad1.left_stick_y  / 1.0;
                turn   = gamepad1.right_stick_x / 2.0;
                telemetry.addData("Manual","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            }
            telemetry.update();

            moveRobot(drive, strafe, turn);
            sleep(10);
        }
    }
//DC Motor Encoders To MetersPerSecond 
    public double calculateSpeed(int encoderCounts, double timeSeconds) {
        int countsPerRevolution = 1440;
        int wheelDiameterMeters = 0.75;
    
        double revolutions = (double) encoderCounts / countsPerRevolution;
    
        double distanceMeters = revolutions * Math.PI * wheelDiameterMeters;
    
        double speedMetersPerSecond = distanceMeters / timeSeconds;
    
        return speedMetersPerSecond;
    }

//Field-Oriented-Mecanum Drive Codes
    public void moveRobot(double x, double y, double yaw) {

        double rotX = x * Math.cos(-m_botHeading) - y * Math.sin(-m_botHeading);
        double rotY = x * Math.sin(-m_botHeading) + y * Math.cos(-m_botHeading);

        rotX = rotX * 1.1;

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(yaw), 1);
        double leftFrontPower = (rotY + rotX + yaw) / denominator;
        double leftBackPower = (rotY - rotX + yaw) / denominator;
        double rightFrontPower = (rotY - rotX - yaw) / denominator;
        double rightBackPower = (rotY + rotX - yaw) / denominator;

        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }
//Camera Setting For Apriltag
    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder().build();

        aprilTag.setDecimation(2);

        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }
    }
//Exposure Setting
    private void setExposure(int exposureMS, int gain) {
        if (visionPortal == null) {
            return;
        }

        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        if (!isStopRequested())
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }
}
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

//@Disabled
@Autonomous(name="AutoSucker")
public class AutoSucker extends LinearOpMode {

    BNO055IMU imu;
    DcMotor leftMotor;
    DcMotor rightMotor;
    DcMotor armLeft;
    DcMotor armRight;
    DcMotor duckMotor;

    CRServo contServoRight;
    CRServo contServoLeft;

    @Override
    public void runOpMode() throws InterruptedException {

        contServoRight = hardwareMap.crservo.get("contServoRight");
        contServoLeft = hardwareMap.crservo.get("contServoLeft");

        leftMotor  = hardwareMap.get(DcMotor.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");
        armLeft = hardwareMap.get(DcMotor.class, "armLeft");
        armRight = hardwareMap.get(DcMotor.class, "armRight");
        duckMotor = hardwareMap.get(DcMotor.class, "duckMotor");


        //leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftMotor.setPower(0);
        rightMotor.setPower(0);

        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);

        //Opens servo claw, ideal for inserting

        waitForStart();

        leftMotor.setPower(.35);
        rightMotor.setPower(.35);
        sleep(850);

        leftMotor.setPower(0);
        rightMotor.setPower(0);
        sleep(1000);


        armLift(2400);
        sleep(1000);

        intake(3000);
        sleep(3000);

        armLeft.setPower(-0.60);
        armRight.setPower(-0.60);
        sleep(2000);

        contServoRight.setPower(0);
        contServoLeft.setPower(0);
        sleep(800);
        turn(-77);

        leftMotor.setPower(-.5);
        rightMotor.setPower(-.5);
        sleep(3000);

        leftMotor.setPower(0);
        rightMotor.setPower(0);
        sleep(2500);


        //corrective turn
        duckMotor.setPower(-0.5);
        sleep(2500);

        duckMotor.setPower(0);
        sleep(2700);

        //leftMotor.setPower(0.5);
        //rightMotor.setPower(0.5);
        //sleep(500);

        leftMotor.setPower(0);
        rightMotor.setPower(0);
        sleep(500);

        rightMotor.setPower(0.4);
        leftMotor.setPower(0.4);
        sleep(1000);

        rightMotor.setPower(0);
        leftMotor.setPower(0);
        sleep(1000);


        leftMotor.setPower(1);
        rightMotor.setPower(1);
        sleep(2000);

    }

    public void driveStraight(double distance) {
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // rightMotor.setDirection(DcMotor.Direction.REVERSE);
        // leftMotor.setDirection(DcMotor.Direction.REVERSE);

        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double lDistance = leftMotor.getCurrentPosition();
        double rDistance = rightMotor.getCurrentPosition();

        double sign = Math.signum(distance);

        while(opModeIsActive() && ((lDistance + rDistance) / 2) < distance) {
            double diff = lDistance - rDistance;

            double leftPower = 0.40;
            double rightPower = 0.40;

            if (diff > 0) leftPower = 0.4;
            else rightPower = 0.4;

            leftMotor.setPower(leftPower * sign);
            rightMotor.setPower(rightPower * sign);

            lDistance = leftMotor.getCurrentPosition();
            rDistance = rightMotor.getCurrentPosition();

            telemetry.addData("left distance", lDistance);
            telemetry.addData("right distance", rDistance);
            telemetry.update();
        }
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }


    public void turn(double desiredAngle) {
        double angle = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
        double sign = Math.signum(desiredAngle - angle);
        while(opModeIsActive() && Math.signum(desiredAngle - angle) == sign) {
            leftMotor.setPower(0.5 * sign);
            rightMotor.setPower(-0.5 * sign);
            angle = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;

            telemetry.addData("angle", angle);
            telemetry.update();
        }
        leftMotor.setPower(0);
        rightMotor.setPower(0);

    }

    public void armLift(double alarDistance){

        armLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armRight.setDirection(DcMotorSimple.Direction.REVERSE);
        armLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double alDistance = armLeft.getCurrentPosition();
        double arDistance = armRight.getCurrentPosition();

        double sign = Math.signum(alarDistance);

        while(opModeIsActive() && ((alDistance + arDistance) / 2) < alarDistance) {
            double diff = alDistance - arDistance;

            double leftarmPower = 0.50;
            double rightarmPower = 0.50;

            if (diff > 0) leftarmPower = 0.25;
            else rightarmPower = 0.5;

            armLeft.setPower(leftarmPower * sign);
            armRight.setPower(rightarmPower * sign);

            alDistance = armLeft.getCurrentPosition();
            arDistance = armRight.getCurrentPosition();

            telemetry.addData("arm left distance", alDistance);
            telemetry.addData("arm right distance", arDistance);
            telemetry.update();
        }

        armLeft.setPower(0);
        armRight.setPower(0);

    }

    public void armDrop(double dropalarDistance){

        armLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armRight.setDirection(DcMotorSimple.Direction.REVERSE);
        armLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double dropalDistance = armLeft.getCurrentPosition();
        double droparDistance = armRight.getCurrentPosition();

        double sign = Math.signum(droparDistance);

        while(opModeIsActive() && ((dropalDistance + droparDistance) / 2) < dropalarDistance) {
            double diff = dropalDistance - droparDistance;

            double dropleftarmPower = -0.75;
            double droprightarmPower = -0.75;

            if (diff > 0) dropleftarmPower = -0.5;
            else droprightarmPower = -0.5;

            armLeft.setPower(dropleftarmPower * sign);
            armRight.setPower(droprightarmPower * sign);

            dropalDistance = -armLeft.getCurrentPosition();
            droparDistance = -armRight.getCurrentPosition();

            telemetry.addData("arm left distance", dropalDistance);
            telemetry.addData("arm right distance", droparDistance);
            telemetry.update();
        }

        armLeft.setPower(0);
        armRight.setPower(0);

    }

    public void driveStraightBackwards(double backdistance) {
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //rightMotor.setDirection(DcMotor.Direction.REVERSE);
        //rightMotor.setDirection(DcMotor.Direction.REVERSE);

        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double lbackdistance = leftMotor.getCurrentPosition();
        double rbackdistance = rightMotor.getCurrentPosition();

        double sign = Math.signum(backdistance);

        while(opModeIsActive() && ((lbackdistance + rbackdistance) / 2) < backdistance) {
            double diff = lbackdistance - rbackdistance;

            double leftPower = -0.75;
            double rightPower = -0.75;

            if (diff > 0) leftPower = -0.5;
            else rightPower = -0.5;

            leftMotor.setPower(leftPower * sign);
            rightMotor.setPower(rightPower * sign);

            lbackdistance = leftMotor.getCurrentPosition();
            rbackdistance = rightMotor.getCurrentPosition();

            telemetry.addData("left distance", lbackdistance);
            telemetry.addData("right distance", rbackdistance);
            telemetry.update();
        }
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

    public void intake(double clawin){

        contServoLeft.setPower(1);
        contServoRight.setPower(-1);

    }

    public void outtake(double clawout){

        contServoLeft.setPower(-1);
        contServoRight.setPower(1);
    }

    public void duckMotor(double duckSpinner){

        duckMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double duckPower = 1;
        duckMotor.setPower(duckPower);



    }

}

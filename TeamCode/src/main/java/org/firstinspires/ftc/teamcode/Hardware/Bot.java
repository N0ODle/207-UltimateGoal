package org.firstinspires.ftc.teamcode.Hardware;
import com.qualcomm.hardware.bosch.BNO055IMU;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.*;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

//import org.firstinspires.ftc.teamcode.TensorFlowStuff.TensorFlow;

public class Bot {
    public static DcMotorEx BL, BR, FL, FR;
    public static DcMotor shooter, intake;
//    public static double p = 2.5;
//    public static double i = 0.1;
//    public static double d = 0.2;

    //  TOP, BOT, jointl;
//    public CRServo inBOBO;
//    public CRServo LC, RC;
    public Servo wobbleGrabber, wobbleHook, discPlacer;

    //    public DigitalChannel liftLimit, hookLimit;
//    public RevBlinkinLedDriver blinkin;
//    int originTick;
    HardwareMap map;
    Telemetry tele;
//    TensorFlow tensorFlow;

    //    Double powerModifier = 0.02;
    double turnSpeed = 0.25;
    final double proportionalValue = 0.000005;

    //double error = 180 - gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    //Double turnSpeed = 0.5;
    //Integer angle = -45;
    public static BNO055IMU gyro;
    BNO055IMU.Parameters parameters;
    Orientation angles;
    public static PIDCoefficients pid;

    public Bot() {
    }

    public void init(HardwareMap map, Telemetry tele, boolean auton) {
        this.map = map;
        this.tele = tele;
        BR = (DcMotorEx) this.map.get(DcMotor.class, "BR");
        BL = (DcMotorEx) this.map.get(DcMotor.class, "BL");
        FL = (DcMotorEx) this.map.get(DcMotor.class, "FL");
        FR = (DcMotorEx) this.map.get(DcMotor.class, "FR");
        pid = BR.getPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        shooter = this.map.get(DcMotor.class, "shooter");
        intake = this.map.get(DcMotor.class, "intake");

        wobbleHook = this.map.get(Servo.class, "wobbleHook");
        wobbleGrabber = this.map.get(Servo.class, "wobbleGrabber");
        discPlacer = this.map.get(Servo.class, "discPlacer");

        tele.addData(">", "1");
        tele.update();
        BR.setDirection(DcMotorSimple.Direction.FORWARD);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        FR.setDirection( DcMotorSimple.Direction.FORWARD);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);

        this.changeRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        tele.addData(">", "2");
        tele.update();


        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        gyro = this.map.get(BNO055IMU.class, "gyro");
        gyro.initialize(parameters);
        tele.addData(">", "Gyro Calibrating. Do Not Move!");
        tele.update();

//        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        joint.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        hook.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public static void changeRunModeAuton(DcMotor.RunMode runMode) {
        BL.setMode(runMode);
        BR.setMode(runMode);
        FL.setMode(runMode);
        FR.setMode(runMode);
    }

    public static void changeRunMode(DcMotor.RunMode runMode) {
//        pid = new PIDCoefficients(p, i, d);
        BR.setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pid);
        FR.setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pid);
        BL.setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pid);
        FL.setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pid);
//        BL.setMode(runMode);
//        BR.setMode(runMode);
//        FL.setMode(runMode);
//        FR.setMode(runMode);
        //      TOP.setMode(runMode);
        //      BOT.setMode(runMode);
//        RI.setMode(runMode);
//        LI.setMode(runMode);
//        hook.setMode(runMode);
        shooter.setMode(runMode);
        intake.setMode(runMode);
//        joint.setMode(runMode);
        //  intake.setMode(runMode);
    }

    public void drive(double in) {
        BL.setPower(in);
        BR.setPower(in);
        FR.setPower(in);
        FL.setPower(in);
    }

    public void stop(){
        this.drive(0.0);
    }

    public int FLcurPos() {
        return FL.getCurrentPosition();
    }

    public int FRcurPos() {
        return FR.getCurrentPosition();
    }

    public int BLcurPos() {
        return BL.getCurrentPosition();
    }

    public int BRcurPos() {
        return BR.getCurrentPosition();
    }

    public void getPos() {
        FLcurPos();
        FRcurPos();
        BLcurPos();
        BRcurPos();
    }

    public void getDrivePosition() {
        FL.getCurrentPosition();
        FR.getCurrentPosition();
        BL.getCurrentPosition();
        BR.getCurrentPosition();
    }
    public void turnPower(double power) {
        BL.setPower(-power);
        BR.setPower(power);
        FR.setPower(-power);
        FL.setPower(power);
    }

    public void drivePower(double power) {
        FL.setPower(-power);
        FR.setPower(-power);
        BL.setPower(power);
        BR.setPower(power);
    }
    public void strafePower(double power) {
        FL.setPower(power);
        FR.setPower(-power);
        BL.setPower(-power);
        BR.setPower(power);
    }
    public void setPower(double power) {
        strafePower(power);
    }

    public void suck(double power){
//        LI.setPower(power);
//        RI.setPower(power);
    }

    public void shooter(double power){
        shooter.setPower(power);

    }



    public void PID() {
        if (Math.abs(FL.getCurrentPosition() / 1000) <= FL.getTargetPosition()) {setPower(1);}
        else if ((Math.abs(FL.getCurrentPosition() / 800) <= FL.getTargetPosition())) {setPower(0.8);}
        else if ((Math.abs(FL.getCurrentPosition() / 650) <= FL.getTargetPosition())) {setPower(0.65);}
        else if ((Math.abs(FL.getCurrentPosition() / 500) <= FL.getTargetPosition())) {setPower(0.5);}
        else if ((Math.abs(FL.getCurrentPosition() / 450) <= FL.getTargetPosition())) {setPower(0.45);}
        else if ((Math.abs(FL.getCurrentPosition() / 200) <= FL.getTargetPosition())) {setPower(0.2);}
        else if ((Math.abs(FL.getCurrentPosition()) >= FL.getTargetPosition())) {setPower(0);}
    }

    public int autonDrive(MovementEnum movement, int target) {
        int x = -1;
        switch (movement) {
            case FORWARD:
                FL.setTargetPosition(target);
                FR.setTargetPosition(target);
                BL.setTargetPosition(target);
                BR.setTargetPosition(target);
                x = Math.max(BR.getCurrentPosition(), Math.max(BL.getCurrentPosition(), Math.max(FR.getCurrentPosition(), FL.getCurrentPosition())));
                break;

            case BACKWARD:
                FL.setTargetPosition(-target);
                FR.setTargetPosition(-target);
                BL.setTargetPosition(-target);
                BR.setTargetPosition(-target);
                x = Math.max(BR.getCurrentPosition(), Math.max(BL.getCurrentPosition(), Math.max(-FR.getCurrentPosition(), -FL.getCurrentPosition())));
                break;

            case LEFTSTRAFE:
                FL.setTargetPosition(target);
                FR.setTargetPosition(-target);
                BL.setTargetPosition(target);
                BR.setTargetPosition(-target);
                x = Math.max(BR.getCurrentPosition(), Math.max(BL.getCurrentPosition(), Math.max(FR.getCurrentPosition(), FL.getCurrentPosition())));
                break;

            case RIGHTSTRAFE:
//                FL.setTargetPosition(-target);
//                FR.setTargetPosition(target);
//                BL.setTargetPosition(target);
//                BR.setTargetPosition(-target);
                FL.setTargetPosition(-target);
                FR.setTargetPosition(target);
                BL.setTargetPosition(-target);
                BR.setTargetPosition(target);
                x = Math.max(-BR.getCurrentPosition(), Math.max(-BL.getCurrentPosition(), Math.max(-FR.getCurrentPosition(), -FL.getCurrentPosition())));
                break;

            case LEFTTURN:
                FL.setTargetPosition(-target);
                FR.setTargetPosition(target);
                BL.setTargetPosition(-target);
                BR.setTargetPosition(target);
                x = Math.max(BR.getCurrentPosition(), Math.max(-BL.getCurrentPosition(), Math.max(FR.getCurrentPosition(), -FL.getCurrentPosition())));
                break;

            case RIGHTTURN:
                FL.setTargetPosition(target);
                FR.setTargetPosition(-target);
                BL.setTargetPosition(target);
                BR.setTargetPosition(-target);
                x = Math.max(-BR.getCurrentPosition(), Math.max(BL.getCurrentPosition(), Math.max(-FR.getCurrentPosition(), FL.getCurrentPosition())));
                break;

            case STOP:
                FL.setTargetPosition(FL.getCurrentPosition());
                FR.setTargetPosition(FR.getCurrentPosition());
                BL.setTargetPosition(BL.getCurrentPosition());
                BR.setTargetPosition(BR.getCurrentPosition());
                break;
        }
        return x;
    }

    public boolean adjustHeading(int targetHeading) {
        double curHeading = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        double headingError;
        headingError = targetHeading - curHeading;
        double driveScale = headingError;
        this.changeRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if(headingError < -15)
            driveScale = -0.15;
        else if(headingError > 15)
            driveScale = 0.15;
        else {
            driveScale = 0;
            this.drivePower(driveScale);
            return true;
        }
        this.turnPower(driveScale);
        //    this.tele.addData("drive Scale",driveScale);
        //   tele.update();
        //   this.tankDrive(driveScale, -driveScale, 0, 0, false, false);
        // this.changeRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        return false;
    }

    public boolean headingAdjuster(int targetHeading) {
        if(Math.abs(targetHeading - gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle ) > 15) {
            this.adjustHeading(targetHeading);
            return false;
        }
        else if(Math.abs(targetHeading - gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle ) < 15) {
            this.drivePower(0.0);
            tele.update();
            return true;
        }
        return false;
    }


    public double motorSpeed() {
        if (Math.abs(FL.getCurrentPosition()) < Math.abs(FL.getTargetPosition())) {
        } return Math.abs(FL.getTargetPosition()) - Math.abs(FL.getCurrentPosition() * proportionalValue);
    }

}
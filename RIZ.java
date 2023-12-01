package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

public class RIZ {
    DcMotor fL;
    DcMotor bL;
    DcMotor fR;
    DcMotor bR;
    Servo clawL;
    Servo clawR;
    DcMotor elbow;
    CRServo wrist;

    DcMotor VP;
    DcMotor VP2;

    public Telemetry telem;

    boolean clawClose;

    private final ElapsedTime runtime = new ElapsedTime();

    static final double Counts_PER_MOTOR_REV = 537.7;
    static final double DRIVE_GEAR_REDUCTION = 1.0;
    static final double WHEEL_DIAMETER_INCHES = 3.77953;
    static final double COUNTS_PER_INCH = (Counts_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    static final double DRIVE_SPEED = 0.85;
    static final double TURN_SPEED = 0.3;

    public void init(HardwareMap hwmap, Telemetry telem) {
        fL = hardwareMap.dcMotor.get("fL");
        bL = hardwareMap.dcMotor.get("bL");
        fR = hardwareMap.dcMotor.get("fR");
        bR = hardwareMap.dcMotor.get("bR");

        elbow = hardwareMap.dcMotor.get("elbow");

        VP = hardwareMap.dcMotor.get("VP");
        VP2 = hardwareMap.dcMotor.get("VP2");


        clawL = hardwareMap.servo.get("clawL");
        clawR = hardwareMap.servo.get("clawR");
        wrist = hardwareMap.crservo.get("wrist");


        this.telem = telem;


        fL.setDirection(DcMotor.Direction.FORWARD);
        fR.setDirection(DcMotor.Direction.REVERSE);
        bL.setDirection(DcMotor.Direction.FORWARD);
        bR.setDirection(DcMotor.Direction.REVERSE);


        fL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }


    public void encoderDrive(double speed,
                             double leftDrive, double rightDrive, double timeoutS) {
        int newLeftTar;
        int newRightTar;
        int newBackLeftTar;
        int newBackRightTar;

        // Ensure that the opmode is still active

        // Determine new target position, and pass to motor controller
        newLeftTar = fL.getCurrentPosition() + (int) (leftDrive * COUNTS_PER_INCH);
        newRightTar = fR.getCurrentPosition() + (int) (rightDrive * COUNTS_PER_INCH);
        newBackRightTar = bR.getCurrentPosition() + (int) (rightDrive * COUNTS_PER_INCH);
        newBackLeftTar = bL.getCurrentPosition() + (int) (leftDrive * COUNTS_PER_INCH);
        fL.setTargetPosition(newLeftTar);
        fR.setTargetPosition(newRightTar);
        bL.setTargetPosition(newBackLeftTar);
        bR.setTargetPosition(newBackRightTar);

        // Turn On RUN_TO_POSITION
        fL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bR.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        // reset the timeout time and start motion.
        runtime.reset();
        fL.setPower(Math.abs(speed));
        fR.setPower(Math.abs(speed));
        bL.setPower(Math.abs(speed));
        bR.setPower(Math.abs(speed));

        // keep looping while we are still active, and there is time left, and both motors are running.
        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        // However, if you require that BOTH motors have finished their moves before the robot continues
        // onto the next step, use (isBusy() || isBusy()) in the loop test.

        while (runtime.seconds() < timeoutS && (fL.isBusy() && fR.isBusy() && bL.isBusy() && bR.isBusy())) {

            //telemetry.addData("Pat1", "Running to %7d :%7d", newLeftFrontTarget, newRightFrontTarget);
            //telemetry.addData("Path2", "Running to %7d :%7d", frontLeft.getCurrentPosition(), frontRight.getCurrentPosition());
            telem.addData("fL pos", fL.getCurrentPosition());
            telem.addData("fR pos", fR.getCurrentPosition());
            telem.addData("bL pos", bL.getCurrentPosition());
            telem.addData("bR pos", bR.getCurrentPosition());

            // Turn off RUN_TO_POSITION


            fL.setPower(0);
            fR.setPower(0);
            bL.setPower(0);
            bR.setPower(0);


            fL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            fR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            bL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            bR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //sleep(250);   // optional pause after each move.
        }

    }


    public void strafeRight(double speed, double frontLeft,
                       double frontRight, double backLeft,
                       double backRight, double timeoutS) {
        int newfL;
        int newfR;
        int newbL;
        int newbR;

        // Ensure that the opmode is still active

        // Determine new target position, and pass to motor controller
        newfL = fL.getCurrentPosition() + (int) (frontLeft * COUNTS_PER_INCH);
        newfR = fR.getCurrentPosition() + (int) (frontRight * COUNTS_PER_INCH);
        newbR = bR.getCurrentPosition() + (int) (backRight * COUNTS_PER_INCH);
        newbL = bL.getCurrentPosition() + (int) (backLeft * COUNTS_PER_INCH);
        fL.setTargetPosition(newfL);
        fR.setTargetPosition(newfR);
        bL.setTargetPosition(newbL);
        bR.setTargetPosition(newbR);

        // Turn On RUN_TO_POSITION
        fL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bR.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        // reset the timeout time and start motion.
        runtime.reset();
        fL.setPower(Math.abs(speed));
        fR.setPower(Math.abs(-speed));
        bL.setPower(Math.abs(-speed));
        bR.setPower(Math.abs(speed));

        // keep looping while we are still active, and there is time left, and both motors are running.
        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        // However, if you require that BOTH motors have finished their moves before the robot continues
        // onto the next step, use (isBusy() || isBusy()) in the loop test.
        while (runtime.seconds() < timeoutS && (fL.isBusy() && fR.isBusy() && bL.isBusy() && bR.isBusy())) {

            //telemetry.addData("Pat1", "Running to %7d :%7d", newLeftFrontTarget, newRightFrontTarget);
            //telemetry.addData("Path2", "Running to %7d :%7d", frontLeft.getCurrentPosition(), frontRight.getCurrentPosition());
            telem.addData("fL pos", fL.getCurrentPosition());
            telem.addData("fR pos", fR.getCurrentPosition());
            telem.addData("bL pos", bL.getCurrentPosition());
            telem.addData("bR pos", bR.getCurrentPosition());


            // Turn off RUN_TO_POSITION

            fL.setPower(0);
            fR.setPower(0);
            bL.setPower(0);
            bR.setPower(0);

            fL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            fR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            bL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            bR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move.
        }
    }






    public void strafeLeft(double speed, double frontLeft,
                           double frontRight, double backLeft,
                           double backRight, double timeoutS) {
        int newfL;
        int newfR;
        int newbL;
        int newbR;

        // Ensure that the opmode is still active

        // Determine new target position, and pass to motor controller
        newfL = fL.getCurrentPosition() + (int) (frontLeft * COUNTS_PER_INCH);
        newfR = fR.getCurrentPosition() + (int) (frontRight * COUNTS_PER_INCH);
        newbR = bR.getCurrentPosition() + (int) (backRight * COUNTS_PER_INCH);
        newbL = bL.getCurrentPosition() + (int) (backLeft * COUNTS_PER_INCH);
        fL.setTargetPosition(newfL);
        fR.setTargetPosition(newfR);
        bL.setTargetPosition(newbL);
        bR.setTargetPosition(newbR);

        // Turn On RUN_TO_POSITION
        fL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bR.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        // reset the timeout time and start motion.
        runtime.reset();
        fL.setPower(Math.abs(-speed));
        fR.setPower(Math.abs(speed));
        bL.setPower(Math.abs(speed));
        bR.setPower(Math.abs(-speed));

        // keep looping while we are still active, and there is time left, and both motors are running.
        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        // However, if you require that BOTH motors have finished their moves before the robot continues
        // onto the next step, use (isBusy() || isBusy()) in the loop test.
        while (runtime.seconds() < timeoutS && (fL.isBusy() && fR.isBusy() && bL.isBusy() && bR.isBusy())) {

            //telemetry.addData("Pat1", "Running to %7d :%7d", newLeftFrontTarget, newRightFrontTarget);
            //telemetry.addData("Path2", "Running to %7d :%7d", frontLeft.getCurrentPosition(), frontRight.getCurrentPosition());
            telem.addData("fL pos", fL.getCurrentPosition());
            telem.addData("fR pos", fR.getCurrentPosition());
            telem.addData("bL pos", bL.getCurrentPosition());
            telem.addData("bR pos", bR.getCurrentPosition());


            // Turn off RUN_TO_POSITION

            fL.setPower(0);
            fR.setPower(0);
            bL.setPower(0);
            bR.setPower(0);

            fL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            fR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            bL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            bR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move.
        }
    }


    public void dropleftPix() {
        clawL.setPosition(0.4);
    }

    public void droprightPix() {
        clawR.setPosition(0.4);
    }


    public void closed() {
        clawL.setPosition(0);
        clawR.setPosition(0);
        clawClose = true;

    }
}



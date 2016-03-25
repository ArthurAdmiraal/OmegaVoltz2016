package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.text.SimpleDateFormat;
import java.util.Date;

/**
 * TeleOp Mode
 * <p>
 *Enables control of the robot via the gamepad
 */
public class TeleOp_V1 extends OpMode {
    public class Vector2d
    {
        public double x = 0.0;
        public double y = 0.0;

        public Vector2d(double _x, double _y) {
            this.x = _x;
            this.y = _y;
        }

        public void rotate(double n)
        {
            double rx = (this.x * Math.cos(n)) - (this.y * Math.sin(n));
            double ry = (this.x * Math.sin(n)) + (this.y * Math.cos(n));
            this.x    = rx;
            this.y    = ry;
        }
    }

    private final double rotInf = 0.2;
    private final double extraRot = 1.0 - rotInf;

    private String startDate;
    private ElapsedTime runtime = new ElapsedTime();

    DcMotor motorRightFront;
    DcMotor motorLeftFront;
    DcMotor motorRightBack;
    DcMotor motorLeftBack;

    DcMotor motorWinch1;
    DcMotor motorWinch2;

    @Override
    public void init() {
        motorRightFront = hardwareMap.dcMotor.get("MotorRF");
        motorLeftFront  = hardwareMap.dcMotor.get("MotorLF");
        motorRightBack  = hardwareMap.dcMotor.get("MotorRB");
        motorLeftBack   = hardwareMap.dcMotor.get("MotorLB");

        motorWinch1     = hardwareMap.dcMotor.get("Motor1");
        motorWinch2     = hardwareMap.dcMotor.get("Motor2");

        motorWinch2.setDirection(DcMotor.Direction.REVERSE);

        motorWinch1.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorWinch2.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);

        /*motorRightFront.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorLeftFront.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorRightBack.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorLeftBack.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);*/
    }

    /*
       * Code to run when the op mode is first enabled goes here
       * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
       */
    @Override
    public void init_loop() {
        startDate = new SimpleDateFormat("yyyy/MM/dd HH:mm:ss").format(new Date());
        runtime.reset();
        telemetry.addData("Runtime", runtime.toString());
    }

    /*
     * This method will be called repeatedly in a loop
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
     */
    @Override
    public void loop() {
        // get inputs
        Vector2d drive = new Vector2d(Math.pow(gamepad1.left_stick_x, 3), Math.pow(gamepad1.left_stick_y, 3));
        double   turn  = -Math.pow(gamepad1.right_stick_x, 3)*(rotInf + extraRot*Math.pow(gamepad1.right_trigger, 3));
        double   winch = Range.clip(gamepad1.left_trigger*((gamepad1.dpad_up? 1 : 0) - (gamepad1.dpad_down? 1 : 0)), -1.0, 1.0);

        // do calculations
        drive.rotate(-Math.PI / 4);

        // store intermediate results
        double[] buff = { drive.y + turn,
                         -drive.y + turn,
                          drive.x + turn,
                         -drive.x + turn};

        // array normalization
        double f = Math.max(Math.max(Math.abs(buff[0]), Math.abs(buff[1])), Math.max(Math.abs(buff[2]), Math.abs(buff[3])));
        if(f > 1.0) {
            f       = 1/f;
            buff[0] = f*buff[0];
            buff[1] = f*buff[1];
            buff[2] = f*buff[2];
            buff[3] = f*buff[3];
        }

        // write to motors
        motorLeftFront.setPower(Range.clip(buff[0], -1.0, 1.0));
        motorRightFront.setPower(Range.clip(buff[1], -1.0, 1.0));
        motorLeftBack.setPower(Range.clip(buff[2], -1.0, 1.0));
        motorRightBack.setPower(Range.clip(buff[3], -1.0, 1.0));

        motorWinch1.setPower(winch);
        motorWinch2.setPower(winch);
    }
}

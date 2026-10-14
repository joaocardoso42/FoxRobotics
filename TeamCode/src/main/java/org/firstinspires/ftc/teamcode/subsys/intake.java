package org.firstinspires.ftc.teamcode.subsys;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class intake {
    private DcMotor intakeMotor;

    // Construtor: Inicializa o motor no HardwareMap
    public intake(HardwareMap hardwareMap) {
        // "shooter_motor" é o nome configurado na Driver Station
        intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");

        // Se o motor girar ao contrário na hora do tiro, muda pra REVERSE
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);

        // Desliga o motor por segurança no init
        intakeMotor.setPower(0);
    }

    // Liga o shooter com a potência desejada (ex: 0.8 ou 1.0)
    public void enter(double power) {

        intakeMotor.setPower(power);

    }

    public void out(double power) {

        intakeMotor.setPower(-power);

    }


    // Parada rápida
    public void stop() {
        intakeMotor.setPower(0);
    }
}
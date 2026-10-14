package org.firstinspires.ftc.teamcode.subsys;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Shooter {
    private DcMotor shooterMotor;

    // Construtor: Inicializa o motor no HardwareMap
    public Shooter(HardwareMap hardwareMap) {
        // "shooter_motor" é o nome configurado na Driver Station
        shooterMotor = hardwareMap.get(DcMotor.class, "shooter_motor");

        // Se o motor girar ao contrário na hora do tiro, muda pra REVERSE
        shooterMotor.setDirection(DcMotor.Direction.FORWARD);

        // Desliga o motor por segurança no init
        shooterMotor.setPower(0);
    }

    // Liga o shooter com a potência desejada (ex: 0.8 ou 1.0)
    public void shoot(double power) {

        shooterMotor.setPower(power);

    }


    // Parada rápida
    public void stop() {
        shooterMotor.setPower(0);
    }
}
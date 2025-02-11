package br.edu.insper.desagil.pf.omega;

public class Fracionaria {
    private int numerador;
    private int denominador;

    public Fracionaria(int numerador, int denominador) {
        this.numerador = numerador;
        this.denominador = denominador;
    }

    public double lucro(double aposta) {
        if (denominador == 0) {
            throw new IllegalStateException("chances inválidas");
        }

        return (aposta * numerador) / denominador;
    }

    public double total(double aposta) {
        return lucro(aposta) + aposta;
    }

    public String toString() {
        return numerador + "/" + denominador;
    }
}

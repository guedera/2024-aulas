package br.edu.insper.desagil.pf.omega;

public class Decimal {
    private double valor;

    public Decimal(double valor) {
        this.valor = valor;
    }

    public double lucro(double aposta) {
        return total(aposta) - aposta;
    }

    public double total(double aposta) {
        if (valor < 1) {
            throw new IllegalStateException("chances inválidas");
        }

        return aposta * valor;
    }

    public String toString() {
        return "%.2f".formatted(valor);
    }
}

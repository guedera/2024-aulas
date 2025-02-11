package br.edu.insper.desagil.pf.omega;

public class Moneyline {
    private boolean favorito;
    private int valor;

    public Moneyline(boolean favorito, int valor) {
        this.favorito = favorito;
        this.valor = valor;
    }

    public double lucro(double aposta) {
        if (valor == 0) {
            throw new IllegalStateException("chances inválidas");
        }

        if (favorito) {
            return (100 * aposta) / valor;
        } else {
            return (aposta / 100) * valor;
        }
    }

    public double total(double aposta) {
        return lucro(aposta) + aposta;
    }

    public String toString() {
        String prefixo;

        if (favorito) {
            prefixo = "-";
        } else {
            prefixo = "+";
        }

        return prefixo + valor;
    }
}

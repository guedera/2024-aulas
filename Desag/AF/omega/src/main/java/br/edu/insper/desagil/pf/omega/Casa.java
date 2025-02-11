package br.edu.insper.desagil.pf.omega;

import java.util.List;

public class Casa {
    private List<Moneyline> moneylines;
    private List<Fracionaria> fracionarias;
    private List<Decimal> decimais;

    public Casa(List<Moneyline> moneylines, List<Fracionaria> fracionarias, List<Decimal> decimais) {
        this.moneylines = moneylines;
        this.fracionarias = fracionarias;
        this.decimais = decimais;
    }

    public double calculaLucro(Usuario usuario, double aposta, int indice) {
        if (usuario.getPais().equals("US")) {
            return moneylines.get(indice).lucro(aposta);
        } else if (usuario.getPais().equals("UK")) {
            return fracionarias.get(indice).lucro(aposta);
        } else if (usuario.getPais().equals("CA")) {
            return decimais.get(indice).lucro(aposta);
        } else {
            throw new IllegalArgumentException("país inválido");
        }
    }

    public void adicionaTotal(Usuario usuario, double aposta, int indice) {
        double total;

        if (usuario.getPais().equals("US")) {
            total = moneylines.get(indice).total(aposta);
        } else if (usuario.getPais().equals("UK")) {
            total = fracionarias.get(indice).total(aposta);
        } else if (usuario.getPais().equals("CA")) {
            total = decimais.get(indice).total(aposta);
        } else {
            throw new IllegalArgumentException("país inválido");
        }

        usuario.setSaldo(usuario.getSaldo() + total);
    }

    public boolean podeApostar(Usuario usuario, double aposta) {
        return usuario.getSaldo() - aposta >= 0;
    }

    public void subtraiAposta(Usuario usuario, double aposta) {
        usuario.setSaldo(usuario.getSaldo() - aposta);
    }
}

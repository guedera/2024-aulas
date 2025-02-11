package br.edu.insper.desagil.pf.omega;

public class Usuario {
    private String login;
    private String pais;
    private String nome;
    private double saldo;

    public Usuario(String login, String pais, String nome) {
        this.login = login;
        this.pais = pais;
        this.nome = nome;
        this.saldo = 0;
    }

    public String getLogin() {
        return login;
    }

    public String getPais() {
        return pais;
    }

    public String getNome() {
        return nome;
    }

    public void setNome(String nome) {
        this.nome = nome;
    }

    public double getSaldo() {
        return saldo;
    }

    public void setSaldo(double saldo) {
        this.saldo = saldo;
    }
}

package br.edu.insper.desagil.pf.igordao;

public class Maquina {
    private String nome;
    private String urlFoto;
    private int pesoMinimo;
    private int pesoMaximo;

    //constroi
    public Maquina (String nome,String url) {
        this.nome = nome;
        this.urlFoto = url;
        this.pesoMinimo = 10;
        this.pesoMaximo = 100;
    }

    //getters
    public String getNome() {
        return nome;
    }

    public String getUrlFoto() {
        return urlFoto;
    }

    public int getPesoMinimo() {
        return pesoMinimo;
    }

    public int getPesoMaximo() {
        return pesoMaximo;
    }

    //setters
    public void setNome(String nome) {
        this.nome = nome;
    }

    public void setUrlFoto(String urlFoto) {
        this.urlFoto = urlFoto;
    }

    //metodos
    public void atualizaPesos(int pmin,int pmax) {
        //parte 1
        if (pmin < 1) {
            pesoMinimo = 1;
        }
        else {
            pesoMinimo = pmin;
        }

        //parte 2
        if (pmax <= pesoMinimo) {
            pesoMaximo = pesoMinimo + 1;
        }
        else {
            pesoMaximo = pmax;
        }
    }
}

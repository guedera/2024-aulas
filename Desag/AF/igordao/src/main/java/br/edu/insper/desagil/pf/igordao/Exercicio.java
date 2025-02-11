package br.edu.insper.desagil.pf.igordao;

public class Exercicio {
    private String identificador;
    private Maquina maquina;
    private String nome;
    private int repeticoes;
    private int descanso;

    //constroi
    public Exercicio (String identificador,Maquina maquina, String nome, int repeticoes, int descanso) {
        this.identificador = identificador;
        this.maquina = maquina;
        this.nome = nome;
        this.repeticoes = repeticoes;
        this.descanso = descanso;
    }

    //getters
    public String getIdentificador() {
        return identificador;
    }

    public Maquina getMaquina() {
        return maquina;
    }

    public String getNome() {
        return nome;
    }

    public int getRepeticoes() {
        return repeticoes;
    }

    public int getDescanso() {
        return descanso;
    }

    //setters

    public void setNome(String nome) {
        this.nome = nome;
    }

    //metodos
    public void decrementaRepeticoes() {
        repeticoes = repeticoes -1;
    }

    public void incrementaRepeticoes() {
        repeticoes = repeticoes +1;
    }

    public void decrementaDescanso() {
        descanso = descanso -1;
    }

    public void incrementaDescanso() {
        descanso = descanso +1;
    }
}

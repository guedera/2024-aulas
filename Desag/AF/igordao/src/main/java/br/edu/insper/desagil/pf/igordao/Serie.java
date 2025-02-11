package br.edu.insper.desagil.pf.igordao;

public class Serie {
    private Exercicio exercicio;
    private int numeroVezes;
    private int peso;

    //constroi
    public Serie(Exercicio exercicio, int numeroVezes, int peso) {
        this.exercicio = exercicio;

        if (numeroVezes < 1) {
            throw new IllegalArgumentException("número de vezes deve ser positivo");
        }
        else {
            this.numeroVezes = numeroVezes;
        }

        if (peso < exercicio.getMaquina().getPesoMinimo()) {
            throw new IllegalArgumentException("peso não pode ser menor que o mínimo");

        } else if (peso > exercicio.getMaquina().getPesoMaximo()) {
            throw new IllegalArgumentException("peso não pode ser maior que o máximo");
        }

        else {
            this.peso = peso;
        }
    }

    //getters
    public Exercicio getExercicio() {
        return exercicio;
    }

    public int getNumeroVezes() {
        return numeroVezes;
    }

    public int getPeso() {
        return peso;
    }

    //metodos
    public String toString() {
        return exercicio.getNome() + " " + numeroVezes + "x" + peso;
    }

}

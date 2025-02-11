package br.edu.insper.desagil.pf.igordao;

public class ExercicioIntenso extends Exercicio {
    private int repeticoes;
    private int descanso;

    public ExercicioIntenso(String identificador, Maquina maquina, String nome) {
        super(identificador, maquina, nome, 10, 30);
    }

    @Override
    public void decrementaRepeticoes() {
        repeticoes = repeticoes -10;
    }

    @Override
    public void incrementaRepeticoes() {
        repeticoes = repeticoes +10;
    }

    @Override
    public void decrementaDescanso() {
        descanso = descanso -30;
    }

    @Override
    public void incrementaDescanso() {
        descanso = descanso +30;
    }
}

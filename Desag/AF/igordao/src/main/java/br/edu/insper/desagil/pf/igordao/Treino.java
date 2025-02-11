package br.edu.insper.desagil.pf.igordao;

import java.time.LocalDate;
import java.time.LocalTime;
import java.util.ArrayList;
import java.util.List;

public class Treino {
    private LocalDate dia;
    private LocalTime hora;
    private List<Serie> series;

    //constroi
    public Treino(LocalDate dia, LocalTime  hora) {
        this.dia = dia;
        this.hora = hora;
        this.series = new ArrayList<>();
    }

    //getters
    public LocalDate getDia() {
        return dia;
    }

    public LocalTime getHora() {
        return hora;
    }

    public List<Serie> getSeries() {
        return series;
    }

    //metodos
    public void adiciona(Serie serienova) {
        boolean jaTem = false;

        for (Serie serie : series) {

            if (serienova.getExercicio().getIdentificador().equals(serie.getExercicio().getIdentificador())) {

                if (serienova.getNumeroVezes() == serie.getNumeroVezes()) {

                    if (serienova.getPeso() == serie.getPeso()) {

                        jaTem = true;

                    }

                }

            }

        }

        if (!jaTem) {
            series.add(serienova);
        }
    }
}

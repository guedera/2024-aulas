package br.edu.insper.desagil.pf.igordao;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import java.time.LocalDate;
import java.time.LocalDate;

import java.time.LocalTime;
import java.util.ArrayList;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

public class TreinoTest {
    private Treino treino;
    private Serie serie;
    private List<Serie> series1;

    @BeforeEach
    public void setUp() {
        series1 = new ArrayList<>();

        LocalDate dia = LocalDate.now();
        LocalTime hora = LocalTime.now();

        treino = new Treino(dia, hora);
    }

    @Test
    public void nenhumaSerie() {
        List<Serie> series1 = new ArrayList<>();
        assertEquals(series1,treino.getSeries());
    }

    @Test
    public void umaSerie() {
        Exercicio exercicio = mock(Exercicio.class);
        Maquina maquina = mock(Maquina.class);
        when(exercicio.getNome()).thenReturn("leg press inclinado");
        when(maquina.getPesoMinimo()).thenReturn(10);
        when(maquina.getPesoMaximo()).thenReturn(100);
        when(exercicio.getMaquina()).thenReturn(maquina);
        when(maquina.getNome()).thenReturn("leg press 45");


        Serie serie = new Serie(exercicio,15,30);

        treino.adiciona(serie);
        series1.add(serie);

        assertEquals(series1,treino.getSeries());
    }
}

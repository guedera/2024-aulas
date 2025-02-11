package br.edu.insper.desagil.pf.igordao;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.function.Executable;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

public class SerieTest {
    private Exercicio exercicio;
    private Maquina maquina;

    @BeforeEach
    public void setUp() {
        exercicio = mock(Exercicio.class);
        maquina = mock(Maquina.class);
    }
    @Test
    public void numeroVezesErrado() {

        assertThrows(IllegalArgumentException.class, () -> {
            criaSerie(exercicio,0,50);
        },"número de vezes deve ser positivo");
    }

    @Test
    public void pesoMuitoBaixo() {
        when(exercicio.getMaquina()).thenReturn(maquina);
        when(maquina.getPesoMinimo()).thenReturn(10);

        assertThrows(IllegalArgumentException.class, () -> {
            criaSerie(exercicio,10,0);
        },"peso não pode ser menor que o mínimo");
    }

    @Test
    public void pesoMuitoAlto() {
        when(exercicio.getMaquina()).thenReturn(maquina);
        when(maquina.getPesoMinimo()).thenReturn(10);
        when(maquina.getPesoMaximo()).thenReturn(100);

        assertThrows(IllegalArgumentException.class, () -> {
            criaSerie(exercicio,10,150);
        },"peso não pode ser maior que o máximo");
    }

    @Test
    public void constroi() {
        when(exercicio.getMaquina()).thenReturn(maquina);
        when(exercicio.getNome()).thenReturn("leg press inclinado");
        when(maquina.getPesoMinimo()).thenReturn(10);
        when(maquina.getPesoMaximo()).thenReturn(100);
        Serie serie = new Serie(exercicio,10,50);

        assertEquals("leg press inclinado 10x50", serie.toString());
    }

    private void criaSerie(Exercicio exercicio, int numeroX, int peso) {
        Serie serie = new Serie(exercicio,numeroX,peso);
    }
}

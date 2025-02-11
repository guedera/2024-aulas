package br.edu.insper.desagil.pf.igordao;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;

public class MaquinaTest {
    private Maquina maquina;

    @BeforeEach
    public void setUp() {
        maquina = new Maquina("Escada", "xyz");
    }

    @Test
    public void constroi() {
        String nome = "Escada";
        String url = "xyz";

        assertEquals(nome, maquina.getNome());
        assertEquals(url, maquina.getUrlFoto());
        assertEquals(100,maquina.getPesoMaximo());
        assertEquals(10,maquina.getPesoMinimo());
    }

    @Test
    public void muda() {
        String nome = "Barra";
        String url = "whz";

        maquina.setNome(nome);
        maquina.setUrlFoto(url);

        assertEquals(nome, maquina.getNome());
        assertEquals(url, maquina.getUrlFoto());
    }

    @Test
    public void pesoMinimoErrado() {
        maquina.atualizaPesos(0,200);

        assertEquals(1,maquina.getPesoMinimo());
        assertEquals(200,maquina.getPesoMaximo());
    }

    @Test
    public void pesoMaximoErrado() {
        maquina.atualizaPesos(20,10);

        assertEquals(20,maquina.getPesoMinimo());
        assertEquals(21,maquina.getPesoMaximo());
    }

    @Test
    public void ambosPesosErrados() {
        maquina.atualizaPesos(0,-10);

        assertEquals(1,maquina.getPesoMinimo());
        assertEquals(2,maquina.getPesoMaximo());
    }

    @Test
    public void ambosPesosCertos() {
        maquina.atualizaPesos(20,200);

        assertEquals(20,maquina.getPesoMinimo());
        assertEquals(200,maquina.getPesoMaximo());
    }
}

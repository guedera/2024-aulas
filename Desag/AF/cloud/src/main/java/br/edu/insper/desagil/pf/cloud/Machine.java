package br.edu.insper.desagil.pf.cloud;

public class Machine extends Node {
    private String name;

    public Machine(String name) {
        this.name = name;
    }

    public String getName() {
        return name;
    }

    public void setName(String name) {
        this.name = name;
    }
}

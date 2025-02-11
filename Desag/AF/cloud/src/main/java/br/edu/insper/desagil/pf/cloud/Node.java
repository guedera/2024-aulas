package br.edu.insper.desagil.pf.cloud;

import java.util.ArrayList;
import java.util.List;

public class Node {
    private List<Connection> connections;

    public Node() {
        this.connections = new ArrayList<>();
    }

    public List<Connection> getConnections() {
        return connections;
    }

    public void addConnection(Router router, String address) {
        Connection connection = createConnection(router, address);
        connections.add(connection);
    }

    private Connection createConnection(Router router, String address) {
        return new Connection(router, address);
    }
}

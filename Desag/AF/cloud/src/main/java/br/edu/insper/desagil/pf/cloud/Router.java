package br.edu.insper.desagil.pf.cloud;

import java.util.Map;

public class Router {
    private Map<String, Node> nodes;
    private Map<String, String> dns;

    public Router(Map<String, Node> nodes, Map<String, String> dns) {
        this.nodes = nodes;
        this.dns = dns;
    }

    public Map<String, Node> getNodes() {
        return nodes;
    }

    public Map<String, String> getDns() {
        return dns;
    }

    public Node nodeFromAddress(String address) {
        return nodes.get(address);
    }

    public Node nodeFromAlias(String alias) {
        String address = dns.get(alias);
        return nodeFromAddress(address);
    }
}

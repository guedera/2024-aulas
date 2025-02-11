package br.edu.insper.desagil.pf.cloud;

public class SecureConnection extends Connection {
    public SecureConnection(Router router, String targetAddress) {
        super(router, targetAddress);
    }

    @Override
    public Node target() {
        System.out.println("INFO: target");
        return super.target();
    }
}

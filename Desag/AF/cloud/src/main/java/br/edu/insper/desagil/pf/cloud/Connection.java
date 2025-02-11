package br.edu.insper.desagil.pf.cloud;

public class Connection {
    private Router router;
    private String targetAddress;

    public Connection(Router router, String targetAddress) {
        this.router = router;
        this.targetAddress = targetAddress;
    }

    public Node target() {
        return router.nodeFromAddress(targetAddress);
    }
}

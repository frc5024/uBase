package frc.lib5k.networking.servers;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.SocketException;
import java.util.HashMap;

import frc.lib5k.networking.Port;
import frc.lib5k.networking.PortManager;
import frc.lib5k.networking.Port.Protocol;
import frc.lib5k.networking.exceptions.PortException;

public class UDPServer extends Thread {

    private DatagramSocket socket;
    private boolean running;
    private byte[] buffer;
    private String name;
    private Port port;

    private HashMap<String, Integer> clients = new HashMap();

    public UDPServer(String name, int port_number, int buffer_size) throws PortException, SocketException {
        // Try to allocate port
        this.port = new Port(Protocol.UDP, port_number);
        PortManager.getInstance().allocatePort(port);

        // Set locals
        this.name = name;
        this.buffer = new byte[buffer_size];
        this.socket = new DatagramSocket(port.getNumber());

    }

    @Override
    public void run() {
        running = true;

        // Run-loop
        while (running) {
            // Read the next packet
            DatagramPacket packet = new DatagramPacket(buffer, buffer.length);

            // Recv data
            try{
                socket.receive(packet);
            } catch(IOException e){
                // TODO: handle this.. maybe a fault counter?
            }

            // Read the client info, and add to clients map
            
        }

        // Close and free the socket
        socket.close();
    }

}
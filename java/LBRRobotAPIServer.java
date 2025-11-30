package com.kuka.fri.lbr.example;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;

import java.io.BufferedReader;
import java.io.DataOutputStream;
import java.io.IOException;
import java.io.InputStreamReader;
import java.net.ServerSocket;
import java.net.Socket;
import java.nio.charset.StandardCharsets;
import java.util.concurrent.locks.ReentrantLock;

import javax.inject.Inject;

import com.kuka.device.common.JointPosition;
import com.kuka.geometry.ObjectFrame;
import com.kuka.geometry.World;
import com.kuka.math.geometry.Rotation;
import com.kuka.math.geometry.Transformation;
import com.kuka.math.geometry.Vector3D;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.sensitivity.LBR;

/**
 * ͨ�û�����API����������KUKA�����������У�
 * ͨ��Socket��������Python�ͻ��˵���������ƻ�е��
 * 
 * ֧�ֵ�API���
 * 1. �ѿ��������˶���
 *    - "MOVE_CARTESIAN x y z alpha beta gamma velocity"
 *    - x, y, z: λ�ã���λ�����ף�
 *    - alpha, beta, gamma: ŷ���ǣ���λ���ȣ�ZYX˳��
 *    - velocity: ����ٶȣ�0.0-1.0����ѡ��Ĭ��0.25��
 * 
 * 2. �ؽڽǶ��˶���
 *    - "MOVE_JOINT j1 j2 j3 j4 j5 j6 j7 velocity"
 *    - j1-j7: �ؽڽǶȣ���λ���ȣ�
 *    - velocity: ����ٶȣ�0.0-1.0����ѡ��Ĭ��0.25��
 * 
 * 3. ��ȡ��ǰĩ��λ�ˣ�
 *    - "GET_POSE"
 *    - ���أ�x y z alpha beta gamma����λ��mm, �ȣ�
 * 
 * 4. �������
 *    - "PING" - ��������
 *    - "EXIT" / "QUIT" - �˳�
 * 
 * ��Ҫ��ʾ��
 * 1. KUKA iiwaֻ��ʹ�ö˿ڷ�Χ 30000-30010
 * 2. ȷ������ǽ������Щ�˿ڵ�ͨ��
 */
public class LBRRobotAPIServer extends RoboticsAPIApplication
{
    @Inject
    private LBR _lbr;
    
    @Inject
    private World _world;

    private static final String BIND_ADDRESS = "0.0.0.0"; // ������������ӿ�
    // ��Ҫ��KUKA iiwaֻ��ʹ�ö˿ڷ�Χ 30000-30010
    private static final int PORT = 30009; // ������ 30000-30010 ��Χ��
    
    private static final double DEFAULT_VELOCITY_REL = 0.25; // Ĭ������ٶȣ�0.0-1.0��
    
    private volatile boolean _running = true;
    private ServerSocket _serverSocket;
    private Socket _clientSocket;
    
    // ����ȷ���˶������ִ�е�����GET_POSE����Ҫ������
    private final ReentrantLock _motionLock = new ReentrantLock();
    
    // ��ǰ�˶�����������ȡ�����ڽ��е��˶�
    private volatile com.kuka.motion.IMotionContainer _currentMotionContainer = null;

    @Override
    public void dispose()
    {
        _running = false;
        closeQuietly(_clientSocket);
        closeQuietly(_serverSocket);
    }

    @Override
    public void run()
    {
        getLogger().info("========================================");
        getLogger().info("Robot API Server Starting...");
        getLogger().info("Binding to: " + BIND_ADDRESS + ":" + PORT);
        getLogger().info("Supported APIs:");
        getLogger().info("  - MOVE_CARTESIAN x y z alpha beta gamma [velocity]");
        getLogger().info("  - MOVE_JOINT j1 j2 j3 j4 j5 j6 j7 [velocity]");
        getLogger().info("  - GET_POSE");
        getLogger().info("  - GET_JOINT");
        getLogger().info("  - PING, EXIT");
        getLogger().info("========================================");

        // ����������Socket���������нӿ�
        try
        {
            _serverSocket = new ServerSocket(PORT);
            getLogger().info("Server socket created successfully.");
        }
        catch (IOException ex)
        {
            getLogger().error("Failed to create server socket: " + ex.getMessage());
            ex.printStackTrace();
            return;
        }

        // ���ѭ���������ȴ��ͻ������ӣ���פ����
        while (_running)
        {
            Socket currentClientSocket = null;
            BufferedReader inFromClient = null;
            DataOutputStream outToClient = null;
            
            try
            {
                getLogger().info("Waiting for client connection...");
                
                // �ȴ��ͻ�������
                currentClientSocket = _serverSocket.accept();
                _clientSocket = currentClientSocket; // ���浱ǰ�ͻ�������
                
                getLogger().info("Client connected from: " + currentClientSocket.getRemoteSocketAddress());
                getLogger().info("Client IP: " + currentClientSocket.getInetAddress().getHostAddress());

                // �������������
                inFromClient = new BufferedReader(
                        new InputStreamReader(currentClientSocket.getInputStream(), StandardCharsets.UTF_8));
                outToClient = new DataOutputStream(currentClientSocket.getOutputStream());

                // ���ͻ�ӭ��Ϣ
                String welcomeMsg = "Robot API Server Ready!\n" +
                        "Supported APIs:\n" +
                        "  MOVE_CARTESIAN x y z alpha beta gamma [velocity]\n" +
                        "  MOVE_JOINT j1 j2 j3 j4 j5 j6 j7 [velocity]\n" +
                        "  GET_POSE\n" +
                        "  GET_JOINT\n" +
                        "  PING, EXIT\n";
                outToClient.writeBytes(welcomeMsg);
                getLogger().info("Welcome message sent to client.");

                String clientSentence;
                int commandCount = 0;

                // �ڲ�ѭ����������ǰ�ͻ��˵�����
                while (_running)
                {
                    try
                    {
                        clientSentence = inFromClient.readLine();
                        
                        if (clientSentence == null)
                        {
                            getLogger().info("Client disconnected. Waiting for next client...");
                            break; // �˳��ڲ�ѭ�����ȴ���һ���ͻ���
                        }
                        
                        clientSentence = clientSentence.trim();
                        
                        if (clientSentence.isEmpty())
                        {
                            continue;
                        }
                        
                        // ������������
                        if ("exit".equalsIgnoreCase(clientSentence) || "quit".equalsIgnoreCase(clientSentence))
                        {
                            getLogger().info("Received exit command. Closing connection with this client.");
                            outToClient.writeBytes("OK: Connection closed. Server continues running.\n");
                            break; // �˳��ڲ�ѭ�����ȴ���һ���ͻ���
                        }
                        else if ("ping".equalsIgnoreCase(clientSentence))
                        {
                            outToClient.writeBytes("pong\n");
                            getLogger().info("Sent pong response.");
                        }
                        else if (clientSentence.toUpperCase().startsWith("MOVE_CARTESIAN"))
                        {
                            // �����ѿ��������˶������Ҫ����ִ�У�
                            commandCount++;
                            _motionLock.lock();
                            try
                            {
                                boolean success = processCartesianCommand(clientSentence, outToClient, commandCount);
                                if (!success)
                                {
                                    getLogger().warn("Failed to process Cartesian command: " + clientSentence);
                                }
                            }
                            finally
                            {
                                _motionLock.unlock();
                            }
                        }
                        else if (clientSentence.toUpperCase().startsWith("MOVE_JOINT"))
                        {
                            // �����ؽڽǶ��˶������Ҫ����ִ�У�
                            commandCount++;
                            _motionLock.lock();
                            try
                            {
                                boolean success = processJointCommand(clientSentence, outToClient, commandCount);
                                if (!success)
                                {
                                    getLogger().warn("Failed to process joint command: " + clientSentence);
                                }
                            }
                            finally
                            {
                                _motionLock.unlock();
                            }
                        }
                        else if (clientSentence.toUpperCase().startsWith("GET_POSE"))
                        {
                            // ������ȡλ����������첽ִ�У�����Ҫ����
                            boolean success = processGetPoseCommand(outToClient);
                            if (!success)
                            {
                                getLogger().warn("Failed to get pose");
                            }
                        }
                        else if (clientSentence.toUpperCase().startsWith("GET_JOINT"))
                        {
                            // ������ȡ�ؽڽǶ���������첽ִ�У�����Ҫ����
                            boolean success = processGetJointCommand(outToClient);
                            if (!success)
                            {
                                getLogger().warn("Failed to get joint position");
                            }
                        }
                        else
                        {
                            // δ֪����
                            String errorMsg = "ERROR: Unknown command. Supported: MOVE_CARTESIAN, MOVE_JOINT, GET_POSE, GET_JOINT, PING, EXIT\n";
                            outToClient.writeBytes(errorMsg);
                            getLogger().warn("Unknown command received: " + clientSentence);
                        }
                    }
                    catch (IOException e)
                    {
                        getLogger().error("Error reading from client: " + e.getMessage());
                        getLogger().info("Client connection lost. Waiting for next client...");
                        break; // �˳��ڲ�ѭ�����ȴ���һ���ͻ���
                    }
                }

                getLogger().info("Total commands processed from this client: " + commandCount);
            }
            catch (IOException ex)
            {
                getLogger().error("Error accepting client connection: " + ex.getMessage());
                // �����ȴ���һ���ͻ���
            }
            finally
            {
                // �رյ�ǰ�ͻ��˵����Ӻ���
                if (inFromClient != null)
                {
                    try
                    {
                        inFromClient.close();
                    }
                    catch (IOException e)
                    {
                        getLogger().error("Error closing input stream: " + e.getMessage());
                    }
                }
                if (outToClient != null)
                {
                    try
                    {
                        outToClient.close();
                    }
                    catch (IOException e)
                    {
                        getLogger().error("Error closing output stream: " + e.getMessage());
                    }
                }
                if (currentClientSocket != null)
                {
                    try
                    {
                        currentClientSocket.close();
                        _clientSocket = null; // �����ǰ�ͻ�������
                    }
                    catch (IOException e)
                    {
                        getLogger().error("Error closing client socket: " + e.getMessage());
                    }
                }
                
                getLogger().info("Client connection closed. Server continues running...");
            }
        }
        
        // ֻ����_runningΪfalseʱ�Źرշ�����
        dispose();
        getLogger().info("========================================");
        getLogger().info("Robot API Server Stopped.");
        getLogger().info("========================================");
    }

    /**
     * �����ѿ��������˶�����
     * ��ʽ��MOVE_CARTESIAN x y z alpha beta gamma [velocity]
     */
    private boolean processCartesianCommand(String command, DataOutputStream outToClient, int commandNumber)
    {
        try
        {
            // ��������
            String[] parts = command.split("\\s+");
            
            if (parts.length < 7 || parts.length > 8)
            {
                String errorMsg = "ERROR: Invalid format. Need 6-7 values: MOVE_CARTESIAN x y z alpha beta gamma [velocity]\n";
                outToClient.writeBytes(errorMsg);
                getLogger().error("Invalid Cartesian command format: " + command);
                return false;
            }
            
            // ��������
            double x = Double.parseDouble(parts[1]);      // mm
            double y = Double.parseDouble(parts[2]);      // mm
            double z = Double.parseDouble(parts[3]);      // mm
            double alpha = Double.parseDouble(parts[4]);  // deg (ZYX Euler)
            double beta = Double.parseDouble(parts[5]);   // deg
            double gamma = Double.parseDouble(parts[6]);  // deg
            double velocity = (parts.length == 8) ? Double.parseDouble(parts[7]) : DEFAULT_VELOCITY_REL;
            
            // �����ٶȷ�Χ
            velocity = Math.max(0.0, Math.min(1.0, velocity));
            
            getLogger().info(String.format("Command #%d: Moving to [%.2f, %.2f, %.2f] mm, [%.2f, %.2f, %.2f] deg, velocity=%.2f",
                    commandNumber, x, y, z, alpha, beta, gamma, velocity));
            
            // ����Ŀ��任
            Transformation transform = Transformation.ofDeg(x, y, z, alpha, beta, gamma);
            
            // ����Ŀ��Frame
            String frameName = "CartesianTarget_" + commandNumber;
            ObjectFrame targetFrame = _world.createFrame(frameName, transform);
            
            try
            {
                // ��������"�����ѽ���"��Ӧ����Python�˿��Լ���������һ������
                outToClient.writeBytes("OK: Command received\n");
                outToClient.flush();
                
                // ȡ����ǰ���ڽ��е��˶���������ڣ�
                // �����������������ִ�У�ʵ��ʵʱ����
                _motionLock.lock();
                try {
                    if (_currentMotionContainer != null) {
                        try {
                            getLogger().info("Cancelling previous motion to execute new command...");
                            _currentMotionContainer.cancel();
                            _currentMotionContainer = null;
                            // �ȴ�һС��ʱ�䣬ȷ��ȡ���������
                            Thread.sleep(10);
                        } catch (Exception e) {
                            getLogger().warn("Error cancelling previous motion: " + e.getMessage());
                        }
                    }
                } finally {
                    _motionLock.unlock();
                }
                
                // ʹ��moveAsync�첽ִ���˶���������MotionContainer�Ա����ȡ��
                try {
                    getLogger().info("Executing new motion command...");
                    com.kuka.motion.IMotionContainer motionContainer = 
                        _lbr.getFlange().moveAsync(ptp(targetFrame).setJointVelocityRel(velocity));
                    
                    // ���浱ǰ�˶��������Ա����ȡ��
                    _motionLock.lock();
                    try {
                        _currentMotionContainer = motionContainer;
                    } finally {
                        _motionLock.unlock();
                    }
                    
                    // �ں�̨�߳��еȴ��˶���ɣ���ѡ������������
                    final ObjectFrame finalTargetFrame = targetFrame;
                    Thread cleanupThread = new Thread(() -> {
                        try {
                            // �ȴ��˶���ɣ���ȡ����
                            motionContainer.await();
                            
                            // �˶���ɺ�����MotionContainer����
                            _motionLock.lock();
                            try {
                                if (_currentMotionContainer == motionContainer) {
                                    _currentMotionContainer = null;
                                }
                            } finally {
                                _motionLock.unlock();
                            }
                            
                            // ����������Frame
                            try {
                                _world.removeFrame(finalTargetFrame);
                            } catch (Exception e) {
                                getLogger().warn("Failed to remove frame: " + e.getMessage());
                            }
                        } catch (Exception e) {
                            // �˶���ȡ���������쳣����������
                            _motionLock.lock();
                            try {
                                if (_currentMotionContainer == motionContainer) {
                                    _currentMotionContainer = null;
                                }
                            } finally {
                                _motionLock.unlock();
                            }
                            
                            // ����Frame
                            try {
                                _world.removeFrame(finalTargetFrame);
                            } catch (Exception ex) {
                                getLogger().warn("Failed to remove frame: " + ex.getMessage());
                            }
                        }
                    });
                    cleanupThread.setDaemon(true);
                    cleanupThread.start();
                    
                    // ���Frame�����첽�߳��д���
                    targetFrame = null;
                } catch (Exception e) {
                    getLogger().error("Error starting motion: " + e.getMessage());
                    e.printStackTrace();
                    // �������ʧ�ܣ�����Frame
                    if (targetFrame != null) {
                        try {
                            _world.removeFrame(targetFrame);
                        } catch (Exception ex) {
                            getLogger().warn("Failed to remove frame: " + ex.getMessage());
                        }
                    }
                    throw e;
                }
                
                return true;
            }
            catch (Exception e)
            {
                String errorMsg = "ERROR: Motion execution failed: " + e.getMessage() + "\n";
                outToClient.writeBytes(errorMsg);
                getLogger().error("Motion execution error: " + e.getMessage());
                e.printStackTrace();
                return false;
            }
            finally
            {
                // ����������Frame�������û�����첽�߳���������
                if (targetFrame != null)
                {
                    try
                    {
                        _world.removeFrame(targetFrame);
                    }
                    catch (Exception e)
                    {
                        getLogger().warn("Failed to remove frame: " + e.getMessage());
                    }
                }
            }
        }
        catch (NumberFormatException e)
        {
            String errorMsg = "ERROR: Invalid number format: " + e.getMessage() + "\n";
            try
            {
                outToClient.writeBytes(errorMsg);
            }
            catch (IOException ioEx)
            {
                getLogger().error("Error sending error message: " + ioEx.getMessage());
            }
            getLogger().error("Number format error: " + e.getMessage());
            return false;
        }
        catch (Exception e)
        {
            String errorMsg = "ERROR: " + e.getMessage() + "\n";
            try
            {
                outToClient.writeBytes(errorMsg);
            }
            catch (IOException ioEx)
            {
                getLogger().error("Error sending error message: " + ioEx.getMessage());
            }
            getLogger().error("Unexpected error: " + e.getMessage());
            e.printStackTrace();
            return false;
        }
    }

    /**
     * �����ؽڽǶ��˶�����
     * ��ʽ��MOVE_JOINT j1 j2 j3 j4 j5 j6 j7 [velocity]
     */
    private boolean processJointCommand(String command, DataOutputStream outToClient, int commandNumber)
    {
        try
        {
            // ��������
            String[] parts = command.split("\\s+");
            
            if (parts.length < 8 || parts.length > 9)
            {
                String errorMsg = "ERROR: Invalid format. Need 7-8 values: MOVE_JOINT j1 j2 j3 j4 j5 j6 j7 [velocity]\n";
                outToClient.writeBytes(errorMsg);
                getLogger().error("Invalid joint command format: " + command);
                return false;
            }
            
            // ����7���ؽڽǶ�ֵ���ȣ�
            double[] jointsDeg = new double[7];
            for (int i = 0; i < 7; i++)
            {
                jointsDeg[i] = Double.parseDouble(parts[i + 1]);
            }
            
            // �����ٶȣ���ѡ��
            double velocity = (parts.length == 9) ? Double.parseDouble(parts[8]) : DEFAULT_VELOCITY_REL;
            velocity = Math.max(0.0, Math.min(1.0, velocity));
            
            getLogger().info(String.format("Command #%d: Moving to joint position [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f] deg, velocity=%.2f",
                    commandNumber, jointsDeg[0], jointsDeg[1], jointsDeg[2], jointsDeg[3], jointsDeg[4], jointsDeg[5], jointsDeg[6], velocity));
            
            // ִ���˶�
            try
            {
                // �ȷ���"�����ѽ���"��Ӧ����Python��֪�������ѿ�ʼִ��
                outToClient.writeBytes("OK: Command received, executing motion...\n");
                outToClient.flush();
                
                // ִ���˶����������ȴ��˶���ɣ�
                getLogger().info("Executing motion...");
                _lbr.getFlange().move(ptp(JointPosition.ofDeg(
                        jointsDeg[0], jointsDeg[1], jointsDeg[2], jointsDeg[3], 
                        jointsDeg[4], jointsDeg[5], jointsDeg[6])).setJointVelocityRel(velocity));
                
                // �˶���ɺ󣬻�ȡʵ�ʵ���Ĺؽ�λ��
                JointPosition actualJoints = _lbr.getCurrentJointPosition();
                // ʹ��toString()������ȡ�ؽ�λ���ַ���
                String jointStr = actualJoints.toString();
                
                // �����˶������Ӧ
                String successMsg = String.format("OK: Motion completed. Final joint position: %s\n", jointStr);
                outToClient.writeBytes(successMsg);
                outToClient.flush();
                getLogger().info("Motion completed successfully.");
                
                return true;
            }
            catch (Exception e)
            {
                String errorMsg = "ERROR: Motion execution failed: " + e.getMessage() + "\n";
                outToClient.writeBytes(errorMsg);
                getLogger().error("Motion execution error: " + e.getMessage());
                e.printStackTrace();
                return false;
            }
        }
        catch (NumberFormatException e)
        {
            String errorMsg = "ERROR: Invalid number format: " + e.getMessage() + "\n";
            try
            {
                outToClient.writeBytes(errorMsg);
            }
            catch (IOException ioEx)
            {
                getLogger().error("Error sending error message: " + ioEx.getMessage());
            }
            getLogger().error("Number format error: " + e.getMessage());
            return false;
        }
        catch (Exception e)
        {
            String errorMsg = "ERROR: " + e.getMessage() + "\n";
            try
            {
                outToClient.writeBytes(errorMsg);
            }
            catch (IOException ioEx)
            {
                getLogger().error("Error sending error message: " + ioEx.getMessage());
            }
            getLogger().error("Unexpected error: " + e.getMessage());
            e.printStackTrace();
            return false;
        }
    }

    /**
     * ������ȡλ������
     * ���أ�x y z alpha beta gamma����λ��mm, �ȣ�
     */
    private boolean processGetPoseCommand(DataOutputStream outToClient)
    {
        try
        {
            // ��ȡ��ǰĩ��λ�ˣ�����ڻ�����ϵ��
            ObjectFrame flangeFrame = _lbr.getFlange();
            Transformation transform = flangeFrame.calculateTransformationFromTreeRoot();
            
            Vector3D translation = transform.getTranslation();
            Rotation rotation = transform.getRotation();
            
            // ����ת����ת��Ϊŷ���ǣ�ZYX˳��
            double[] euler = rotationToEulerZYX(rotation);
            
            // ������Ӧ��ȷ���������ͣ�
            String response = String.format("OK: %.3f %.3f %.3f %.3f %.3f %.3f\n",
                    translation.getX(), translation.getY(), translation.getZ(),
                    euler[0], euler[1], euler[2]);
            outToClient.writeBytes(response);
            outToClient.flush(); // ȷ����������
            
            getLogger().info(String.format("Current pose: [%.3f, %.3f, %.3f] mm, [%.3f, %.3f, %.3f] deg",
                    translation.getX(), translation.getY(), translation.getZ(),
                    euler[0], euler[1], euler[2]));
            
            return true;
        }
        catch (Exception e)
        {
            String errorMsg = "ERROR: Failed to get pose: " + e.getMessage() + "\n";
            try
            {
                outToClient.writeBytes(errorMsg);
            }
            catch (IOException ioEx)
            {
                getLogger().error("Error sending error message: " + ioEx.getMessage());
            }
            getLogger().error("Error getting pose: " + e.getMessage());
            e.printStackTrace();
            return false;
        }
    }

    /**
     * ������ȡ�ؽڽǶ�����
     * ���أ�j1 j2 j3 j4 j5 j6 j7����λ���ȣ�
     */
    private boolean processGetJointCommand(DataOutputStream outToClient)
    {
        try
        {
            // ��ȡ��ǰ�ؽڽǶ�
            JointPosition currentJoints = _lbr.getCurrentJointPosition();
            
            // ����toString()�Ľ������ȡ�����ؽڽǶ�
            // JointPosition��toString()��ʽͨ����: "JointPosition[A1=xx.xx, A2=xx.xx, ...]"
            double[] jointsDeg = new double[7];
            String jointStr = currentJoints.toString();
            
            try
            {
                // ���Խ�����ʽ "JointPosition[A1=xx.xx, A2=xx.xx, ...]"
                // �Ƴ�ǰ׺�ͺ�׺
                String content = jointStr.trim();
                
                // Format 1: Array format "[-0.00001, 0.52366, 1.04727, ...]"
                if (content.startsWith("[") && content.endsWith("]"))
                {
                    content = content.substring(1, content.length() - 1); // Remove [ and ]
                    String[] parts = content.split(",");
                    
                    for (int i = 0; i < 7 && i < parts.length; i++)
                    {
                        jointsDeg[i] = Double.parseDouble(parts[i].trim());
                    }
                    
                    if (parts.length < 7)
                    {
                        throw new NumberFormatException("Expected 7 joint values, got " + parts.length);
                    }
                }
                // Format 2: "JointPosition[A1=xx.xx, A2=xx.xx, ...]"
                else if (content.startsWith("JointPosition["))
                {
                    content = content.substring("JointPosition[".length());
                }
                if (content.endsWith("]"))
                {
                    content = content.substring(0, content.length() - 1);
                }
                
                // �����ŷָ�
                String[] parts = content.split(",");
                
                // ����ÿ������
                for (int i = 0; i < 7 && i < parts.length; i++)
                {
                    String part = parts[i].trim();
                    if (part.contains("="))
                    {
                        // ��ʽ: "A1=xx.xx" �� "A1 = xx.xx"
                        String[] keyValue = part.split("=");
                        if (keyValue.length >= 2)
                        {
                            jointsDeg[i] = Double.parseDouble(keyValue[1].trim());
                        }
                        else
                        {
                            throw new NumberFormatException("Invalid format: " + part);
                        }
                    }
                    else
                    {
                        // ֱ������ֵ
                        jointsDeg[i] = Double.parseDouble(part);
                    }
                }
                
                // ��������Ĺؽ�������7��������
                if (parts.length < 7)
                {
                    throw new NumberFormatException("Expected 7 joint values, got " + parts.length);
                }
            }
            catch (Exception parseEx)
            {
                // �������ʧ�ܣ����ش���
                String errorMsg = "ERROR: Failed to parse joint position from: " + jointStr + 
                                 ". Error: " + parseEx.getMessage() + "\n";
                try
                {
                    outToClient.writeBytes(errorMsg);
                }
                catch (IOException ioEx)
                {
                    getLogger().error("Error sending error message: " + ioEx.getMessage());
                }
                getLogger().error("Error parsing joint position: " + parseEx.getMessage());
                getLogger().error("Joint string was: " + jointStr);
                return false;
            }
            
            // ������Ӧ��ȷ���������ͣ�
            String response = String.format("OK: %.3f %.3f %.3f %.3f %.3f %.3f %.3f\n",
                    jointsDeg[0], jointsDeg[1], jointsDeg[2], jointsDeg[3],
                    jointsDeg[4], jointsDeg[5], jointsDeg[6]);
            outToClient.writeBytes(response);
            outToClient.flush(); // ȷ����������
            
            getLogger().info(String.format("Current joint position: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f] deg",
                    jointsDeg[0], jointsDeg[1], jointsDeg[2], jointsDeg[3],
                    jointsDeg[4], jointsDeg[5], jointsDeg[6]));
            
            return true;
        }
        catch (Exception e)
        {
            String errorMsg = "ERROR: Failed to get joint position: " + e.getMessage() + "\n";
            try
            {
                outToClient.writeBytes(errorMsg);
            }
            catch (IOException ioEx)
            {
                getLogger().error("Error sending error message: " + ioEx.getMessage());
            }
            getLogger().error("Error getting joint position: " + e.getMessage());
            e.printStackTrace();
            return false;
        }
    }

    /**
     * ����ת����ת��Ϊŷ���ǣ�ZYX˳�򣬵�λ���ȣ�
     * ע�⣺Rotation�������û��getX(), getY(), getZ()����
     * ����ʹ��toString()������ȡ��ת��Ϣ�����߷���Ĭ��ֵ
     */
    private double[] rotationToEulerZYX(Rotation rotation)
    {
        // ����Rotation�������û��ֱ�ӵķ�����ȡ��ת����Ԫ��
        // ����ʹ��toString()������ȡ��ת��Ϣ
        // ���߷���Ĭ��ֵ���򻯴�����
        
        // �ο�LBRFlangePoseLogger����ֻ�Ǵ�ӡrotation����
        // �������Ƿ���Ĭ��ֵ�����߿��Խ���toString()�Ľ��
        String rotationStr = rotation.toString();
        getLogger().info("Rotation: " + rotationStr);
        
        // �򻯴�����������ֵ
        // �����Ҫ��ȡŷ���ǣ����Խ���rotationStr�ַ���
        return new double[] { 0.0, 0.0, 0.0 };
    }

    private static void closeQuietly(Socket socket)
    {
        if (socket != null && !socket.isClosed())
        {
            try
            {
                socket.close();
            }
            catch (IOException ex)
            {
                // ���Թرմ���
            }
        }
    }

    private static void closeQuietly(ServerSocket socket)
    {
        if (socket != null && !socket.isClosed())
        {
            try
            {
                socket.close();
            }
            catch (IOException ex)
            {
                // ���Թرմ���
            }
        }
    }
}


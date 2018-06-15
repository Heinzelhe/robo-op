/*

René Heijdens
heijdensrene@hotmail.com

ABB Yumi Robot Controller C#


Based on the work of mad & zack
(mad) www.madlab.cc
(zack)enartdezark.blogspot.com 

https://github.com/peopleplusrobots/robo-op  
 */

using System;
using System.Linq;
using System.Text;
using System.Net;
using System.Net.Sockets;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Robot : MonoBehaviour {

    public string IP_Address = "127.0.0.1"; //192.168.125.1 for 
    public int PortRightArm = 5000;
    public int PortLeftArm = 4000;

    Socket Rightarm;
    Socket Leftarm;

    //joints angles initilized

    private double a = 0.0;
    private double b = -50;
    private double c = -21.75;
    private double d = 0;
    private double e = 25;
    private double f = 0;
    private double g = 0;

    //current joint positions left
    private double la;
    private double lb;
    private double lc;
    private double ld;
    private double le;
    private double lf;
    private double lg;


    //current joint positions right
    private double ra;
    private double rb;
    private double rc;
    private double rd;
    private double re;
    private double rf;
    private double rg;

    //current coordinates left
    private double lx;
    private double ly;
    private double lz;

    private double lrx;
    private double lry;
    private double lrz;
    private double lro;

    private double lcf1;
    private double lcf4;
    private double lcf6;
    private double lcfx;

    private double leax;

    //current coordinates right
    private double rx;
    private double ry;
    private double rz;

    private double rrx;
    private double rry;
    private double rrz;
    private double rro;

    private double rcf1;
    private double rcf4;
    private double rcf6;
    private double rcfx;

    private double reax;

    private int offset;
    string RobotString;

    byte[] bytes = new byte[1024];

    int offsetDist = 10;

    /*
     *          Startfuncie maakt verbinding met de robot
     * 
     */

	void Start() {
        //Start making connection to both of the robotarms
    try
    {
        
        System.Net.IPAddress ipAddress = System.Net.IPAddress.Parse(IP_Address);
        IPEndPoint remoteRA = new IPEndPoint(ipAddress, PortRightArm);
        IPEndPoint remoteLA = new IPEndPoint(ipAddress, PortLeftArm);


        Rightarm = new Socket(AddressFamily.InterNetwork, SocketType.Stream, ProtocolType.Tcp);
        Leftarm = new Socket(AddressFamily.InterNetwork, SocketType.Stream, ProtocolType.Tcp);

        try
        {
            
            Rightarm.Connect(remoteRA);
            Leftarm.Connect(remoteLA);
            print("Robot Connected");



            print("Initialize robot");          //Beginpositie van de robot          
            Initialized();

        }
        catch (ArgumentNullException ane)       //errors during connection
        {
            Console.WriteLine("ArgumentNullException : {0}", ane.ToString());
        }
        catch (SocketException se)
        {
            Console.WriteLine("SocketException : {0}", se.ToString());
        }
        catch (Exception e)
        {
            Console.WriteLine("Unexpected exception : {0}", e.ToString());
        }
    }
    catch (Exception e)
    {
        Console.WriteLine(e.ToString());
    }
}

    /*
     * Voor nu nog een test functie. Later gaat Update weg. Dan kan functies worden opgeroepen dmv Robot.Right... of Robot.Left....
     * 
     */

    private void Update()
    {

        if (Input.anyKey)
            {
                if (Input.GetKey("up"))
                {
                    lx = lx - offset;
                    LeftsetPosition(lx, ly, lz);
                }

                if (Input.GetKey("down"))
                {
                    lx = lx + offset;
                LeftsetPosition(lx, ly,  lz);
                }
                if (Input.GetKey("right"))
                {
                    ly = ly + offset;
                    LeftsetPosition(lx, ly, lz);
                }
                if (Input.GetKey("left"))
                {
                    ly = ly - offset;
                    LeftsetPosition(lx, ly, lz);
                }
                if (Input.GetKey("o"))
                {
                    lz = lz + offset;
                    LeftsetPosition(lx, ly, lz);
                }
                if (Input.GetKey("l"))
                {
                    lz = lz - offset;
                    LeftsetPosition(lx, ly, lz);
                }
                if (Input.GetKey("w"))
               {
                    rx = rx - offset;
                    RightsetPosition(rx, ry, rz);
                }
        
                if (Input.GetKey("s"))
                {
                    rx = rx + offset;
                    RightsetPosition(rx, ry, rz);
                }
                if (Input.GetKey("d"))
                {
                    ry = ry + offset;
                    RightsetPosition(rx, ry, rz);
                }
                if (Input.GetKey("a"))
                {
                    ry = ry - offset;
                    RightsetPosition(rx, ry, rz);
                }
                if (Input.GetKey("r"))
                {
                    rz = rz + offset;
                    RightsetPosition(rx, ry, rz);
                }
                if (Input.GetKey("f"))
                {
                    rz = rz - offset;
                    RightsetPosition(rx, ry, rz);
                }
                if (Input.GetKey("i")) {
                    LeftsetJoints(a, b, c, d, e, f, g);
                    RightsetJoints(a, b, c, d, e, f, g);
                }
                if (Input.GetKey("z")) {
                    updateStatus();
                    getStatus();
                }
                if (Input.GetKey("x"))
                {
                    RightsetCartesian(rx, ry, rz, rrx, rry, rrz, rro, rcf1, rcf4, rcf6, rcfx, reax);
                    LeftsetCartesian(lx, ly, lz, lrx, lry, lrz, lro, lcf1, lcf4, lcf6, lcfx, leax);
                }
                if (Input.GetKey("c"))
                {
                    LeftsetCartesian(302.1, 151.7, 137.4, 0.01, 0.15, -0.73, 0.67, -1, 0, 0, 4, 160.7);
                    RightsetCartesian(295.6, -154.3, 108.3, 0.05, -0.06, -0.70, -0.71, 0, -1, -1, 4, -157.9);
                }
                if (Input.GetKey("1")) {
                    offset = 1;
                }
                if (Input.GetKey("2"))
                {
                    offset = 5;
                }
                if (Input.GetKey("3"))
                {
                    offset = 10;
                }
        }
        if (!Input.anyKey)
        {
            //ping();
        }

    }


    /*
     * 
     *  Set functions.
     *  
     * 
     * 
     */

    public void setSpeed(int tool, int orient, int extLinear, int extRot)
    {
        String key = "speed";
        String val = "[" + tool + "," + orient + "," + extLinear + "," + extRot + "]";
        RightsendMessage((key + "/" + val + ";"), true);
        LeftsendMessage((key + "/" + val + ";"), true);
    }

    public void RightsetOrientation(double rx, double ry, double rz)
    {
        String key = "orient";
        String val = "[" + rx + "," + ry + "," + rz + "]";
        RightsendMessage((key + "/" + val + ";"), true);
    }

    public void LeftsetOrientation(double rx, double ry, double rz)
    {
        String key = "orient";
        String val = "[" + rx + "," + ry + "," + rz + "]";
        LeftsendMessage((key + "/" + val + ";"), true);
    }

    public void RightsetPosition(double x, double y, double z)
    {
        String key = "pos";
        String val = "[" + x + "," + y + "," + z + "]";
        RightsendMessage((key + "/" + val + ";"), true);
    }

    public void LeftsetPosition(double x, double y, double z)
    {
        String key = "pos";
        String val = "[" + x + "," + y + "," + z + "]";
        LeftsendMessage((key + "/" + val + ";"), true);
    }

    public void setZone(String z)
    {
        String key = "zone";
        String val = z;
        RightsendMessage((key + "/" + val + ";"), true);
        LeftsendMessage((key + "/" + val + ";"), true);
    }

    public void RightmoveOffset(double x, double y, double z, double rx, double ry, double rz)
    {
        String key = "offset";
        String val = "[" + x + "," + y + "," + z + "," + rx + "," + ry + "," + rz + "]";
        RightsendMessage((key + "/" + val + ";"), true);
    }

    public void LeftmoveOffset(double x, double y, double z, double rx, double ry, double rz)
    {
        String key = "offset";
        String val = "[" + x + "," + y + "," + z + "," + rx + "," + ry + "," + rz + "]";
        LeftsendMessage((key + "/" + val + ";"), true);
        
    }

    public void LeftsetCartesian(double x, double y, double z, double rx, double ry, double rz, double ro,
        double cf1, double cf4, double cf6, double cfx,
        double eax
        )
    {
        String key = "ct";
        String val = "[" + x + "," + y + "," + z + "," + rx + "," + ry + "," + rz + "," + ro + "," 
            + cf1 + "," + cf4 + "," + cf6 + ","  + cfx + "," +
            eax + "]";
        LeftsendMessage((key + "/" + val + ";"), true);
    }

    public void RightsetCartesian(double x, double y, double z, double rx, double ry, double rz, double ro,
        double cf1, double cf4, double cf6, double cfx,
        double eax
        )
    {
        String key = "ct";
        String val = "[" + x + "," + y + "," + z + "," + rx + "," + ry + "," + rz + "," + ro + ","
            + cf1 + "," + cf4 + "," + cf6 + "," + cfx + "," +
            eax + "]";
        RightsendMessage((key + "/" + val + ";"), true);
    }

    public void LeftsetJoints(double A, double B, double C, double D, double E, double F, double G)
    {
        String key = "gewr";
        String val = "[" + A + "," + B + "," + C + "," + D + "," + E + "," + F + "," + G + "]";
        LeftsendMessage((key + "/" + val + ";"), true);
    }

    public void RightsetJoints(double A, double B, double C, double D, double E, double F, double G)
    {
        String key = "gewr";
        String val = "[" + A + "," + B + "," + C + "," + D + "," + E + "," + F + "," + G + "]";
        RightsendMessage((key + "/" + val + ";"), true);
    }

    /*
     * 
     *  get functions.
     *  Krijg de huidige status van de robots' positie binnen
     * 
     * 
     */

    public void LeftgetCartesian()
    {
        

        String key = "query";
        String val = "gcart";
        LeftsendMessage((key + "/" + val + ";"), true);

        // block until you receive the new data from robot
        String msg = LeftmessageReceived();
        print(msg);

        string[] returnValues = msg.Split(';');

        lx = Double.Parse(returnValues[0]);
        ly = Double.Parse(returnValues[1]);
        lz = Double.Parse(returnValues[2]);

        lrx = Double.Parse(returnValues[3]);
        lry = Double.Parse(returnValues[4]);
        lrz = Double.Parse(returnValues[5]);
        lro = Double.Parse(returnValues[6]);

        lcf1 = Double.Parse(returnValues[7]);
        lcf4 = Double.Parse(returnValues[8]);
        lcf6 = Double.Parse(returnValues[9]);
        lcfx = Double.Parse(returnValues[10]);

        leax = Double.Parse(returnValues[11]);

    }

    public void RightgetCartesian()
    {


        String key = "query";
        String val = "gcart";
        RightsendMessage((key + "/" + val + ";"), true);

        // block until you receive the new data from robot
        String msg = RightmessageReceived();
        print(msg);

        string[] returnValues = msg.Split(';');

        rx = Double.Parse(returnValues[0]);
        ry = Double.Parse(returnValues[1]);
        rz = Double.Parse(returnValues[2]);

        rrx = Double.Parse(returnValues[3]);
        rry = Double.Parse(returnValues[4]);
        rrz = Double.Parse(returnValues[5]);
        rro = Double.Parse(returnValues[6]);

        rcf1 = Double.Parse(returnValues[7]);
        rcf4 = Double.Parse(returnValues[8]);
        rcf6 = Double.Parse(returnValues[9]);
        rcfx = Double.Parse(returnValues[10]);

        reax = Double.Parse(returnValues[11]);


    }

    public void RightgetJoints()
    {


        String key = "query";
        String val = "gjoints";
        RightsendMessage((key + "/" + val + ";"), true);

        // block until you receive the new data from robot
        String msg = RightmessageReceived();
        print("Joints position: " + msg);

        string[] returnValues = msg.Split(';');


        ra = Double.Parse(returnValues[0]);
        rb = Double.Parse(returnValues[1]);
        rc = Double.Parse(returnValues[2]);
        rd = Double.Parse(returnValues[3]);
        re = Double.Parse(returnValues[4]);
        rf = Double.Parse(returnValues[5]);
        rg = Double.Parse(returnValues[6]);



    }

    public void LeftgetJoints()
    {

        String key = "query";
        String val = "gjoints";
        LeftsendMessage((key + "/" + val + ";"), true);

        // block until you receive the new data from robot
        String msg = LeftmessageReceived();
        //print("Joints position: " + msg);
        string [] returnValues = msg.Split(';');

        
        la = Double.Parse(returnValues[0]);
        lb = Double.Parse(returnValues[1]);
        lc = Double.Parse(returnValues[2]);
        ld = Double.Parse(returnValues[3]);
        le = Double.Parse(returnValues[4]);
        lf = Double.Parse(returnValues[5]);
        lg = Double.Parse(returnValues[6]);

    }

    public String LeftgetPos()
    {


        String key = "query";
        String val = "pos";
        LeftsendMessage((key + "/" + val + ";"), true);

        // block until you receive the new data from robot
        String msg = LeftmessageReceived();
        print("positure: " + msg);

        return msg;

    }

    public String RightgetPos()
    {


        String key = "query";
        String val = "pos";
        RightsendMessage((key + "/" + val + ";"), true);

        // block until you receive the new data from robot
        String msg = RightmessageReceived();
        print("positure: " + msg);

        return msg;

    }

    public String LeftgetZone()
    {
        String key = "query";
        String val = "zone";
        LeftsendMessage((key + "/" + val + ";"), true);

        // block until you receive the new data from robot
        String msg = LeftmessageReceived();

        print("robot zone: " + msg);
        return msg;
    }

    public String RightgetZone()
    {
        String key = "query";
        String val = "zone";
        RightsendMessage((key + "/" + val + ";"), true);

        // block until you receive the new data from robot
        String msg = RightmessageReceived();

        print("robot zone: " + msg);
        return msg;
    }

    /*
    * 
    *  Overige functions.
    *  Functies voor de communicatie tussen robot en PC
    * 
    * 
    */

    private String LeftmessageReceived()
    {
        int? temp = null;

        int RobotValue;
        try
        {
            int? roboInput = Leftarm.Receive(bytes);
            while (roboInput != null)
            {
                temp = roboInput;
                RobotValue = roboInput.Value;
                //print(temp);
                roboInput = null;
                RobotString = Encoding.ASCII.GetString(bytes, 0, RobotValue);
                //print(RobotString);
            }
        }
        catch (SocketException e)
        {
            print(e);
        }
        catch (ArgumentNullException ex)
        {
            print(ex);
        }
        return RobotString;
    }

    private String RightmessageReceived()
    {
        int? temp = null;

        int RobotValue;
        try
        {
            int? roboInput = Rightarm.Receive(bytes);
            while (roboInput != null)
            {
                temp = roboInput;
                RobotValue = roboInput.Value;
                //print(temp);
                roboInput = null;
                RobotString = Encoding.ASCII.GetString(bytes, 0, RobotValue);
                //print(RobotString);
            }
        }
        catch (SocketException e)
        {
            print(e);
        }
        catch (ArgumentNullException ex)
        {
            print(ex);
        }
        return RobotString;
    }

    public String RightsendMessage(String msg, bool wait)
    {
        byte[] msgbyte = Encoding.ASCII.GetBytes(msg);
        int bytesSent = Rightarm.Send(msgbyte);


        if (wait)
            return RightmessageReceived();
        else
            return "";
    }

    public String LeftsendMessage(String msg, bool wait)
    {
        byte[] msgbyte = Encoding.ASCII.GetBytes(msg);
        int bytesSent = Leftarm.Send(msgbyte);


        if (wait)
            return LeftmessageReceived();
        else
            return "";
    }

    public void Initialized()
    {
        setSpeed(150, 100, 100, 100);
        LeftsetJoints(a, b, c, d, e, f, g);
        RightsetJoints(a, b, c, d, e, f, g);
        LeftsetCartesian(300, 150, 100, 0.00, 0.15, -0.70, 0.70, -1, 0, 0, 4, 160.7);
        RightsetCartesian(300, -150, 100, 0.00, -0.15, -0.70, -0.70, 0, -1, -1, 4, -157.9);
        offset = 10;

        updateStatus();

    }

    public void updateStatus() {
        LeftgetJoints();
        RightgetJoints();
        LeftgetCartesian();
        RightgetCartesian();
        
    }

    public void getStatus()
    {

        print("lx" + lx.ToString());
        print("ly" + ly.ToString());
        print("lz" + lz.ToString());


        print("rx" + rx.ToString());
        print("ry" + ry.ToString());
        print("rz" + rz.ToString());


        print("left Joints");
        print(la.ToString());
        print(lb.ToString());
        print(lc.ToString());
        print(ld.ToString());
        print(le.ToString());
        print(lf.ToString());
        print(lg.ToString());


        print("right Joints");
        print(ra.ToString());
        print(rb.ToString());
        print(rc.ToString());
        print(rd.ToString());
        print(re.ToString());
        print(rf.ToString());
        print(rg.ToString());

    }

    public void ping()
    {
        String key = "query";
        String val = "ping";
        LeftsendMessage((key + "/" + val + ";"), true);
        String msgl = LeftmessageReceived();
        RightsendMessage((key + "/" + val + ";"), true);
        String msgr = RightmessageReceived();

    }
}

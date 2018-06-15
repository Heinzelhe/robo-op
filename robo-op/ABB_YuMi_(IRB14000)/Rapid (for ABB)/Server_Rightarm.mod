MODULE Server_Rightarm

    !***********************************************************
    !
    !
    ! RAPID module to remotely send commands to an industrial robot from an external
    ! application.
    !
    ! Code adapted from open-abb-driver {https://github.com/robotics/open-abb-driver}
    !  
    !
    ! @authors mad & zack
    ! (mad) www.madlab.cc
    ! (zack)enartdezark.blogspot.com
    !
    !   Edited for ABB YuMi By René Heijdens heijdensrene@hotmail.com
    !
    !***********************************************************

    ! Setup default tool, work object, speed, and zone data
    PERS tooldata currToolR:=[TRUE,[[0,0,114.2],[1,0,0,0]],[0.3,[8.2,12.5,48.1],[1,0,0,0],0.00022,0.00024,9E-05]];
    PERS wobjdata currWobjR:=[FALSE,TRUE,"",[[0,0,0],[1,0,0,0]],[[0,0,0],[1,0,0,0]]];
    VAR speeddata currSpeedR; !:=[100,50,0,0];
    VAR zonedata currZoneR; !:=z0;


    ! Setup start pose and position
    CONST jointtarget homePoseR:=[[0,0,0,0,0,0],[0,0,0,0,0,0]];
    CONST robtarget StartTargetR:=[[0,0,0],[1,0,0,0],[-2,-1,0,0],[84.5,9E9,9E9,9E9,9E9,9E9]];

    ! Setup program variables
    VAR pos currPosR;
    VAR robtarget currTargR;
    VAR jointtarget jointsTargetR;
    VAR robtarget cartesianPoseR;
    VAR string addStringR;

    ! Setup communication variables
    VAR socketdev temp_socketR;
    VAR socketdev client_socketR;
    VAR bool connectedR:=FALSE;
    VAR num coords{10};
    VAR string msg_receivedR;
    VAR bool listenR:=TRUE;
    

    
    !//External axis position variables
    VAR extjoint externalAxisR;
   

    !listen to incomming messages from Unity
    PROC Main()                                 

        ConfL\On;
        SingArea \Wrist;
        ConfJ\On;
        !//Initialization of WorkObject, Tool, Speed and Zone
        Initialize;
        
        
        currPosR:=CPos();
        currTargR:=CRobT();
        

        
        ! Connect Server to Client
        ConnectToClient;
        connectedR:=TRUE;

        WHILE listenR DO

            !SocketSend client_socketR\Str:="ready for msg, Computer!\0D\0A";
            ! SocketReceive blocks until it gets a msg from the client
            SocketReceive client_socketR\Str:=msg_receivedR;
            TPWrite "Incoming Msg:  "+msg_receivedR;
            ! Verify to client that we received the message
            SocketSend client_socketR\Str:="\0D\0A";!received msg

            ! parse the message received from the client
            ParseMsg msg_receivedR;

            ! clear the received message
            msg_receivedR:="";

        ENDWHILE

        SocketSend client_socketR\Str:="Closing Server Socket\0D\0A";
        TPWrite "Closing Server Socket";
        SocketClose temp_socketR;

    ENDPROC
    
PROC Initialize()
    currToolR := [TRUE,[[0,0,114.2],[1,0,0,0]],[0.3,[8.2,12.5,48.1],[1,0,0,0],0.00022,0.00024,9E-05]];
    currWobjR := [FALSE,TRUE,"",[[0,0,0],[1,0,0,0]],[[0,0,0],[1,0,0,0]]];
    currSpeedR := [100, 50, 0, 0];
    currZoneR := [FALSE, 0.3, 0.3,0.3,0.03,0.3,0.03]; !z0
	
	!Find the current external axis values so they don't move when we start
	jointsTargetR := CJointT();
	externalAxisR := jointsTargetR.extax;
ENDPROC

    !Setting up Socket connection C#
    PROC ConnectToClient()
        VAR string clientIP;

        SocketCreate temp_socketR;
        SocketBind temp_socketR,"192.168.125.1",5000;
        SocketListen temp_socketR;
        SocketAccept temp_socketR,client_socketR\ClientAddress:=clientIP;

        TPWrite "SERVER: Connected to IP "+clientIP;
    ENDPROC

    !check command from incomming string, In case of key=query, send info back. Key!=query, go to function.
    PROC ParseMsg(string msg)
        VAR num msgLength;
        VAR bool badMsg:=FALSE;
        VAR bool done:=FALSE;
        VAR string key;
        VAR string val;
        VAR num split;
        VAR num dataType;
        VAR num pin;
        VAR string msgSend;

        ! Check that we received a full message
        msgLength:=StrMatch(msg,1,";");
        IF msgLength>StrLen(msg) THEN
            badMsg:=TRUE;
        ENDIF

        IF badMsg=TRUE THEN
            TPWrite "corrupt or incomplete message";
            TPWrite msg;
        ELSE
            split:=StrMatch(msg,1,"/");
            key:=StrPart(msg,1,split-1);
            TPWrite "key: "+key;
            val:=StrPart(msg,split+1,msgLength-split-1);
            TPWrite "val: "+val;

            WHILE done=FALSE DO

                ! Check which data type we are trying to parse

                ! JOINT modifies the Position and Orientation (MoveJ)
                dataType:=StrMatch(key,1,"joint");
                IF dataType<StrLen(key) THEN
                    TPWrite "moveto";
                    moveTo(val);
                    done:=TRUE;
                    GOTO end;
                ENDIF
                
                ! RENE EDIT ######################################################################################
                !Set cartesian coordinates
                dataType:=StrMatch(key,1,"ct");
                IF dataType<StrLen(key) THEN
                    setCartesian(val);
                    done:=TRUE;
                    GOTO end;
                ENDIF
                
                dataType:=StrMatch(key,1,"pro");
                IF dataType<StrLen(key) THEN
                    setProper(val);
                    done:=TRUE;
                    GOTO end;
                ENDIF
                
                !Gripper Close
                dataType:=StrMatch(key,1,"gclo");
                IF dataType<StrLen(key) THEN
                    gripperClose(val);
                    done:=TRUE;
                    GOTO end;
                ENDIF
                
                !Gripper Open
                dataType:=StrMatch(key,1,"gope");
                IF dataType<StrLen(key) THEN
                    gripperOpen(val);
                    done:=TRUE;
                    GOTO end;
                ENDIF
                
                !Set joints
                dataType:=StrMatch(key,1,"gewr");
                IF dataType<StrLen(key) THEN
                    TPWrite "going to set joints";
                    setJoints(val);
                    done:=TRUE;
                    GOTO end;
                ENDIF
                ! END RENE EDIT #################################################################################
                
                ! OFFSET moves the robot relative to a tool position (MoveJ)
                dataType:=StrMatch(key,1,"offset");
                IF dataType<StrLen(key) THEN
                    moveRelTool(val);
                    done:=TRUE;
                    GOTO end;
                ENDIF
                
                ! POS modifies the Position of the robot (MoveL)
                dataType:=StrMatch(key,1,"pos");
                IF dataType<StrLen(key) THEN
                    updatePosition(val);
                    done:=TRUE;
                    GOTO end;
                ENDIF
                
                ! ORIENT modifies the Orient of the robot (MoveL)
                dataType:=StrMatch(key,1,"orient");
                IF dataType<StrLen(key) THEN
                    updateOrientation(val);
                    done:=TRUE;
                    GOTO end;
                ENDIF
                
                !CONFIG modifies the robot's configuration (MoveJ)
                dataType:=StrMatch(key,1,"config");
                IF dataType<StrLen(key) THEN
                    updateConfig(val);
                    done:=TRUE;
                    GOTO end;
                ENDIF
                
                !EXTAX modifies external axes (MoveL)
                dataType:=StrMatch(key,1,"extax");
                IF dataType<StrLen(key) THEN
                    updateExternalAxes(val);
                    done:=TRUE;
                    GOTO end;
                ENDIF
              

                ! ZONE sets a new zone for the robot's movements
                dataType:=StrMatch(key,1,"zone");
                IF dataType<StrLen(key) THEN
                    IF StrToVal(val,currZoneR) THEN
                    ENDIF
                    done:=TRUE;
                    GOTO end;
                ENDIF

                ! SPEED sets a new zone for the robot's movements
                dataType:=StrMatch(key,1,"speed");
                IF dataType<StrLen(key) THEN
                    IF StrToVal(val,currSpeedR) THEN
                    ENDIF
                    done:=TRUE;
                    GOTO end;
                ENDIF

                

                ! WAIT pauses the program until a given signal
                dataType:=StrMatch(key,1,"wait");
                IF dataType<StrLen(key) THEN 

                    dataType:=StrMatch(val,1,"InPos");
                    IF dataType<StrLen(val) THEN                  
                        WaitRob \InPos;
                        done:=TRUE;
                        GOTO end;
                    ENDIF

                    ! Add more wait types below

                ENDIF 

                ! FLAG changes a program state
                dataType:=StrMatch(key,1,"flag");
                IF dataType<StrLen(key) THEN

                    dataType:=StrMatch(val,1,"exit");
                    IF dataType<StrLen(val) THEN
                        listenR:=FALSE;
                        done:=TRUE;
                        GOTO end;
                    ENDIF
                    
                    ! Add more flags below
                    
                ENDIF

                ! QUERY asks the robot to send information back to the computer
                dataType:=StrMatch(key,1,"query");
                IF dataType<StrLen(key) THEN

                    dataType:=StrMatch(val,1,"pos");
                    IF dataType<StrLen(val) THEN
                        msgSend:=ValToStr(currTargR.trans);
                        SocketSend client_socketR\Str:=msgSend+"\0D\0A";
                        done:=TRUE;
                        GOTO end;
                    ENDIF

                    dataType:=StrMatch(val,1,"orient");
                    IF dataType<StrLen(val) THEN
                        msgSend:=ValToStr(currTargR.rot);
                        SocketSend client_socketR\Str:=msgSend+"\0D\0A";
                        done:=TRUE;
                        GOTO end;
                    ENDIF

                    dataType:=StrMatch(val,1,"config");
                    IF dataType<StrLen(val) THEN
                        msgSend:=ValToStr(currTargR.robconf);
                        SocketSend client_socketR\Str:=msgSend+"\0D\0A";
                        done:=TRUE;
                        GOTO end;
                    ENDIF

                    dataType:=StrMatch(val,1,"extax");
                    IF dataType<StrLen(val) THEN
                        msgSend:=ValToStr(currTargR.extax);
                        SocketSend client_socketR\Str:=msgSend+"\0D\0A";
                        done:=TRUE;
                        GOTO end;
                    ENDIF

                    dataType:=StrMatch(val,1,"speed");
                    IF dataType<StrLen(val) THEN
                        msgSend:=ValToStr(currSpeedR);
                        TPWrite "speed: "+msgSend;
                        SocketSend client_socketR\Str:=msgSend+"\0D\0A";
                        done:=TRUE;
                        GOTO end;
                    ENDIF

                    dataType:=StrMatch(val,1,"zone");
                    IF dataType<StrLen(val) THEN
                        msgSend:=ValToStr(currZoneR);
                        TPWrite "zone: "+msgSend;
                        SocketSend client_socketR\Str:=msgSend+"\0D\0A";
                        done:=TRUE;
                        GOTO end;
                    ENDIF
                    
                    !RENE EDIT      #######################################################################################
                    !send cartesian coordinates
                    dataType:=StrMatch(val,1,"gcart");
                    IF dataType<StrLen(val) THEN
                        cartesianPoseR := CRobT(\Tool:=currToolR \WObj:=currWobjR);		!linkt de pose
                        addStringR := NumToStr(cartesianPoseR.trans.x,1) + ";";
                        addStringR := addStringR + NumToStr(cartesianPoseR.trans.y,1) + ";";
                        addStringR := addStringR + NumToStr(cartesianPoseR.trans.z,1) + ";";
                        addStringR := addStringR + NumToStr(cartesianPoseR.rot.q1,2) + ";";
                        addStringR := addStringR + NumToStr(cartesianPoseR.rot.q2,2) + ";";
                        addStringR := addStringR + NumToStr(cartesianPoseR.rot.q3,2) + ";";
                        addStringR := addStringR + NumToStr(cartesianPoseR.rot.q4,2) + ";"; 
                        
                        addStringR := addStringR + NumToStr(cartesianPoseR.robconf.cf1,0) + ";";
                        addStringR := addStringR + NumToStr(cartesianPoseR.robconf.cf4,0) + ";";
                        addStringR := addStringR + NumToStr(cartesianPoseR.robconf.cf6,0) + ";";
                        addStringR := addStringR + NumToStr(cartesianPoseR.robconf.cfx,0) + ";";
                        
                        addStringR := addStringR + NumToStr(cartesianPoseR.extax.eax_a,1); !einde van alle variabelen
                        SocketSend client_socketR\Str:=addStringR;
                        
                        GOTO end;
                    ENDIF
                    
                    !send position of the joints
                    dataType:=StrMatch(val,1,"gjoints");
                    IF dataType<StrLen(val) THEN
                        jointsTargetR := CJointT();
                        addStringR := NumToStr(jointsTargetR.robax.rax_1,2) + ";";
                        addStringR := addStringR + NumToStr(jointsTargetR.robax.rax_2,2) + ";";
                        addStringR := addStringR + NumToStr(jointsTargetR.robax.rax_3,2) + ";";
                        addStringR := addStringR + NumToStr(jointsTargetR.robax.rax_4,2) + ";";
                        addStringR := addStringR + NumToStr(jointsTargetR.robax.rax_5,2) + ";";
                        addStringR := addStringR + NumToStr(jointsTargetR.robax.rax_6,2) + ";"; 
                        addStringR := addStringR + NumToStr(jointsTargetR.extax.eax_a,2);!End of string
                        SocketSend client_socketR\Str:=addStringR;
                        
                        GOTO end;
                    ENDIF
                    
                    dataType:=StrMatch(val,1,"ping");
                    IF dataType<StrLen(val) THEN
                        addStringR := " ";
                        SocketSend client_socketR\Str:=addStringR;
                        
                        GOTO end;
                    ENDIF

                    !END RENE EDIT #######################################################################################

                ENDIF

                end:

                ! if we haven't found an appropriate command, send a message to the computer
                ! that they're sending the wrong key
                IF done=FALSE THEN
                    TPWrite "key ["+key+"] could not be found";
                    !SocketSend client_socketR\Str:="key ["+key+"] could not be found"+"\0D\0A";
                    done:=TRUE;
                ENDIF

            ENDWHILE

        ENDIF

    ENDPROC

    
    
    !Functions
    PROC moveRelTool(String val)
        VAR num coords{6};

        IF StrToVal(val,coords) THEN
            MoveJ RelTool(currTargR,coords{1},coords{2},coords{3}\Rx:=coords{4}\Ry:=coords{5}\Rz:=coords{6}),currSpeedR,currZoneR,currToolR;
            currPosR:=CPos();
            currTargR:=CRobT();
        ELSE
            TPWrite "Improper msg format: "+val;
        ENDIF
    ENDPROC 
    
    
    !#######################################################################################################
    PROC setCartesian(String val)
        VAR num coords{12};

        IF StrToVal(val,coords) THEN
            currTargR :=[[coords{1},coords{2},coords{3}],
                            [coords{4},coords{5},coords{6},coords{7}],
                            [coords{8},coords{9},coords{10},coords{11}],
                            [coords{12},9E9,9E9,9E9,9E9,9E9]];
            MoveL currTargR, currSpeedR, currZoneR, currToolR \WObj:=currWobjR ;
        ELSE
            TPWrite "Improper msg format: "+val;
        ENDIF
                    
    ENDPROC
    
    PROC setProper(String val)
        VAR num coords{8};

        IF StrToVal(val,coords) THEN
            currTargR.trans :=[coords{1},coords{2},coords{3}];
            currTargR.rot := [coords{4},coords{5},coords{6},coords{7}];
            currTargR.extax := [coords{12},9E9,9E9,9E9,9E9,9E9];
            MoveL currTargR, currSpeedR, currZoneR, currToolR \WObj:=currWobjR ;
        ELSE
            TPWrite "Improper msg format: "+val;
        ENDIF
                    
    ENDPROC
    
    PROC setJoints(String val)
        VAR num coords{7};
        
        
        IF StrToVal(val,coords) THEN
            TPWrite "Set Joints";
            jointsTargetR.robax:= [coords{1},coords{2},coords{3},coords{4},coords{5},coords{6}];
            jointsTargetR.extax:= [coords{7},9E9,9E9,9E9,9E9,9E9];
            MoveAbsJ jointsTargetR, currSpeedR, currZoneR, currToolR \Wobj:=currWobjR;
        ELSE
            TPWrite "Improper msg format: "+val;
        ENDIF
            !#######################################################################################################

    ENDPROC
    
    PROC gripperClose(String val)
            g_GripIn;
    ENDPROC
    
    PROC gripperOpen(String val)
            g_GripOut;
    ENDPROC
    
    
    
    PROC moveTo(String val)
        VAR num coords{6};
        VAR robtarget target:=[[0,0,0],[1,0,0,0],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
        VAR jointtarget testJoint;
        VAR robtarget testTarget;

        IF StrToVal(val,coords) THEN
            target.trans:=[coords{1},coords{2},coords{3}];
            target.rot:=OrientZYX(coords{6},coords{5},coords{4});
            testJoint:=CJointT();
            testTarget:=CalcRobT(testJoint,currToolR);
            target.robconf:=testTarget.robconf;

            MoveJ target,currSpeedR,currZoneR,currToolR;

            currPosR:=CPos();
            currTargR:=CRobT();
        ELSE
            TPWrite "Improper msg format: "+val;
        ENDIF
    ENDPROC

     PROC updatePosition(String val)
        VAR pos position;

        IF StrToVal(val,position) THEN
            currTargR.trans:=position;
            MoveJ currTargR,currSpeedR,currZoneR,currToolR;
            currPosR:=CPos();
            currTargR:=CRobT();
            SocketSend client_socketR\Str:="Done";
        ELSE
            TPWrite "Improper msg format: "+val;
        ENDIF
    ENDPROC
    
     PROC updateOrientation(String val)
        VAR num rots{3};

        IF StrToVal(val,rots) THEN        
            currTargR.rot:=OrientZYX(rots{3},rots{2},rots{1});
            MoveL currTargR,currSpeedR,currZoneR,currToolR;
            currPosR:=CPos();
            currTargR:=CRobT();
        ELSE
            TPWrite "Improper msg format: "+val;
        ENDIF
    ENDPROC
    
    PROC updateConfig(String val)
        VAR confdata config;

        IF StrToVal(val,config) THEN
            currTargR.robconf:=config;
            MoveJ currTargR,currSpeedR,currZoneR,currToolR;
            currPosR:=CPos();
            currTargR:=CRobT();
        ELSE
            TPWrite "Improper msg format: "+val;
        ENDIF
    ENDPROC

    PROC updateExternalAxes(String val)
        VAR extjoint extax;

        IF StrToVal(val,extax) THEN
            TPWrite "Moving external axis:";
            currTargR.extax:=extax;
            MoveL currTargR,currSpeedR,currZoneR,currToolR;
            currPosR:=CPos();
            currTargR:=CRobT();
        ELSE
            TPWrite "Improper msg format: "+val;
        ENDIF
    ENDPROC

ENDMODULE
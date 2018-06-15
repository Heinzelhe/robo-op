MODULE Server_Leftarm

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
    PERS tooldata currTool:=[TRUE,[[0,0,114.2],[1,0,0,0]],[0.3,[8.2,12.5,48.1],[1,0,0,0],0.00022,0.00024,9E-05]];
    PERS wobjdata currWobj:=[FALSE,TRUE,"",[[0,0,0],[1,0,0,0]],[[0,0,0],[1,0,0,0]]];
    VAR speeddata currSpeed; !:=[100,50,0,0];
    VAR zonedata currZone; !:=z0;


    ! Setup start pose and position
    CONST jointtarget homePose:=[[0,0,0,0,0,0],[0,0,0,0,0,0]];
    CONST robtarget StartTarget:=[[0,0,0],[1,0,0,0],[-2,-1,0,0],[84.5,9E9,9E9,9E9,9E9,9E9]];

    ! Setup program variables
    VAR pos currPosL;
    VAR robtarget currTargL;
    VAR jointtarget jointsTarget;
    VAR robtarget cartesianPose;
    VAR string addString;

    ! Setup communication variables
    VAR socketdev temp_socket;
    VAR socketdev client_socket;
    VAR bool connected:=FALSE;
    VAR num coords{10};
    VAR string msg_received;
    VAR bool listen:=TRUE;
    

    
    !//External axis position variables
    VAR extjoint externalAxis;

    !listen to incomming messages from Unity
    PROC Main()                                 

        ConfL\On;
        SingArea \Wrist;
        ConfJ\On;
        !//Initialization of WorkObject, Tool, Speed and Zone
        Initialize;
        
        
        currPosL:=CPos();
        currTargL:=CRobT();
        

        
        ! Connect Server to Client
        ConnectToClient;
        connected:=TRUE;

        WHILE listen DO

            !SocketSend client_socket\Str:="ready for msg, Computer!\0D\0A";
            ! SocketReceive blocks until it gets a msg from the client
            SocketReceive client_socket\Str:=msg_received;
            TPWrite "Incoming Msg:  "+msg_received;
            ! Verify to client that we received the message
            SocketSend client_socket\Str:="\0D\0A";!received msg

            ! parse the message received from the client
            ParseMsg msg_received;

            ! clear the received message
            msg_received:="";

        ENDWHILE

        SocketSend client_socket\Str:="Closing Server Socket\0D\0A";
        TPWrite "Closing Server Socket";
        SocketClose temp_socket;

    ENDPROC
    
PROC Initialize()
    currTool := [TRUE,[[0,0,114.2],[1,0,0,0]],[0.3,[8.2,12.5,48.1],[1,0,0,0],0.00022,0.00024,9E-05]]; 
    currWobj := [FALSE,TRUE,"",[[0,0,0],[1,0,0,0]],[[0,0,0],[1,0,0,0]]];
    currSpeed := [100, 50, 0, 0];
    currZone := [FALSE, 0.3, 0.3,0.3,0.03,0.3,0.03]; !z0
	
	!Find the current external axis values so they don't move when we start
	jointsTarget := CJointT();
	externalAxis := jointsTarget.extax;
ENDPROC

    !Setting up Socket connection C#
    PROC ConnectToClient()
        VAR string clientIP;

        SocketCreate temp_socket;
        SocketBind temp_socket,"192.168.125.1",4000;
        SocketListen temp_socket;
        SocketAccept temp_socket,client_socket\ClientAddress:=clientIP;

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
                
                !Set joints
                dataType:=StrMatch(key,1,"gewr");
                IF dataType<StrLen(key) THEN
                    TPWrite "going to set joints";
                    setJoints(val);
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
                    IF StrToVal(val,currZone) THEN
                    ENDIF
                    done:=TRUE;
                    GOTO end;
                ENDIF

                ! SPEED sets a new zone for the robot's movements
                dataType:=StrMatch(key,1,"speed");
                IF dataType<StrLen(key) THEN
                    IF StrToVal(val,currSpeed) THEN
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
                        listen:=FALSE;
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
                        msgSend:=ValToStr(currTargL.trans);
                        SocketSend client_socket\Str:=msgSend+"\0D\0A";
                        done:=TRUE;
                        GOTO end;
                    ENDIF

                    dataType:=StrMatch(val,1,"orient");
                    IF dataType<StrLen(val) THEN
                        msgSend:=ValToStr(currTargL.rot);
                        SocketSend client_socket\Str:=msgSend+"\0D\0A";
                        done:=TRUE;
                        GOTO end;
                    ENDIF

                    dataType:=StrMatch(val,1,"config");
                    IF dataType<StrLen(val) THEN
                        msgSend:=ValToStr(currTargL.robconf);
                        SocketSend client_socket\Str:=msgSend+"\0D\0A";
                        done:=TRUE;
                        GOTO end;
                    ENDIF

                    dataType:=StrMatch(val,1,"extax");
                    IF dataType<StrLen(val) THEN
                        msgSend:=ValToStr(currTargL.extax);
                        SocketSend client_socket\Str:=msgSend+"\0D\0A";
                        done:=TRUE;
                        GOTO end;
                    ENDIF

                    dataType:=StrMatch(val,1,"speed");
                    IF dataType<StrLen(val) THEN
                        msgSend:=ValToStr(currSpeed);
                        TPWrite "speed: "+msgSend;
                        SocketSend client_socket\Str:=msgSend+"\0D\0A";
                        done:=TRUE;
                        GOTO end;
                    ENDIF

                    dataType:=StrMatch(val,1,"zone");
                    IF dataType<StrLen(val) THEN
                        msgSend:=ValToStr(currZone);
                        TPWrite "zone: "+msgSend;
                        SocketSend client_socket\Str:=msgSend+"\0D\0A";
                        done:=TRUE;
                        GOTO end;
                    ENDIF
                    
                    !RENE EDIT      #######################################################################################
                    !send cartesian coordinates
                    dataType:=StrMatch(val,1,"gcart");
                    IF dataType<StrLen(val) THEN
                        cartesianPose := CRobT(\Tool:=currTool \WObj:=currWobj);		!linkt de pose
                        addString := NumToStr(cartesianPose.trans.x,1) + ";";
                        addString := addString + NumToStr(cartesianPose.trans.y,1) + ";";
                        addString := addString + NumToStr(cartesianPose.trans.z,1) + ";";
                        addString := addString + NumToStr(cartesianPose.rot.q1,2) + ";";
                        addString := addString + NumToStr(cartesianPose.rot.q2,2) + ";";
                        addString := addString + NumToStr(cartesianPose.rot.q3,2) + ";";
                        addString := addString + NumToStr(cartesianPose.rot.q4,2) + ";";
                        
                        addString := addString + NumToStr(cartesianPose.robconf.cf1,0) + ";";
                        addString := addString + NumToStr(cartesianPose.robconf.cf4,0) + ";";
                        addString := addString + NumToStr(cartesianPose.robconf.cf6,0) + ";";
                        addString := addString + NumToStr(cartesianPose.robconf.cfx,0) + ";";
                        
                        addString := addString + NumToStr(cartesianPose.extax.eax_a,1); !einde van alle variabelen
                        
                        
                        
                        SocketSend client_socket\Str:=addString;
                        
                        GOTO end;
                    ENDIF
                    
                    !send position of the joints
                    dataType:=StrMatch(val,1,"gjoints");
                    IF dataType<StrLen(val) THEN
                        jointsTarget := CJointT();
                        addString := NumToStr(jointsTarget.robax.rax_1,2) + ";";
                        addString := addString + NumToStr(jointsTarget.robax.rax_2,2) + ";";
                        addString := addString + NumToStr(jointsTarget.robax.rax_3,2) + ";";
                        addString := addString + NumToStr(jointsTarget.robax.rax_4,2) + ";";
                        addString := addString + NumToStr(jointsTarget.robax.rax_5,2) + ";";
                        addString := addString + NumToStr(jointsTarget.robax.rax_6,2) + ";";
                        addString := addString + NumToStr(jointsTarget.extax.eax_a,2);!End of string
                        SocketSend client_socket\Str:=addString;
                        
                        GOTO end;
                    ENDIF
                    
                    dataType:=StrMatch(val,1,"ping");
                    IF dataType<StrLen(val) THEN
                        addString := " ";
                        SocketSend client_socket\Str:=addString;
                        
                        GOTO end;
                    ENDIF

                    !END RENE EDIT #######################################################################################

                ENDIF

                end:

                ! if we haven't found an appropriate command, send a message to the computer
                ! that they're sending the wrong key
                IF done=FALSE THEN
                    TPWrite "key ["+key+"] could not be found";
                    !SocketSend client_socket\Str:="key ["+key+"] could not be found"+"\0D\0A";
                    done:=TRUE;
                ENDIF

            ENDWHILE

        ENDIF

    ENDPROC

    
    
    !Functions
    PROC moveRelTool(String val)
        VAR num coords{6};

        IF StrToVal(val,coords) THEN
            MoveJ RelTool(currTargL,coords{1},coords{2},coords{3}\Rx:=coords{4}\Ry:=coords{5}\Rz:=coords{6}),currSpeed,currZone,currTool;
            currPosL:=CPos();
            currTargL:=CRobT();
        ELSE
            TPWrite "Improper msg format: "+val;
        ENDIF
    ENDPROC 
    
    
    !#######################################################################################################
    PROC setCartesian(String val)
        VAR num coords{12};

        IF StrToVal(val,coords) THEN
            currTargL :=[[coords{1},coords{2},coords{3}],
                            [coords{4},coords{5},coords{6},coords{7}],
                            [coords{8},coords{9},coords{10},coords{11}],
                            [coords{12},9E9,9E9,9E9,9E9,9E9]];
            MoveL currTargL, currSpeed, currZone, currTool \WObj:=currWobj ;
        ELSE
            TPWrite "Improper msg format: "+val;
        ENDIF
                    
    ENDPROC
    
    PROC setProper(String val)
        VAR num coords{8};

        IF StrToVal(val,coords) THEN
            currTargL.trans :=[coords{1},coords{2},coords{3}];
            currTargL.rot := [coords{4},coords{5},coords{6},coords{7}];
            currTargL.extax := [coords{12},9E9,9E9,9E9,9E9,9E9];
            MoveL currTargL, currSpeed, currZone, currTool \WObj:=currWobj ;
        ELSE
            TPWrite "Improper msg format: "+val;
        ENDIF
                    
    ENDPROC
    
    PROC setJoints(String val)
        VAR num coords{7};
        
        
        IF StrToVal(val,coords) THEN
            TPWrite "Set Joints";
            jointsTarget.robax:= [coords{1},coords{2},coords{3},coords{4},coords{5},coords{6}];
            jointsTarget.extax:= [coords{7},9E9,9E9,9E9,9E9,9E9];
            MoveAbsJ jointsTarget, currSpeed, currZone, currTool \Wobj:=currWobj;
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
            testTarget:=CalcRobT(testJoint,currTool);
            target.robconf:=testTarget.robconf;

            MoveJ target,currSpeed,currZone,currTool;

            currPosL:=CPos();
            currTargL:=CRobT();
        ELSE
            TPWrite "Improper msg format: "+val;
        ENDIF
    ENDPROC

     PROC updatePosition(String val)
        VAR pos position;

        IF StrToVal(val,position) THEN
            currTargL.trans:=position;
            MoveJ currTargL,currSpeed,currZone,currTool;
            currPosL:=CPos();
            currTargL:=CRobT();
            SocketSend client_socket\Str:="Done";
        ELSE
            TPWrite "Improper msg format: "+val;
        ENDIF
    ENDPROC
    
     PROC updateOrientation(String val)
        VAR num rots{3};

        IF StrToVal(val,rots) THEN        
            currTargL.rot:=OrientZYX(rots{3},rots{2},rots{1});
            MoveL currTargL,currSpeed,currZone,currTool;
            currPosL:=CPos();
            currTargL:=CRobT();
        ELSE
            TPWrite "Improper msg format: "+val;
        ENDIF
    ENDPROC
    
    PROC updateConfig(String val)
        VAR confdata config;

        IF StrToVal(val,config) THEN
            currTargL.robconf:=config;
            MoveJ currTargL,currSpeed,currZone,currTool;
            currPosL:=CPos();
            currTargL:=CRobT();
        ELSE
            TPWrite "Improper msg format: "+val;
        ENDIF
    ENDPROC

    PROC updateExternalAxes(String val)
        VAR extjoint extax;

        IF StrToVal(val,extax) THEN
            TPWrite "Moving external axis:";
            currTargL.extax:=extax;
            MoveL currTargL,currSpeed,currZone,currTool;
            currPosL:=CPos();
            currTargL:=CRobT();
        ELSE
            TPWrite "Improper msg format: "+val;
        ENDIF
    ENDPROC

ENDMODULE
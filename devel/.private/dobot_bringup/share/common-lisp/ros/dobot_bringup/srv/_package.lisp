(cl:defpackage dobot_bringup-srv
  (:use )
  (:export
   "AI"
   "<AI-REQUEST>"
   "AI-REQUEST"
   "<AI-RESPONSE>"
   "AI-RESPONSE"
   "AO"
   "<AO-REQUEST>"
   "AO-REQUEST"
   "<AO-RESPONSE>"
   "AO-RESPONSE"
   "AOEXECUTE"
   "<AOEXECUTE-REQUEST>"
   "AOEXECUTE-REQUEST"
   "<AOEXECUTE-RESPONSE>"
   "AOEXECUTE-RESPONSE"
   "ACCJ"
   "<ACCJ-REQUEST>"
   "ACCJ-REQUEST"
   "<ACCJ-RESPONSE>"
   "ACCJ-RESPONSE"
   "ACCL"
   "<ACCL-REQUEST>"
   "ACCL-REQUEST"
   "<ACCL-RESPONSE>"
   "ACCL-RESPONSE"
   "ARC"
   "<ARC-REQUEST>"
   "ARC-REQUEST"
   "<ARC-RESPONSE>"
   "ARC-RESPONSE"
   "ARCH"
   "<ARCH-REQUEST>"
   "ARCH-REQUEST"
   "<ARCH-RESPONSE>"
   "ARCH-RESPONSE"
   "BRAKECONTROL"
   "<BRAKECONTROL-REQUEST>"
   "BRAKECONTROL-REQUEST"
   "<BRAKECONTROL-RESPONSE>"
   "BRAKECONTROL-RESPONSE"
   "CP"
   "<CP-REQUEST>"
   "CP-REQUEST"
   "<CP-RESPONSE>"
   "CP-RESPONSE"
   "CIRCLE"
   "<CIRCLE-REQUEST>"
   "CIRCLE-REQUEST"
   "<CIRCLE-RESPONSE>"
   "CIRCLE-RESPONSE"
   "CLEARERROR"
   "<CLEARERROR-REQUEST>"
   "CLEARERROR-REQUEST"
   "<CLEARERROR-RESPONSE>"
   "CLEARERROR-RESPONSE"
   "CONTINUESCRIPT"
   "<CONTINUESCRIPT-REQUEST>"
   "CONTINUESCRIPT-REQUEST"
   "<CONTINUESCRIPT-RESPONSE>"
   "CONTINUESCRIPT-RESPONSE"
   "CONTINUES"
   "<CONTINUES-REQUEST>"
   "CONTINUES-REQUEST"
   "<CONTINUES-RESPONSE>"
   "CONTINUES-RESPONSE"
   "DI"
   "<DI-REQUEST>"
   "DI-REQUEST"
   "<DI-RESPONSE>"
   "DI-RESPONSE"
   "DIGROUP"
   "<DIGROUP-REQUEST>"
   "DIGROUP-REQUEST"
   "<DIGROUP-RESPONSE>"
   "DIGROUP-RESPONSE"
   "DO"
   "<DO-REQUEST>"
   "DO-REQUEST"
   "<DO-RESPONSE>"
   "DO-RESPONSE"
   "DOEXECUTE"
   "<DOEXECUTE-REQUEST>"
   "DOEXECUTE-REQUEST"
   "<DOEXECUTE-RESPONSE>"
   "DOEXECUTE-RESPONSE"
   "DOGROUP"
   "<DOGROUP-REQUEST>"
   "DOGROUP-REQUEST"
   "<DOGROUP-RESPONSE>"
   "DOGROUP-RESPONSE"
   "DIGITALOUTPUTS"
   "<DIGITALOUTPUTS-REQUEST>"
   "DIGITALOUTPUTS-REQUEST"
   "<DIGITALOUTPUTS-RESPONSE>"
   "DIGITALOUTPUTS-RESPONSE"
   "DISABLEROBOT"
   "<DISABLEROBOT-REQUEST>"
   "DISABLEROBOT-REQUEST"
   "<DISABLEROBOT-RESPONSE>"
   "DISABLEROBOT-RESPONSE"
   "EMERGENCYSTOP"
   "<EMERGENCYSTOP-REQUEST>"
   "EMERGENCYSTOP-REQUEST"
   "<EMERGENCYSTOP-RESPONSE>"
   "EMERGENCYSTOP-RESPONSE"
   "ENABLEROBOT"
   "<ENABLEROBOT-REQUEST>"
   "ENABLEROBOT-REQUEST"
   "<ENABLEROBOT-RESPONSE>"
   "ENABLEROBOT-RESPONSE"
   "GETANGLE"
   "<GETANGLE-REQUEST>"
   "GETANGLE-REQUEST"
   "<GETANGLE-RESPONSE>"
   "GETANGLE-RESPONSE"
   "GETCOILS"
   "<GETCOILS-REQUEST>"
   "GETCOILS-REQUEST"
   "<GETCOILS-RESPONSE>"
   "GETCOILS-RESPONSE"
   "GETERRORID"
   "<GETERRORID-REQUEST>"
   "GETERRORID-REQUEST"
   "<GETERRORID-RESPONSE>"
   "GETERRORID-RESPONSE"
   "GETHOLDREGS"
   "<GETHOLDREGS-REQUEST>"
   "GETHOLDREGS-REQUEST"
   "<GETHOLDREGS-RESPONSE>"
   "GETHOLDREGS-RESPONSE"
   "GETINBITS"
   "<GETINBITS-REQUEST>"
   "GETINBITS-REQUEST"
   "<GETINBITS-RESPONSE>"
   "GETINBITS-RESPONSE"
   "GETINREGS"
   "<GETINREGS-REQUEST>"
   "GETINREGS-REQUEST"
   "<GETINREGS-RESPONSE>"
   "GETINREGS-RESPONSE"
   "GETPATHSTARTPOSE"
   "<GETPATHSTARTPOSE-REQUEST>"
   "GETPATHSTARTPOSE-REQUEST"
   "<GETPATHSTARTPOSE-RESPONSE>"
   "GETPATHSTARTPOSE-RESPONSE"
   "GETPOSE"
   "<GETPOSE-REQUEST>"
   "GETPOSE-REQUEST"
   "<GETPOSE-RESPONSE>"
   "GETPOSE-RESPONSE"
   "GETSIXFORCEDATA"
   "<GETSIXFORCEDATA-REQUEST>"
   "GETSIXFORCEDATA-REQUEST"
   "<GETSIXFORCEDATA-RESPONSE>"
   "GETSIXFORCEDATA-RESPONSE"
   "GETTERMINAL485"
   "<GETTERMINAL485-REQUEST>"
   "GETTERMINAL485-REQUEST"
   "<GETTERMINAL485-RESPONSE>"
   "GETTERMINAL485-RESPONSE"
   "GETTRACESTARTPOSE"
   "<GETTRACESTARTPOSE-REQUEST>"
   "GETTRACESTARTPOSE-REQUEST"
   "<GETTRACESTARTPOSE-RESPONSE>"
   "GETTRACESTARTPOSE-RESPONSE"
   "HANDLETRAJPOINTS"
   "<HANDLETRAJPOINTS-REQUEST>"
   "HANDLETRAJPOINTS-REQUEST"
   "<HANDLETRAJPOINTS-RESPONSE>"
   "HANDLETRAJPOINTS-RESPONSE"
   "INVERSESOLUTION"
   "<INVERSESOLUTION-REQUEST>"
   "INVERSESOLUTION-REQUEST"
   "<INVERSESOLUTION-RESPONSE>"
   "INVERSESOLUTION-RESPONSE"
   "JOINTMOVJ"
   "<JOINTMOVJ-REQUEST>"
   "JOINTMOVJ-REQUEST"
   "<JOINTMOVJ-RESPONSE>"
   "JOINTMOVJ-RESPONSE"
   "JUMP"
   "<JUMP-REQUEST>"
   "JUMP-REQUEST"
   "<JUMP-RESPONSE>"
   "JUMP-RESPONSE"
   "LIMZ"
   "<LIMZ-REQUEST>"
   "LIMZ-REQUEST"
   "<LIMZ-RESPONSE>"
   "LIMZ-RESPONSE"
   "LOADSWITCH"
   "<LOADSWITCH-REQUEST>"
   "LOADSWITCH-REQUEST"
   "<LOADSWITCH-RESPONSE>"
   "LOADSWITCH-RESPONSE"
   "MODBUSCLOSE"
   "<MODBUSCLOSE-REQUEST>"
   "MODBUSCLOSE-REQUEST"
   "<MODBUSCLOSE-RESPONSE>"
   "MODBUSCLOSE-RESPONSE"
   "MODBUSCREATE"
   "<MODBUSCREATE-REQUEST>"
   "MODBUSCREATE-REQUEST"
   "<MODBUSCREATE-RESPONSE>"
   "MODBUSCREATE-RESPONSE"
   "MOVJ"
   "<MOVJ-REQUEST>"
   "MOVJ-REQUEST"
   "<MOVJ-RESPONSE>"
   "MOVJ-RESPONSE"
   "MOVJEXT"
   "<MOVJEXT-REQUEST>"
   "MOVJEXT-REQUEST"
   "<MOVJEXT-RESPONSE>"
   "MOVJEXT-RESPONSE"
   "MOVJIO"
   "<MOVJIO-REQUEST>"
   "MOVJIO-REQUEST"
   "<MOVJIO-RESPONSE>"
   "MOVJIO-RESPONSE"
   "MOVL"
   "<MOVL-REQUEST>"
   "MOVL-REQUEST"
   "<MOVL-RESPONSE>"
   "MOVL-RESPONSE"
   "MOVLIO"
   "<MOVLIO-REQUEST>"
   "MOVLIO-REQUEST"
   "<MOVLIO-RESPONSE>"
   "MOVLIO-RESPONSE"
   "MOVEJOG"
   "<MOVEJOG-REQUEST>"
   "MOVEJOG-REQUEST"
   "<MOVEJOG-RESPONSE>"
   "MOVEJOG-RESPONSE"
   "PAUSESCRIPT"
   "<PAUSESCRIPT-REQUEST>"
   "PAUSESCRIPT-REQUEST"
   "<PAUSESCRIPT-RESPONSE>"
   "PAUSESCRIPT-RESPONSE"
   "PAYLOAD"
   "<PAYLOAD-REQUEST>"
   "PAYLOAD-REQUEST"
   "<PAYLOAD-RESPONSE>"
   "PAYLOAD-RESPONSE"
   "POSITIVESOLUTION"
   "<POSITIVESOLUTION-REQUEST>"
   "POSITIVESOLUTION-REQUEST"
   "<POSITIVESOLUTION-RESPONSE>"
   "POSITIVESOLUTION-RESPONSE"
   "POWERON"
   "<POWERON-REQUEST>"
   "POWERON-REQUEST"
   "<POWERON-RESPONSE>"
   "POWERON-RESPONSE"
   "RELJOINTMOVJ"
   "<RELJOINTMOVJ-REQUEST>"
   "RELJOINTMOVJ-REQUEST"
   "<RELJOINTMOVJ-RESPONSE>"
   "RELJOINTMOVJ-RESPONSE"
   "RELMOVJ"
   "<RELMOVJ-REQUEST>"
   "RELMOVJ-REQUEST"
   "<RELMOVJ-RESPONSE>"
   "RELMOVJ-RESPONSE"
   "RELMOVJTOOL"
   "<RELMOVJTOOL-REQUEST>"
   "RELMOVJTOOL-REQUEST"
   "<RELMOVJTOOL-RESPONSE>"
   "RELMOVJTOOL-RESPONSE"
   "RELMOVJUSER"
   "<RELMOVJUSER-REQUEST>"
   "RELMOVJUSER-REQUEST"
   "<RELMOVJUSER-RESPONSE>"
   "RELMOVJUSER-RESPONSE"
   "RELMOVL"
   "<RELMOVL-REQUEST>"
   "RELMOVL-REQUEST"
   "<RELMOVL-RESPONSE>"
   "RELMOVL-RESPONSE"
   "RELMOVLTOOL"
   "<RELMOVLTOOL-REQUEST>"
   "RELMOVLTOOL-REQUEST"
   "<RELMOVLTOOL-RESPONSE>"
   "RELMOVLTOOL-RESPONSE"
   "RELMOVLUSER"
   "<RELMOVLUSER-REQUEST>"
   "RELMOVLUSER-REQUEST"
   "<RELMOVLUSER-RESPONSE>"
   "RELMOVLUSER-RESPONSE"
   "RESETROBOT"
   "<RESETROBOT-REQUEST>"
   "RESETROBOT-REQUEST"
   "<RESETROBOT-RESPONSE>"
   "RESETROBOT-RESPONSE"
   "ROBOTMODE"
   "<ROBOTMODE-REQUEST>"
   "ROBOTMODE-REQUEST"
   "<ROBOTMODE-RESPONSE>"
   "ROBOTMODE-RESPONSE"
   "RUNSCRIPT"
   "<RUNSCRIPT-REQUEST>"
   "RUNSCRIPT-REQUEST"
   "<RUNSCRIPT-RESPONSE>"
   "RUNSCRIPT-RESPONSE"
   "SERVOJ"
   "<SERVOJ-REQUEST>"
   "SERVOJ-REQUEST"
   "<SERVOJ-RESPONSE>"
   "SERVOJ-RESPONSE"
   "SERVOP"
   "<SERVOP-REQUEST>"
   "SERVOP-REQUEST"
   "<SERVOP-RESPONSE>"
   "SERVOP-RESPONSE"
   "SETARMORIENTATION"
   "<SETARMORIENTATION-REQUEST>"
   "SETARMORIENTATION-REQUEST"
   "<SETARMORIENTATION-RESPONSE>"
   "SETARMORIENTATION-RESPONSE"
   "SETCOILS"
   "<SETCOILS-REQUEST>"
   "SETCOILS-REQUEST"
   "<SETCOILS-RESPONSE>"
   "SETCOILS-RESPONSE"
   "SETCOLLIDEDRAG"
   "<SETCOLLIDEDRAG-REQUEST>"
   "SETCOLLIDEDRAG-REQUEST"
   "<SETCOLLIDEDRAG-RESPONSE>"
   "SETCOLLIDEDRAG-RESPONSE"
   "SETCOLLISIONLEVEL"
   "<SETCOLLISIONLEVEL-REQUEST>"
   "SETCOLLISIONLEVEL-REQUEST"
   "<SETCOLLISIONLEVEL-RESPONSE>"
   "SETCOLLISIONLEVEL-RESPONSE"
   "SETHOLDREGS"
   "<SETHOLDREGS-REQUEST>"
   "SETHOLDREGS-REQUEST"
   "<SETHOLDREGS-RESPONSE>"
   "SETHOLDREGS-RESPONSE"
   "SETOBSTACLEAVOID"
   "<SETOBSTACLEAVOID-REQUEST>"
   "SETOBSTACLEAVOID-REQUEST"
   "<SETOBSTACLEAVOID-RESPONSE>"
   "SETOBSTACLEAVOID-RESPONSE"
   "SETPAYLOAD"
   "<SETPAYLOAD-REQUEST>"
   "SETPAYLOAD-REQUEST"
   "<SETPAYLOAD-RESPONSE>"
   "SETPAYLOAD-RESPONSE"
   "SETSAFESKIN"
   "<SETSAFESKIN-REQUEST>"
   "SETSAFESKIN-REQUEST"
   "<SETSAFESKIN-RESPONSE>"
   "SETSAFESKIN-RESPONSE"
   "SETTERMINAL485"
   "<SETTERMINAL485-REQUEST>"
   "SETTERMINAL485-REQUEST"
   "<SETTERMINAL485-RESPONSE>"
   "SETTERMINAL485-RESPONSE"
   "SETTERMINALKEYS"
   "<SETTERMINALKEYS-REQUEST>"
   "SETTERMINALKEYS-REQUEST"
   "<SETTERMINALKEYS-RESPONSE>"
   "SETTERMINALKEYS-RESPONSE"
   "SPEEDFACTOR"
   "<SPEEDFACTOR-REQUEST>"
   "SPEEDFACTOR-REQUEST"
   "<SPEEDFACTOR-RESPONSE>"
   "SPEEDFACTOR-RESPONSE"
   "SPEEDJ"
   "<SPEEDJ-REQUEST>"
   "SPEEDJ-REQUEST"
   "<SPEEDJ-RESPONSE>"
   "SPEEDJ-RESPONSE"
   "SPEEDL"
   "<SPEEDL-REQUEST>"
   "SPEEDL-REQUEST"
   "<SPEEDL-RESPONSE>"
   "SPEEDL-RESPONSE"
   "STARTDRAG"
   "<STARTDRAG-REQUEST>"
   "STARTDRAG-REQUEST"
   "<STARTDRAG-RESPONSE>"
   "STARTDRAG-RESPONSE"
   "STARTFCTRACE"
   "<STARTFCTRACE-REQUEST>"
   "STARTFCTRACE-REQUEST"
   "<STARTFCTRACE-RESPONSE>"
   "STARTFCTRACE-RESPONSE"
   "STARTPATH"
   "<STARTPATH-REQUEST>"
   "STARTPATH-REQUEST"
   "<STARTPATH-RESPONSE>"
   "STARTPATH-RESPONSE"
   "STARTTRACE"
   "<STARTTRACE-REQUEST>"
   "STARTTRACE-REQUEST"
   "<STARTTRACE-RESPONSE>"
   "STARTTRACE-RESPONSE"
   "STOPDRAG"
   "<STOPDRAG-REQUEST>"
   "STOPDRAG-REQUEST"
   "<STOPDRAG-RESPONSE>"
   "STOPDRAG-RESPONSE"
   "STOPSCRIPT"
   "<STOPSCRIPT-REQUEST>"
   "STOPSCRIPT-REQUEST"
   "<STOPSCRIPT-RESPONSE>"
   "STOPSCRIPT-RESPONSE"
   "STOPMOVEJOG"
   "<STOPMOVEJOG-REQUEST>"
   "STOPMOVEJOG-REQUEST"
   "<STOPMOVEJOG-RESPONSE>"
   "STOPMOVEJOG-RESPONSE"
   "SYNC"
   "<SYNC-REQUEST>"
   "SYNC-REQUEST"
   "<SYNC-RESPONSE>"
   "SYNC-RESPONSE"
   "SYNCALL"
   "<SYNCALL-REQUEST>"
   "SYNCALL-REQUEST"
   "<SYNCALL-RESPONSE>"
   "SYNCALL-RESPONSE"
   "TCPSPEED"
   "<TCPSPEED-REQUEST>"
   "TCPSPEED-REQUEST"
   "<TCPSPEED-RESPONSE>"
   "TCPSPEED-RESPONSE"
   "TCPSPEEDEND"
   "<TCPSPEEDEND-REQUEST>"
   "TCPSPEEDEND-REQUEST"
   "<TCPSPEEDEND-RESPONSE>"
   "TCPSPEEDEND-RESPONSE"
   "TCPDASHBOARD"
   "<TCPDASHBOARD-REQUEST>"
   "TCPDASHBOARD-REQUEST"
   "<TCPDASHBOARD-RESPONSE>"
   "TCPDASHBOARD-RESPONSE"
   "TCPREALDATA"
   "<TCPREALDATA-REQUEST>"
   "TCPREALDATA-REQUEST"
   "<TCPREALDATA-RESPONSE>"
   "TCPREALDATA-RESPONSE"
   "TOOL"
   "<TOOL-REQUEST>"
   "TOOL-REQUEST"
   "<TOOL-RESPONSE>"
   "TOOL-RESPONSE"
   "TOOLAI"
   "<TOOLAI-REQUEST>"
   "TOOLAI-REQUEST"
   "<TOOLAI-RESPONSE>"
   "TOOLAI-RESPONSE"
   "TOOLDI"
   "<TOOLDI-REQUEST>"
   "TOOLDI-REQUEST"
   "<TOOLDI-RESPONSE>"
   "TOOLDI-RESPONSE"
   "TOOLDO"
   "<TOOLDO-REQUEST>"
   "TOOLDO-REQUEST"
   "<TOOLDO-RESPONSE>"
   "TOOLDO-RESPONSE"
   "TOOLDOEXECUTE"
   "<TOOLDOEXECUTE-REQUEST>"
   "TOOLDOEXECUTE-REQUEST"
   "<TOOLDOEXECUTE-RESPONSE>"
   "TOOLDOEXECUTE-RESPONSE"
   "USER"
   "<USER-REQUEST>"
   "USER-REQUEST"
   "<USER-RESPONSE>"
   "USER-RESPONSE"
   "WAIT"
   "<WAIT-REQUEST>"
   "WAIT-REQUEST"
   "<WAIT-RESPONSE>"
   "WAIT-RESPONSE"
   "PAUSE"
   "<PAUSE-REQUEST>"
   "PAUSE-REQUEST"
   "<PAUSE-RESPONSE>"
   "PAUSE-RESPONSE"
  ))

1 To successfully build the iswarm, please run this first:
iswarm$ ./build.sh

2 The main logic:
run the node crazyswarm_server -> CrazyflieServer constructure function -> server.run()
-> server.runFast() 
(0)-> new groups -> addCrazyflie 
(1)-> m CrazyflieGroup.runSlow()s 
(2)-> what?(see also question 3)

3 Is server's runFast() only executed once? What does it do after "Start n threads"? Please not to concern about this.

4 The hiearchy is: 1 server -- m groups -- m*n crazyflie

5 Where is the part that recieving Vicon message and send out the position to crazyflies?

6 I should define a new flight state and flight mode in the commander.h No

7 What is the useMotionCaptureObjectTracking in the run fast function? -> getGroupCurPos(based on CrazyflieBroadcaster::externalPose state) -> groupcontrol:renew sp_state(nonlinearcontrol:Position+sp=sp_state) -> sendAttSps(sp_states)

8 Positions can be obtained from getGroupCurPos. Obtain the setpoint of each vehicle from a new function instead of aflie_state_traj_cb

9 What is CallbackQueue for? Multi-thread spinning

10 What is Latency in server.runfast?

11 add a new for loop before groupcontrol loop in the group's run fast, to calculate formation control.
Update the PVA setpoint before groupcontrol and delete aflie_state_traj_cb.

Publish positions before the new loop, referring to getGroupCurPos.

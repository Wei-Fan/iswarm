1 To successfully build the iswarm, please run this first:
iswarm$ ./build.sh
2 The main logic:
run the node crazyswarm_server -> CrazyflieServer constructure function -> server.run()
-> server.runFast() 
(0)-> new groups -> addCrazyflie 
(1)-> m CrazyflieGroup.runSlow()s 
(2)-> what?(see also question 3)

3 Is server's runFast() only executed once? What does it do after "Start n threads"?

4 The hiearchy is: 1 server -- m groups -- m*n crazyflie

5 Where is the part that recieving Vicon message and send out the position to crazyflies? 



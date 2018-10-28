#pragma once

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>

#include <unistd.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>

#include <NatNetLinux/NatNet.h>
#include <NatNetLinux/CommandListener.h>
#include <NatNetLinux/FrameGetter.h>
#include <NatNetLinux/FPSCounter.h>

#include <boost/program_options.hpp>
#include <time.h>

#define USE_FPS
#define PRINT_INFO

class NatNetClient{
public:
    NatNetClient(){
        bConnected = false;
        localAddress=0;
        serverAddress=0;
        stringVersion="";
	bGotTimeOut=false;
        lastFrameResult = FrameGetter::UNKNOWN;
    }
    ~NatNetClient(){
        disconnect();
    }
    
    void connect(std::string localIp, std::string serverIp){
        std::cout<<"NatNetClient connecting: "<<localIp<<" to "<<serverIp<<"..."<<std::endl;
        localAddress = stringToIp(localIp);
        serverAddress= stringToIp(serverIp);
        
        serverCommands = NatNet::createAddress(serverAddress, NatNet::commandPort);
        
        sdCommands = NatNet::createCommandSocket(localAddress);
        sdData = NatNet::createDataSocket(localAddress);
        
        commandListener.setSocket(sdCommands);
        commandListener.start();
        
        ping = NatNetPacket::pingPacket();
        ping.send(sdCommands, serverCommands);
        
        commandListener.getNatNetVersion(natNetMajor, natNetMinor);
        updateVersionString();
        std::cout<<"NatNet server version: "<<serverAddress<<std::endl;
        
        frameGetter.set(sdData, natNetMajor, natNetMinor);
    }
    
    void disconnect(){
        std::cout<<"NatNetClient is closing"<<std::endl;
        commandListener.stop();
        commandListener.join();
        
        close(sdData);
        close(sdCommands);
        
        bConnected=false;
    }
    
    void update(){
        lastFrameResult = frameGetter.nextFrame();
        if(lastFrameResult==FrameGetter::SUCCESS){
            lastFrame = frameGetter.getLastFrame();
#ifdef USE_FPS
            fps.update(frameGetter.getLastTimeStamp());
#ifdef PRINT_INFO
            if(bGotTimeOut){
            	std::cout<<"NatNet reconnected after "<<fps.getLatency()<<" sec"<<std::endl;
		bGotTimeOut=false;
            }

#endif
#endif
        }
#ifdef PRINT_INFO
        else{
            if(lastFrameResult == FrameGetter::TIMEOUT){
               std::cout<<"NATNET FRAME TIMEOUT"<<std::endl;
		bGotTimeOut=true;
            }else{
               std::cout<<"NATNET FRAME LOST: bad data"<<std::endl;
            }
        }
#endif
    }
    
    bool isNewFrameReady(){
	return (lastFrameResult==FrameGetter::SUCCESS);
    }
    
    bool isConnected(){return bConnected;}
    
    MocapFrame & getLastFrame(){
        return lastFrame;
    }
    
    uint32_t stringToIp(std::string s){
        return inet_addr(s.c_str());
    }
    
    void updateVersionString(){
        std::stringstream sstr;
        sstr<<natNetMajor<<"."<<natNetMinor;
        stringVersion = sstr.str();
    }
    
    std::string getVersionString(){
        return stringVersion;
    }
    
#ifdef USE_FPS
    double getFps(){return fps.getFps();}
    double getCustomLatency(){return fps.getLatency();}
#endif
private:
#ifdef USE_FPS
    FPSCounter fps;
    bool bGotTimeOut;
#endif
    CommandListener commandListener;
    NatNetPacket ping;
    FrameGetter frameGetter;
    FrameGetter::FrameResult lastFrameResult;
    MocapFrame lastFrame;
    
    struct sockaddr_in serverCommands;
    //sockets
    int sdCommands;
    int sdData;
    
    uint32_t localAddress;
    uint32_t serverAddress;
    
    unsigned char natNetMajor;
    unsigned char natNetMinor;
    std::string stringVersion;
    bool bConnected;
};

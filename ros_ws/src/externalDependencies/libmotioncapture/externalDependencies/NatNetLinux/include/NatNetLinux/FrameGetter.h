/*
 * FrameGetter.h 
 *
 * NatNetLinux is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * NatNetLinux is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with NatNetLinux.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef FRAMEGETTER_H
#define FRAMEGETTER_H

#include <NatNetLinux/NatNet.h>
#include <NatNetLinux/NatNetPacket.h>
#include <NatNetLinux/NatNetSender.h>
#include <utility>
#include <time.h>

#ifdef __MACH__
#include <mach/clock.h>
#include <mach/mach.h>
#endif

class FrameGetter{
public:
    enum FrameResult{
        SUCCESS,
        TIMEOUT,
        UNKNOWN
    };
    
    FrameGetter(){
        bNewFrame=false;
        timeOutMillis=10;
    }
    
    FrameGetter(int sd, unsigned char nnMajor, unsigned char nnMinor, size_t bufferSize=64){
        FrameGetter();
        timeOutMillis=10;
        set(sd, nnMajor, nnMinor, bufferSize);
    }
    
    /*FrameGetter(int sd = -1, unsigned char nnMajor=0, unsigned char nnMinor=0, size_t bufferSize=64) :
    _sd(sd),
    _nnMajor(nnMajor),
    _nnMinor(nnMinor),
    toSec(1),
    bNewFrame(false),
    lastFrame(_nnMajor, _nnMinor)
    {
        
    }*/
    
    ~FrameGetter(){}
    
    
    void set(int sd = -1, unsigned char nnMajor=0, unsigned char nnMinor=0, size_t bufferSize=64){
        _sd = sd;
        _nnMajor = nnMajor;
        _nnMinor = nnMinor;
        lastFrame = MocapFrame(_nnMajor, _nnMinor);
    }
    
    bool isNewFrameReady(){return bNewFrame;}
    
    void updateTimeStamp(){
#ifdef __MACH__
        clock_serv_t cclock;
        mach_timespec_t mts;
        host_get_clock_service(mach_host_self(), CALENDAR_CLOCK, &cclock);
        clock_get_time(cclock, &mts);
        mach_port_deallocate(mach_task_self(), cclock);
        lastTs.tv_sec = mts.tv_sec;
        lastTs.tv_nsec = mts.tv_nsec;
#else
        clock_gettime( CLOCK_MONOTONIC, &lastTs );
#endif
    }
    
    FrameResult nextFrame(){
        FrameResult fr = FrameGetter::UNKNOWN;
        
        timeout.tv_sec =0;
        timeout.tv_usec=timeOutMillis*1000;
        
        FD_ZERO(&rfds);
        FD_SET(_sd, &rfds);
        
        if(!select(_sd+1, &rfds, 0, 0, &timeout)){
            fr = FrameGetter::TIMEOUT;
            return fr;
        }
        
        dataBytes = read(_sd, nnp.rawPtr(), nnp.maxLength());
        
        if(dataBytes>0 && nnp.iMessage()==NatNetPacket::NAT_FRAMEOFDATA){
            MocapFrame mFrame(_nnMajor, _nnMinor);
            mFrame.unpack(nnp.rawPayloadPtr());
            
            updateTimeStamp();
            lastFrame = mFrame;
            bNewFrame = true;
            fr = FrameGetter::SUCCESS;
        }
        
        return fr;
    }
    
    MocapFrame getLastFrame(){
        bNewFrame = false;
        return lastFrame;
    }
    
    struct timespec getLastTimeStamp(){
        return lastTs;
    }
    
    std::pair<MocapFrame, struct timespec> getLastFrameInfo(){
        return std::make_pair(getLastFrame(), getLastTimeStamp());
    }
    
private:
    
    MocapFrame lastFrame; //last MocapFrame
    NatNetPacket nnp;
    struct timespec lastTs; //last timestamp
    struct timeval timeout;
    fd_set rfds;
    size_t dataBytes;
    int _sd;
    int timeOutMillis;
    unsigned char _nnMajor;
    unsigned char _nnMinor;
    bool bNewFrame;
};

#endif /*FRAMEGETTER_H*/

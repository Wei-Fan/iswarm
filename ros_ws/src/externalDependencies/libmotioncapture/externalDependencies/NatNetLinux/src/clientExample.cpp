#include "NatNetLinux/NatNetClient.h"

class Globals{
public:
    static std::string localAddress;
    static std::string serverAddress;
    static bool run;
};

std::string Globals::localAddress="";
std::string Globals::serverAddress="";
bool Globals::run = false;

void terminate(int){
    Globals::run = false;
}

void readOpts(int argc, char * argv[]){
    namespace po = boost::program_options;
    
    po::options_description desc("clientExample:");
    desc.add_options()
    ("help", "Display help message")
    ("local-addr,l", po::value<std::string>(), "Local IPv4 address")
    ("server-addr,s", po::value<std::string>(), "Server IPv4 address")
    ;
    
    po::variables_map vm;
    po::store(po::parse_command_line(argc,argv,desc), vm);
    
    if(
       argc < 5 || vm.count("help") ||
       !vm.count("local-addr") ||
       !vm.count("server-addr")
       )
    {
        std::cout << desc << std::endl;
        exit(1);
    }
    
    Globals::localAddress = vm["local-addr"].as<std::string>();
    Globals::serverAddress= vm["server-addr"].as<std::string>();
}

void update(NatNetClient & c){
    Globals::run = true;
    while(Globals::run){
	
        c.update();
        if(c.isNewFrameReady()){
            std::cout<<c.getLastFrame()<<std::endl;
        }
        //FrameGetter::FrameResult fr = c.frameGetter.nextFrame();
        //if(fr==FrameGetter::SUCCESS){
        //    std::cout<<c.frameGetter.getLastFrame()<<std::endl;
        //}
    }
}

int main(int argc, char* argv[]){
    
    signal(SIGINT, terminate);
    readOpts(argc, argv);
    
    NatNetClient client;
    client.connect(Globals::localAddress, Globals::serverAddress);
    
    update(client);
    
    return 0;
}

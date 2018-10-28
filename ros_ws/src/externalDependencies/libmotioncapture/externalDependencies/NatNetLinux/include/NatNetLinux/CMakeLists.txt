SET( H_FILES
   "CommandListener.h"
   "FrameListener.h"
   "FrameGetter.h"
   "FPSCounter.h"
   "NatNet.h"
   "NatNetPacket.h"
   "NatNetSender.h"
   "NatNetClient.h"
)

INSTALL(
   FILES ${H_FILES}
   DESTINATION include/NatNetLinux
)

#ifndef sunSeek
#define sunSeek

#include <SqrtFit.h>
#include <LightVectorDetermination.h>

//calibrated LVD instance to read angles 
extern LightVectorDetermination LVD;
extern double angle;

void getSunAngle(){
  //calculate desired angle change with photodiodes
  angle = LVD.get_global_angle(); 

}
#endif //sunSeek
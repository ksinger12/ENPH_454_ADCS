To interface with the sunsensors you just need to copy the two folders in the `libraries` folder 
into your local `Arduino/libraries/` folder. Then any project that has 
`#include <LightVectorDetermination.h>` at the top of the script will be able to get light data.
To instantiate a `LightVectorDetermination` object, you need to pass:
```
int : Number of photodiodes in the array (i.e. 4)
int : Number of readings done during calibration
int : Pin offset for photodiode array (it assumes sequential pin ordering)
int : Number of raw photoidode readings to average over
```

The example Arduino sript shows how to calibrate the sunsensors with a stepper moder and demonstrates 
a basic control algorithm to track the sunsource.

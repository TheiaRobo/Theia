FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/differential_drive/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "../msg_gen/cpp/include/differential_drive/Odometry.h"
  "../msg_gen/cpp/include/differential_drive/Sharp.h"
  "../msg_gen/cpp/include/differential_drive/AnalogC.h"
  "../msg_gen/cpp/include/differential_drive/Speed.h"
  "../msg_gen/cpp/include/differential_drive/Encoders.h"
  "../msg_gen/cpp/include/differential_drive/KeyEvent.h"
  "../msg_gen/cpp/include/differential_drive/Params.h"
  "../msg_gen/cpp/include/differential_drive/Servomotors.h"
  "../msg_gen/cpp/include/differential_drive/MouseEvent.h"
  "../msg_gen/cpp/include/differential_drive/PWM.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)

FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/differential_drive/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/differential_drive/msg/__init__.py"
  "../src/differential_drive/msg/_Speed.py"
  "../src/differential_drive/msg/_KeyEvent.py"
  "../src/differential_drive/msg/_MouseEvent.py"
  "../src/differential_drive/msg/_Sharp.py"
  "../src/differential_drive/msg/_AnalogC.py"
  "../src/differential_drive/msg/_Odometry.py"
  "../src/differential_drive/msg/_Encoders.py"
  "../src/differential_drive/msg/_PWM.py"
  "../src/differential_drive/msg/_Params.py"
  "../src/differential_drive/msg/_Servomotors.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)

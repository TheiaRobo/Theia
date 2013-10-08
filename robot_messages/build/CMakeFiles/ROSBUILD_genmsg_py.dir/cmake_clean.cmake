FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/robot_messages/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/robot_messages/msg/__init__.py"
  "../src/robot_messages/msg/_coords.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)

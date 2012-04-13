FILE(REMOVE_RECURSE
  "msg_gen"
  "src/motor/msg"
  "msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "src/motor/msg/__init__.py"
  "src/motor/msg/_MotorControllerStatusStamped.py"
  "src/motor/msg/_MotorControllerStatus.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)

FILE(MAKE_DIRECTORY "/home/rts/ros_workspace/catkin_ws/devel/share/controller/graphs" )
FILE(GLOB_RECURSE FOR_DEL /home/rts/ros_workspace/catkin_ws/devel/share/controller/graphs/* )
#message("delete files from /home/rts/ros_workspace/catkin_ws/devel/share/controller/graphs/ : ${FOR_DEL}")
if( FOR_DEL )
	FILE(REMOVE ${FOR_DEL})
endif()
execute_process(COMMAND /home/rts/ros_workspace/catkin_ws/devel/lib/decision_making_parser/decision_making_parser -pe -xml -dot -src "/home/rts/ros_workspace/catkin_ws/src/controller" -dst "/home/rts/ros_workspace/catkin_ws/devel/share/controller/graphs" -f "/home/rts/ros_workspace/catkin_ws/src/controller/src/Youbot.cpp:" RESULT_VARIABLE rv)
FILE(GLOB_RECURSE CREATED_FILES RELATIVE /home/rts/ros_workspace/catkin_ws/devel/share/controller/graphs/ /home/rts/ros_workspace/catkin_ws/devel/share/controller/graphs/*.scxml /home/rts/ros_workspace/catkin_ws/devel/share/controller/graphs/*.btxml  /home/rts/ros_workspace/catkin_ws/devel/share/controller/graphs/*.taoxml)
message("   -- Created XML files:")
foreach( f ${CREATED_FILES})
     message("      -- ${f} ")
endforeach()
FILE(GLOB_RECURSE CREATED_FILES_ABS /home/rts/ros_workspace/catkin_ws/devel/share/controller/graphs/*.scxml /home/rts/ros_workspace/catkin_ws/devel/share/controller/graphs/*.btxml /home/rts/ros_workspace/catkin_ws/devel/share/controller/graphs/*.taoxml)
execute_process(COMMAND "python" /home/rts/ros_workspace/catkin_ws/devel/lib/decision_making_parser/decision_making_xml_parser.py -i "/home/rts/ros_workspace/catkin_ws/src/controller" "/home/rts/ros_workspace/catkin_ws/devel/share/controller/graphs" "${CREATED_FILES_ABS}" RESULT_VARIABLE rv)
FILE(GLOB_RECURSE CREATED_FILES RELATIVE /home/rts/ros_workspace/catkin_ws/devel/share/controller/graphs/ /home/rts/ros_workspace/catkin_ws/devel/share/controller/graphs/*.dot /home/rts/ros_workspace/catkin_ws/devel/share/controller/graphs/*.xot)
message("   -- Created DOT files:")
foreach( f ${CREATED_FILES})
     message("      -- ${f} ")
endforeach()
FILE(GLOB_RECURSE CREATED_FILES_ABS /home/rts/ros_workspace/catkin_ws/devel/share/controller/graphs/*.dot)
foreach( f ${CREATED_FILES_ABS} )
	execute_process(COMMAND "dot" -q1 -Tgif -o${f}.gif  ${f} RESULT_VARIABLE rv)
endforeach()
FILE(GLOB_RECURSE CREATED_FILES RELATIVE /home/rts/ros_workspace/catkin_ws/devel/share/controller/graphs/ /home/rts/ros_workspace/catkin_ws/devel/share/controller/graphs/*.gif)
message("   -- Created GIF files:")
foreach( f ${CREATED_FILES})
     message("      -- ${f} ")
endforeach()

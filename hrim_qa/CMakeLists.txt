cmake_minimum_required(VERSION 3.5)

project(hrim_qa)
find_package(ament_cmake REQUIRED)

install(
  PROGRAMS
  scripts/publisher_py.py
  scripts/subscriber_py.py
  scripts/requester_py.py
  scripts/replier_py.py
  DESTINATION lib/${PROJECT_NAME}
)

find_package(ros_testing REQUIRED)

MACRO(SUBDIRLIST result curdir)
  FILE(GLOB children RELATIVE ${curdir} ${curdir}/*)
  SET(dirlist "")
  FOREACH(child ${children})
    IF(IS_DIRECTORY ${curdir}/${child})
      LIST(APPEND dirlist ${child})
    ENDIF()
  ENDFOREACH()
  SET(${result} ${dirlist})
ENDMACRO()
##############################################################################
SUBDIRLIST(HRIM_GENERATED_MSGS ${HRIM_DIRECTORY})

MACRO(LAUNCHTEST HRIM_PKG DIRECTORY_TO_LIST result)
  # message("LAUNCHTEST HRIM_PKG " ${HRIM_PKG})

  # message("LAUNCHTEST DIRECTORY_TO_LIST " ${DIRECTORY_TO_LIST})

  set(result "TRUE")
  if(${HRIM_PKG} MATCHES "(msgs)$")
    file(GLOB MSGS ${DIRECTORY_TO_LIST}/msg/*.msg)
    FOREACH(MSG ${MSGS})
      string(REPLACE ${DIRECTORY_TO_LIST}/msg/ "" HRIM_MSG ${MSG} )
      string(REPLACE .msg "" HRIM_MSG ${HRIM_MSG} )
      # message("-> " ${HRIM_PKG} " " ${HRIM_MSG})
      set(HRIM_PACKAGE ${HRIM_PKG})
      configure_file(
        test/test_msgs.py.in
        test/test_msgs_${HRIM_PACKAGE}_${HRIM_MSG}.py.configured
        @ONLY
      )
      file(GENERATE
        OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/test/test_msgs_${HRIM_PKG}_${HRIM_MSG}_configured.py"
        INPUT "${CMAKE_CURRENT_BINARY_DIR}/test/test_msgs_${HRIM_PKG}_${HRIM_MSG}.py.configured"
      )
      add_ros_test(
        "${CMAKE_CURRENT_BINARY_DIR}/test/test_msgs_${HRIM_PKG}_${HRIM_MSG}_configured.py"
        TIMEOUT 60
      )
    ENDFOREACH()
  elseif(${HRIM_PKG} MATCHES "(srvs)$")
    file(GLOB SRVS ${DIRECTORY_TO_LIST}/srv/*.srv)
    FOREACH(SRV ${SRVS})
      string(REPLACE ${DIRECTORY_TO_LIST}/srv/ "" HRIM_MSG ${SRV} )
      string(REPLACE .srv "" HRIM_MSG ${HRIM_MSG} )
      # message("-> " ${HRIM_PKG} " " ${HRIM_MSG})
      set(HRIM_PACKAGE ${HRIM_PKG})
      configure_file(
        test/test_srvs.py.in
        test/test_srvs_${HRIM_PACKAGE}_${HRIM_MSG}.py.configured
        @ONLY
      )
      file(GENERATE
        OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/test/test_srvs_${HRIM_PKG}_${HRIM_MSG}_configured.py"
        INPUT "${CMAKE_CURRENT_BINARY_DIR}/test/test_srvs_${HRIM_PKG}_${HRIM_MSG}.py.configured"
      )
      add_ros_test(
        "${CMAKE_CURRENT_BINARY_DIR}/test/test_srvs_${HRIM_PKG}_${HRIM_MSG}_configured.py"
        TIMEOUT 60
      )
    ENDFOREACH()
  elseif(${HRIM_PKG} MATCHES "(actions)$")
    file(GLOB ACTIONS ${DIRECTORY_TO_LIST}/action/*.action)
    FOREACH(ACTION ${ACTIONS})
      string(REPLACE ${DIRECTORY_TO_LIST}/action/ "" ACTION_REPLACE ${ACTION} )
      string(REPLACE .action "" ACTION_REPLACE ${ACTION_REPLACE} )
      # message("-> " ${HRIM_PKG} " " ${ACTION_REPLACE})
    ENDFOREACH()
  else()
    set(result "FALSE")
  endif()
ENDMACRO()

FOREACH(TYPE_HRIM ${HRIM_GENERATED_MSGS})
  # message("type " /${TYPE_HRIM})
  # message("entering in " ${HRIM_DIRECTORY}/${TYPE_HRIM})
  SUBDIRLIST(HRIM_PKGS ${HRIM_DIRECTORY}/${TYPE_HRIM})
  FOREACH(HRIM_PKG ${HRIM_PKGS})
      LAUNCHTEST(${HRIM_PKG} ${HRIM_DIRECTORY}/${TYPE_HRIM}/${HRIM_PKG} result)
      # message("result " ${result})
      if(NOT ${result})
        # message("Alex3",  ${HRIM_DIRECTORY}/${TYPE_HRIM}/${HRIM_PKG})
        SUBDIRLIST(HRIM_PKGS_2 ${HRIM_DIRECTORY}/${TYPE_HRIM}/${HRIM_PKG})
        FOREACH(HRIM_PKG2 ${HRIM_PKGS_2})
          LAUNCHTEST(${HRIM_PKG2} ${HRIM_DIRECTORY}/${TYPE_HRIM}/${HRIM_PKG}/${HRIM_PKG2} result)
          # message("Alex2", ${HRIM_PKG2})
        ENDFOREACH()
      endif()
   ENDFOREACH()
ENDFOREACH()

ament_package()

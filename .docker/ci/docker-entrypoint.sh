#!/bin/bash

export RESET="\e[0m"
export CYAN="\e[36m"
export RED="\e[31m"
export BOLD"\e[1m"
WS_PATH="/home/root/ros2_ws"
HRIM_FULL_PATH="/home/root/ros2_ws/src/hrim"
INSTALLATOR_PATH="${HRIM_FULL_PATH}/installator"
ROS2_SOURCE="/opt/ros/${ROS2_DISTRO}/setup.bash"
MODULES_SCHEMA="${HRIM_FULL_PATH}/models/schemas/module.xsd"

function validateSchemas()
{
  echo -e "${CYAN}Validating xml files!${RESET}"
  XML_FILES=$(find -name "*.xml" | grep -v topics | grep -v dataMapping)
  ERROR_XML=( )
  index=0
  for i in ${XML_FILES}; do
    xmllint --schema ${MODULES_SCHEMA} $i --noout  --xinclude 2>/dev/null
    result=$?
    if [ ${result} -ne 0 ]; then
      ERROR_XML[index]=$i
      index=$((index+1))
    fi
  done

  if [ "${#ERROR_XML[@]}" -gt 0 ]; then
    for i in ${ERROR_XML[@]}; do
      echo "Error on file: ${i}"
    done
    echo -e "${RED}Number of fails: ${#ERROR_XML[@]}${RESET}"
    exit 1
  else
    echo -e "${CYAN}All xml files validated!${RESET}"
  fi
}

function qaCode()
{
  echo -e "${CYAN}Linter checks for python code, using: pep8 ${BOLD}`pep8 --version`${reset}"
  pep8 ${INSTALLATOR_PATH}/hrim/
  result=$?
  if [ $result -ne 0 ]; then
    echo "${RED}Linter error please review it!${RESET}"
    exit 2
  fi
}

function installHRIM()
{
  echo -e "${CYAN}Installing HRIM command line utility${RESET}"
  cd ${INSTALLATOR_PATH}
  pip3 install -r requirements.txt
  python3 setup.py install
  result=$?
  if [ $result -eq 0 ]; then
    echo -e "${CYAN}HRIM tool succsesfully installed!${RESET}"
  else
    echo -e "${RED}Failed to install HRIM tool!${RESET}"
    exit 3
  fi
}

function generatePackages()
{
  cd ${HRIM_FULL_PATH}
  hrim generate --platform ros2 all
  result=$?
  if [ $result -eq 0 ]; then
    echo -e "${CYAN}All packages generated succsesfully!${RESET}"
  else
    echo -e "${RED} Error creating the packages, please review it!${RESET}"
    exit 4
  fi
}

function compileWS()
{
  source ${ROS2_SOURCE}
  cd ${WS_PATH}
  colcon build --merge-install
  result=$?
  if [ $result -eq 0 ]; then
    echo -e "${CYAN}ROS2 ws compile it!${RESET}"
  else
    echo -e "${RED}Failed to build ROS2 ws, please review it!{RESET}"
    exit 5
  fi
}

validateSchemas
qaCode
installHRIM
generatePackages
compileWS

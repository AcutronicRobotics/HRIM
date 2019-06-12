#!/bin/bash

export RESET="\e[0m"
export CYAN="\e[36m"
export RED="\e[31m"
export BOLD"=\e[1m"
export YELLOW"\e[93m"
WS_PATH="/home/root/ros2_ws"
HRIM_FULL_PATH="/home/root/ros2_ws/src/hrim"
HRIM_FULL_GENERATED_PATH="/home/root/ros2_ws/src/hrim/generated"
INSTALLATOR_PATH="${HRIM_FULL_PATH}/installator"
ROS2_SOURCE="/opt/ros/${ROS2_DISTRO}/setup.bash"
MODULES_SCHEMA="${HRIM_FULL_PATH}/models/schemas/module.xsd"
TEST_LOG_PATH=""

function installDependencies()
{
  echo -e "${CYAN}Checking for missing dependencies${RESET}"
  apt update -qq
  rosdep update -q
  rosdep install -q -y --from-paths ${HRIM_FULL_PATH} --rosdistro ${ROS2_DISTRO} --as-root=apt:false || true
  echo -e "${CYAN}All dependencies installed!${RESET}"
}

function validateSchemas()
{
  echo -e "${CYAN}Validating xml files!${RESET}"
  XML_FILES=$(find -name "*.xml" | grep -v topics | grep -v dataMapping | grep -v package.xml)
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
  echo -e "${CYAN}Linter checks for python code, using: pep8 ${BOLD}$(pep8 --version)${RESET}"
  pep8 ${INSTALLATOR_PATH}/hrim/
  result=$?
  if [ $result -ne 0 ]; then
    echo "${RED}pep8 error/s found, please review it!${RESET}"
    exit 2
  else
    echo -e "${CYAN}No pep8 errors found!${RESET}"
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
    echo -e "${CYAN}HRIM tool successfully installed!${RESET}"
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
    echo -e "${CYAN}All packages generated successfully!${RESET}"
  else
    echo -e "${RED} Error creating the packages, please review it!${RESET}"
    exit 4
  fi
}

function compileWS()
{
  echo -e "${CYAN}Compiling the work space for HRIM!${RESET}"
  source ${ROS2_SOURCE}
  cd ${WS_PATH}
  colcon build --merge-install --cmake-args -DHRIM_DIRECTORY=${HRIM_FULL_GENERATED_PATH}
  result=$?
  if [ $result -eq 0 ]; then
    echo -e "${CYAN}ROS2 ws compiled successfully!${RESET}"
  else
    echo -e "${RED}Failed to build ROS2 ws, please review it!${RESET}"
    exit 5
  fi
}

function testWorkspace()
{
  echo -e "${CYAN}Testing the work space${RESET}"
  cd ${WS_PATH}
  colcon test --merge-install --packages-select hrim_qa
  TEST_FAILURES=$(grep -HiRE '\(FAILED\)' ${TEST_LOG_PATH} | cut -d '-' -f 2 | cut -d '\' -f 1 | cut -d '(' -f 1)

  if [ -z ${TEST_FAILURES} ]; then
    echo -e "${CYAN}All tests passed!${RESET}"
    exit 0
  else
    echo -e "${YELLOW}Detected failures in the test/s${RESET}"
    for i in ${TEST_FAILURES}; do
      echo -e "${RED}${i}${RESET}"
    done
    exit 6
  fi
}

installDependencies
validateSchemas
qaCode
installHRIM
generatePackages
compileWS
testWorkspace

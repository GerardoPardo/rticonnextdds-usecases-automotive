###############################################################################
##                                                                           ##
##         Permission to modify and use for internal purposes granted.       ##
## This software is provided "as is", without warranty, express or implied.  ##
##                                                                           ##
###############################################################################

#! /bin/sh

# Note: In Linux this ends up being a bash shell...

if [[ -z "${ARCH}" ]]; then
   ARCH=x64Linux4gcc7.3.0
fi
EXE_DIR=../objs/${ARCH}


# Determine which terminal program to use.
TERM=notset
for terminal in xterm gnome-terminal lxterminal
do
  type ${terminal} >/dev/null 2>&1
  if test $? -eq 0 ; then
    TERM=${terminal}
    break 
  fi
done

# Did we find one we know how to use? If not, error.
case ${TERM} in
  xterm) ;;
  gnome-terminal) ;;
  lxterminal) ;;
  *)
    echo "Terminal; ${TERM} is unknown! Program error!"
    exit 1
esac

### Make sure all component programs are available
PROGRAMS="${EXE_DIR}/VisionSensor ${EXE_DIR}/Lidar ${EXE_DIR}/sensorFusion ${EXE_DIR}/hmi ${EXE_DIR}/collisionAvoidance ${EXE_DIR}/Platform ${EXE_DIR}/CameraImageDataSub ${EXE_DIR}/CameraImageDataPub"
echo "${PROGRAMS}"

for exe in ${PROGRAMS}
do
  if [ ! -r ${exe} ] ; then
    echo "${exe} is not present. Did you build the application?"
    exit 1
  fi

  if [ -d ${exe} ] ; then
    echo "${exe} is a directory. You cannot execute that"
    exit 1
  fi
done

cd ../resource

### Launch all component programs
POS_X=100
POS_Y=50
ROW_COUNTER=0

for exe in ${PROGRAMS}
do
  progname="${exe##*/}"
  geometry=75x6+${POS_X}+${POS_Y}
  case ${TERM} in
    lxterminal)
      ${TERM} ${GEOMETRY} -t ${progname} -e "${exe}" &
      ;;
    xterm)
      ${TERM} ${GEOMETRY} -e "${progname}" &
      ;;
    gnome-terminal) 
      ${TERM} --geometry ${geometry} --title ${progname}  -- "${exe}" &
#      ${TERM} ${GEOMETRY} -- "${exe}" &
      ;;
    *) exit 1
  esac
  
  let POS_Y+=160
  let ROW_COUNTER+=1
  if [ ${ROW_COUNTER} -eq 4 ] ; then
    ROW_COUNTER=0
    POS_X=850
    POS_Y=50
  fi

done

### When the user is done, let them kill the programs
echo -n "Press return to kill programs> "
read ignored

for exe in ${PROGRAMS}
do
  progname="${exe##*/}"
  # pkill seems to only match the first 14 chars
  pkill -9 "${progname:0:13}"
done

cd ../scripts

exit 0

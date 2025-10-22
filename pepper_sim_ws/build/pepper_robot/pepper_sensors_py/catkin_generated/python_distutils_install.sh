#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/shaytrix2/workspace/pepper_sim_ws/src/pepper_robot/pepper_sensors_py"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/shaytrix2/workspace/pepper_sim_ws/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/shaytrix2/workspace/pepper_sim_ws/install/lib/python3/dist-packages:/home/shaytrix2/workspace/pepper_sim_ws/build/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/shaytrix2/workspace/pepper_sim_ws/build" \
    "/usr/bin/python3" \
    "/home/shaytrix2/workspace/pepper_sim_ws/src/pepper_robot/pepper_sensors_py/setup.py" \
     \
    build --build-base "/home/shaytrix2/workspace/pepper_sim_ws/build/pepper_robot/pepper_sensors_py" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/shaytrix2/workspace/pepper_sim_ws/install" --install-scripts="/home/shaytrix2/workspace/pepper_sim_ws/install/bin"

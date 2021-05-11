#!/usr/bin/env python3

import signal
from typing import Any, Dict, Optional

from roslaunch import rlutil
from roslaunch.scriptapi import roslaunch # hacky way to access roslaunch.parent
from roslaunch.parent import ROSLaunchParent
from rospkg import RosPack
import rospy
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse

def kill_on_sig(*sig):
    """Signal handler routine that kills the program."""
    rospy.signal_shutdown('killed')
    raise KeyboardInterrupt()

class LaunchWrapper:
    """Wraps a roslaunch, keeping track of its state and allowing for re-launching."""

    def __init__(self, launch_file: str, args: Dict[str, Any]):
        """Creates a new roslaunch specification.

        Parameters
        ----------
        launch_file : str
            The launch file to launch.
        args : Dict[str, Any]
            The arguments to pass to the launch file.
        """
        self.launch_tuple = (launch_file, tuple(f'{k}:={v}' for k, v in args.items()))
        self.args = args
        self.rl_parent: Optional[ROSLaunchParent] = None
    
    def is_launched(self) -> bool:
        """Checks whether the launch is currently alive or not.

        Returns
        -------
        bool
            Whether the launch is active or not.
        """
        return self.rl_parent is not None and self.rl_parent.pm is not None\
            and not self.rl_parent.pm.is_shutdown
    
    def launch(self):
        """Launches the roslaunch, if it isn't already launched."""
        if not self.is_launched():
            rospy.loginfo('Starting up...')
            launch_id = rlutil.get_or_generate_uuid(None, False)
            self.rl_parent = ROSLaunchParent(launch_id, [self.launch_tuple], is_core=False)
            self.rl_parent.start()
            signal.signal(signal.SIGINT, kill_on_sig) # roslaunch messes with signal handlers
            signal.signal(signal.SIGHUP, kill_on_sig) # we'll handle them ourself, thank you very much
            signal.signal(signal.SIGTERM, kill_on_sig)
            rospy.loginfo('Started up!')

    def spin_once(self):
        """Spins the roslaunch instance, if it's launched."""
        if self.is_launched():
            self.rl_parent.spin_once()

    def kill(self):
        """Shuts down the roslaunch, if it's launched."""
        if self.is_launched():
            rospy.loginfo('Shutting down...')
            self.rl_parent.shutdown()
            self.rl_parent = None
            rospy.loginfo('Shut down!')

def main():
    """A node that allows for the toggling of an OpenCV video stream.

    Wraps a `video_stream_opencv` camera node so that it can be dynamically enabled and
    disabled at runtime via a ROS service. This is accomplished by using `LaunchWrapper`
    as defined above, which is used to launch and kill the camera node when it is
    enabled and disabled.

    Most ROS parameters will be passed to the camera node as arguments to the
    `camera.launch` launch file provided by the `video_stream_opencv` package[1]_. a few
    parameters are used by this node to configure its behaviour, and will not be passed
    to `camera.launch`; these are listed below, in the "Other Parameters" section.

    Other Parameters
    ----------------
    srv_name : str, optional
        The name of the enable/disable service. By default, "~set_enabled".
    enabled_by_default: bool, optional
        Whether the camera should be enabled or not on startup. By default, the camera
        will initially be enabled.

    References
    ----------
    .. [1] http://wiki.ros.org/video_stream_opencv
    """
    rospy.init_node('camera_control')

    # most params are passed through as args to camera.launch
    args = rospy.get_param('~', dict())
    srv_name = args.pop('srv_name', '~set_enabled')
    enabled = args.pop('enabled_by_default', True)

    # find camera package
    rospack = RosPack()
    video_stream_pkg = rospack.get_path('video_stream_opencv')

    # wrap roslaunch
    launcher = LaunchWrapper(f'{video_stream_pkg}/launch/camera.launch', args)
    try:
        if enabled:
            launcher.launch()

        # create enable/disable service
        def srv_cb(req: SetBoolRequest) -> SetBoolResponse:
            if req.data:
                if launcher.is_launched():
                    return SetBoolResponse(False, 'Camera is already enabled!')
                else:
                    launcher.launch()
            elif launcher.is_launched():
                launcher.kill()
            else:
                return SetBoolResponse(False, 'Camera is not enabled!')
            return SetBoolResponse(True, '')
        srv = rospy.Service(srv_name, SetBool, srv_cb)

        # spin
        sleeper = rospy.Rate(10)
        while not rospy.is_shutdown():
            launcher.spin_once()
            sleeper.sleep()
    finally:
        # make sure to clean up once everything is said and done
        launcher.kill()

if __name__ == '__main__':
    main()

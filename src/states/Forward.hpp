#pragma once

namespace trajectory_follower
{
    namespace proxies {
        class Task;
	class TurnVelocityToSteerAngleTask;
    }
}
namespace traversability
{
    namespace proxies {
        class Simple;
    }
}
namespace motion_planning_libraries
{
    namespace proxies {
        class Task;
    }
}
namespace graph_slam
{
    namespace proxies {
        class VelodyneSLAM;
    }
}
namespace localization {
    namespace proxies {
	class VelodyneInMLS;
	//class PoseProvider;
    }
}
namespace mars {
    namespace proxies {
	class RotatingLaserRangeFinder;
	class IMU;
	class Joints;
    }
}
namespace drive_mode_controller {
    namespace proxies {
	class Task;
    }
}
namespace odometry
{
    namespace proxies {
        class Skid;
    }
}
namespace joint_dispatcher
{
    namespace proxies {
        class Task;
    }
}
namespace drive_mode_controller
{
    namespace proxies {
        class Task;
    }
}
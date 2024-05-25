# Luxonis camera launch

Package for launching luxonis camera nodes in ros2.

### Parameters
Used via `f1tenth_awsim_data_recorder.param.yaml` file in config directory.
| Name         | Type | Description  |
| ------------ | ---- | ------------ |
| params_file | int  | Maximum count of points saved (remaining if path is shorter then specified amount it is filled with 0) |
| rectify_rgb | bool | Rectify rgb image default value is True |
| name | string | Name of node default value is oak |
| odom | bool | Run odometry for camera default value is False |
| slam | bool | Run slam node default value False. Requires odometry. |
| rtabmap_viz | bool | Visualize rtabmap default value is False. Required slam node. |
| mobilenet | bool | runs mobilenet node publisher |

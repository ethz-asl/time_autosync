# auto_timesync

## A ros node with two functions
- Making camera arrival timestamps a hair less garbage.
- Estimates the absolute time offset between the camera and a ridgidly attached IMU

Under the skin the system is just a central difference kalman filter.
- It assumes a constant camera framerate and uses this to get rid of some of the jitter from arrival times
- Absolute offsets are calculated by aligning the IMU's measured angular velocity to that perceived by a simple Lucas-Kanade tracker running on the camera images. Obviously this will only work on systems that undergo significant movement.

This filter is pretty new and only tested on a couple of datasets I had lying around, so expect some issues when you first run it.

## Parameters
------

### General Parameters
| Parameter | Description | Default |
| --------------------  |:-----------:| :-------:|
| `stamp_on_arrival` |  If true it will throw out the image timestamps and replace them with arrival times. | false |
| `max_imu_data_age_s` | How long in seconds to keep IMU data, the camera timing offset needs to be in this range. | 2.0 |
| `delay_by_n_frames` | Delays putting camera images into the filter by this many frames. Used to ensure there are plenty of IMU messages ahead of the frame to get IMU messages from. **Note this does not cause any delay in the output of timesynced images.** | 3 |
| `focal_length` | The focal length of the camera. A rough guess is fine and the system seems to run ok on distorted images | 460 |
| `use_pointcloud` | Uses pointcloud as input rather then an image. In this mode ICP matiching is used to estimate the rotation | false |
| `calc_offset` | If true the absolute time offset between the camera and an IMU is calculated. If false only some dejittering is performed (this saves a lot of CPU and most of the above parameters don't apply). | true |

### Filter Parameters
| Parameter | Description | Default |
| --------------------  |:-----------:| :-------:|
| `verbose` |  Prints a large amount of information about the filters internal state | false |
| `mah_threshold` | Mahalanobis distance theshold for accepting measurement updates | 10.0 |
| `inital_delta_t` | Inital estimate in seconds for rate at which images are received. **Set to 1/framerate**. Note if you set it too small by a factor of 2 or more the automatic frame drop detection will cause weird effects | 0.05 |
| `inital_offset` | Inital estimate in seconds for the offset between the camera and IMU timestamps | 0.0 |
| `inital_timestamp_sd` | Inital standard deviation of systems timestamp in seconds (which is initalized to the first images timestamp) | 0.1 |
| `inital_delta_t_sd` | Inital standard deviation of the time between frames | 0.1 |
| `inital_offset_sd` | Inital standard deviation of the offset between the camera and IMU | 0.1 |
| `timestamp_sd` | Standard deviation of the received image timestamps | 0.02 |
| `angular_velocity_sd` | Standard deviation of the angular velocity estimated from the camera images | 0.03 |
| `delta_t_sd` | Standard deviation of the process noise for the time between frames | 0.0001 |
| `offset_sd` | Standard deviation of the process noise for the time offset between the IMU and camera | 0.0001 |

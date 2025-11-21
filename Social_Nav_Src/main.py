import time
import numpy as np
import rospy
import tf
import message_filters
from geometry_msgs.msg import Point, PointStamped, Twist
from sensor_msgs.msg import Image, CameraInfo, LaserScan
from cv_bridge import CvBridge, CvBridgeError
import cv2
import tensorflow 
import tensorflow_probability as tfp
from ultralytics import YOLO

from control import Control
from Social_State_Map import SocialStateMap
from Target_Human_State_Map import TargetHumanStateMap
from Obstacles_State_Map import ObstaclesStateMap
from Visualise import visualise_reward_over_episodes, save_models, visualise_model_loss, plot_mu_std
from Reset_Env import reset, spawn_model, delete_model, get_model_pose
from agent import ActorCritic
from Actor_model_sync_collide import actor_collision, spawn_model_cylinder

tfd = tfp.distributions


class SensorsNode:
    """
    Handles RGB-D and LIDAR sensor data, performs YOLO detection, transforms
    detections to robot frame, and publishes processed images.
    """

    TOPIC_COLOR_IMAGE = "/d435/color/image_raw"
    TOPIC_DEPTH_IMAGE = "/d435/depth/image_raw"
    TOPIC_RGB_CAMERA_INFO = "/d435/color/camera_info"
    TOPIC_DEPTH_CAMERA_INFO = "/d435/depth/camera_info"
    TOPIC_OUTPUT_IMAGE = "/d435/yolo_detection_result"
    TOPIC_LIDAR = "/front/scan"
    CONFIDENCE_THRESHOLD = 0.5

    def __init__(self):
        self.bridge = CvBridge()
        self.model = YOLO("yolo11n.pt")
        self.listener = tf.TransformListener()
        self.rgb_intrinsics = [None] * 4
        self.depth_intrinsics = [None] * 4
        self.latest_depth_image = None
        self.latest_color_image = None
        self.human_goal = []
        self.humans_detect_pose = []
        self.lidar_data = []

        self.image_pub = rospy.Publisher(self.TOPIC_OUTPUT_IMAGE, Image, queue_size=10)
        self._setup_subscribers()

    def _setup_subscribers(self):
        """Initialize synchronized ROS subscribers."""
        rgb_image_sub = message_filters.Subscriber(self.TOPIC_COLOR_IMAGE, Image)
        depth_image_sub = message_filters.Subscriber(self.TOPIC_DEPTH_IMAGE, Image)
        rgb_camera_info_sub = message_filters.Subscriber(self.TOPIC_RGB_CAMERA_INFO, CameraInfo)
        depth_camera_info_sub = message_filters.Subscriber(self.TOPIC_DEPTH_CAMERA_INFO, CameraInfo)
        lidar_sub = message_filters.Subscriber(self.TOPIC_LIDAR, LaserScan)

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [rgb_image_sub, depth_image_sub, rgb_camera_info_sub, depth_camera_info_sub, lidar_sub],
            queue_size=5,
            slop=0.1
        )
        self.ts.registerCallback(self.sync_callback)

    def _update_intrinsics(self, camera_info, intrinsic_array):
        """Update camera intrinsic parameters fx, fy, cx, cy."""
        intrinsic_array[0] = camera_info.K[0]
        intrinsic_array[1] = camera_info.K[4]
        intrinsic_array[2] = camera_info.K[2]
        intrinsic_array[3] = camera_info.K[5]

    def sync_callback(self, rgb_image, depth_image, rgb_camera_info, depth_camera_info, lidar_msg):
        """Callback for synchronized sensor data."""
        try:
            self.latest_color_image = self.bridge.imgmsg_to_cv2(rgb_image, "bgr8")
            self.latest_depth_image = self.bridge.imgmsg_to_cv2(depth_image, "16UC1")
        except CvBridgeError as e:
            rospy.logerr(f"Error converting images: {e}")
            return

        self._update_intrinsics(rgb_camera_info, self.rgb_intrinsics)
        self._update_intrinsics(depth_camera_info, self.depth_intrinsics)
        self._process_lidar(lidar_msg)

    def _process_lidar(self, lidar_msg):
        """Process LIDAR data and transform points to robot frame."""
        ranges = np.array(lidar_msg.ranges)
        angles = np.arange(
            lidar_msg.angle_min, 
            lidar_msg.angle_max + lidar_msg.angle_increment, 
            lidar_msg.angle_increment
        )
        valid_indices = ~np.isinf(ranges)
        self.lidar_data = []

        for r, theta in zip(ranges[valid_indices], angles[valid_indices]):
            pt = PointStamped()
            pt.header.frame_id = "front_laser"
            pt.header.stamp = rospy.Time(0)
            pt.point.x = r * np.cos(theta)
            pt.point.y = r * np.sin(theta)
            pt.point.z = 0.0
            try:
                transformed = self.listener.transformPoint("base_link", pt)
                self.lidar_data.append([transformed.point.x, transformed.point.y])
            except (tf.Exception, tf.LookupException, tf.ConnectivityException):
                rospy.logwarn("TF transform failed for LIDAR point")
                continue

    def transform_to_robot_frame(self, point_3d):
        """Transform a 3D point from camera frame to robot frame."""
        pt = PointStamped()
        pt.header.frame_id = "d435_depth_optical_frame"
        pt.header.stamp = rospy.Time(0)
        pt.point.x, pt.point.y, pt.point.z = point_3d
        try:
            return self.listener.transformPoint("base_link", pt).point
        except (tf.Exception, tf.LookupException, tf.ConnectivityException):
            rospy.logwarn("Transformation failed")
            return Point()

    def predict_and_detect(self, img):
        """Perform YOLO detection and annotate humans."""
        results = self.model.track(img, tracker="bytetrack.yaml", classes=[0], conf=self.CONFIDENCE_THRESHOLD)
        for result in results:
            for box in result.boxes:
                if box is None:
                    continue
                if box.conf[0].item() >= self.CONFIDENCE_THRESHOLD:
                    self._draw_bounding_box(img, box)
        return img

    def _draw_bounding_box(self, img, box):
        """Draw bounding box on image and update human goal/pose."""
        x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
        u, v = (x1 + x2) // 2, (y1 + y2) // 2
        cv2.rectangle(img, (x1, y1), (x2, y2), (255, 0, 0), 2)

        if self.latest_depth_image is None:
            return

        z = self.estimate_depth(u, v)
        if np.isnan(z):
            rospy.logwarn("Invalid depth value")
            return

        x, y = self.calculate_3d_coordinates(u, v, z)
        transformed_point = self.transform_to_robot_frame([x, y, z])

        if self.is_yellow_present(img, x1, y1, x2, y2):
            self.human_goal = [transformed_point.x, transformed_point.y, transformed_point.z]
        else:
            self.humans_detect_pose.append([transformed_point.x, transformed_point.y, transformed_point.z])

        # Visualization
        cv2.putText(img, f"Human: {box.conf[0]:.2f}", (x1, y1 - 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
        cv2.putText(img, f"3D: X={transformed_point.x:.2f} Y={transformed_point.y:.2f} Z={transformed_point.z:.2f}",
                    (x1, y1 - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    def estimate_depth(self, u: int, v: int) -> float:
        """Estimate depth from the depth image."""
        try:
            depth_value = self.latest_depth_image[v, u]
            return depth_value / 1000.0 if depth_value > 0 else float('nan')
        except IndexError:
            return float('nan')

    def calculate_3d_coordinates(self, u: int, v: int, z: float):
        """Calculate 3D coordinates from pixel coordinates and depth."""
        fx, fy, cx, cy = self.depth_intrinsics
        x = (u - cx) * z / fx
        y = (v - cy) * z / fy
        return x, y

    def is_yellow_present(self, img, x1, y1, x2, y2) -> bool:
        """Check if yellow is present in the bounding box (human of interest)."""
        roi = img[y1:y2, x1:x2]
        hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([30, 255, 255])
        mask = cv2.inRange(hsv_roi, lower_yellow, upper_yellow)
        yellow_ratio = cv2.countNonZero(mask) / (roi.shape[0] * roi.shape[1])
        return yellow_ratio >= 0.1

    def get_latest_color_img(self):
        return self.latest_color_image

    def get_human_goal(self):
        return self.human_goal

    def get_humans_detect_pose(self):
        return self.humans_detect_pose

    def set_humans_detect_pose(self, humans_detect_pose):
        self.humans_detect_pose = humans_detect_pose

    def get_lidar_data(self):
        return self.lidar_data

    def get_bridge(self):
        return self.bridge

    def get_img_pub(self):
        return self.image_pub


# ---------------- RL Helper Functions ---------------- #

def get_observation(TargetHumanState, SocialState, ObstaclesState, sensors_node: SensorsNode):
    """
    Get the current observation from sensors and state maps.
    """
    processed_image = sensors_node.predict_and_detect(sensors_node.get_latest_color_img())
    try:
        msg = sensors_node.get_bridge().cv2_to_imgmsg(processed_image, encoding="bgr8")
        sensors_node.get_img_pub().publish(msg)
    except CvBridgeError as e:
        rospy.logerr(f"Error converting processed image: {e}")

    humans_detect_pose = sensors_node.get_humans_detect_pose()
    sensors_node.set_humans_detect_pose([])
    lidar_data = sensors_node.get_lidar_data()
    target_human_pos = get_model_pose("pedestrian")
    x_h, y_h = target_human_pos.position.x, target_human_pos.position.y

    target_human_state = TargetHumanState.distance_to_goal_grid([x_h, y_h, 0])
    social_state = SocialState.social_distace_grid(humans_detect_pose)
    obstacle_state = ObstaclesState.obstacle_distance_grid(lidar_data)

    return [target_human_state, social_state, obstacle_state, [x_h, y_h]]


def state_fusion(observation):
    """Flatten all state components into a single state vector."""
    return [*observation[0], *observation[1], *observation[2], *observation[3]]


def compute_full_returns(rewards, dones, gamma):
    """Compute discounted returns."""
    returns = []
    R = 0
    for r, done in zip(reversed(rewards), reversed(dones)):
        if done:
            R = 0
        R = r + gamma * R
        returns.insert(0, R)
    return returns


@tensorflow.function
def train_step(agent, states, actions, returns):
    """Perform a single training step on the agent."""
    # Critic update
    with tensorflow.GradientTape() as tape:
        values = tensorflow.squeeze(agent.critic(states), axis=1)
        critic_loss = tensorflow.reduce_mean((returns - values) ** 2)
    critic_grads = tape.gradient(critic_loss, agent.critic.trainable_variables)
    agent.critic_optimizer.apply_gradients(zip(critic_grads, agent.critic.trainable_variables))

    # Actor update
    with tensorflow.GradientTape() as tape:
        mu = agent.actor(states)
        std = tensorflow.exp(agent.log_std)
        std = tensorflow.expand_dims(std, axis=0)
        std = tensorflow.repeat(std, repeats=states.shape[0], axis=0)
        dist = tfd.TruncatedNormal(loc=mu, scale=std, low=[0.0, -1.0], high=[1.0, 1.0])
        log_probs = tensorflow.reduce_sum(dist.log_prob(actions), axis=1)
        advantages = returns - tensorflow.squeeze(agent.critic(states), axis=1)
        actor_loss = -tensorflow.reduce_mean(log_probs * advantages)

    actor_grads = tape.gradient(actor_loss, agent.actor.trainable_variables)
    agent.actor_optimizer.apply_gradients(zip(actor_grads, agent.actor.trainable_variables))

    return critic_loss, actor_loss, mu, std


# ---------------- Main Training Loop ---------------- #

def main():
    rospy.init_node('yolo_detection_node', anonymous=True)
    sensors_node = SensorsNode()
    controller = Control()
    cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    # Initialize state maps
    grid_size, cell_size, pedestrian_social_space, r_radius = 3, 1, 1.2, 0.5
    TargetHumanState = TargetHumanStateMap(grid_size, cell_size)
    SocialState = SocialStateMap(grid_size, cell_size, pedestrian_social_space, r_radius)
    ObstaclesState = ObstaclesStateMap(grid_size, cell_size)

    # RL parameters
    state_size = (grid_size * grid_size * 3) + 2
    continuous_action_space = 2
    lr_actor, lr_critic = 1e-4, 1e-4
    gamma, batch_size, episode_steps, episodes = 0.99, 128, 150, 12800

    agent = ActorCritic(state_size, continuous_action_space, lr_actor, lr_critic)
    Reward_table, actor_loss_ls, critic_loss_ls, mu_ls, std_ls = [], [], [], [], []

    # Spawn cylinder and pedestrian
    spawn_model_cylinder("cylinder1",
                         "/home/alien/Desktop/catkin_ws/src/ridgeback_simulator/ridgeback_gazebo/Media/models/cylinder/cylinder.sdf",
                         get_model_pose("actor1"))

    delete_model("pedestrian")
    time.sleep(3)
    spawn_model("pedestrian",
                "/home/alien/Desktop/catkin_ws/src/ridgeback_simulator/ridgeback_gazebo/Media/models/person/pedestrian.sdf")
    time.sleep(3)

    for epoch in range(episodes // batch_size):
        rospy.loginfo(f"Epoch {epoch + 1}")
        agent.decay_log_std(epoch, decay_rate=0.01)
        episodes_states, episodes_actions, episodes_rewards, episodes_dones = [], [], [], []

        for episode in range(batch_size):
            reset()
            time.sleep(2)
            episode_states, episode_actions, episode_rewards, episode_dones, episode_reward = [], [], [], [], 0

            for step in range(episode_steps):
                rospy.loginfo(f"Episode {episode + 1}, Step {step + 1}")
                actor_collision()

                observation = get_observation(TargetHumanState, SocialState, ObstaclesState, sensors_node)
                state = np.array(state_fusion(observation), dtype=np.float32)
                action = agent.select_action(state)
                cmd_vel_pub.publish(controller.action_continuous(action))
                rospy.Rate(5).sleep()

                next_observation = get_observation(TargetHumanState, SocialState, ObstaclesState, sensors_node)
                next_state = np.array(state_fusion(next_observation), dtype=np.float32)

                reward, done = controller.reward(state, action, next_state, step, episode_steps)
                episode_reward += reward

                episode_states.append(state)
                episode_actions.append(action)
                episode_rewards.append(reward)
                episode_dones.append(float(done))

                if done:
                    break

            rospy.loginfo(f"Episode {episode + 1} reward: {episode_reward}")
            Reward_table.append(episode_reward)
            episodes_states.append(episode_states)
            episodes_actions.append(episode_actions)
            episodes_rewards.append(episode_rewards)
            episodes_dones.append(episode_dones)

        # Flatten batch and compute returns
        batch_states, batch_actions, batch_returns = [], [], []
        for ep_rewards, ep_dones, ep_states, ep_actions in zip(episodes_rewards, episodes_dones, episodes_states, episodes_actions):
            returns = compute_full_returns(ep_rewards, ep_dones, gamma)
            batch_states.extend(ep_states)
            batch_actions.extend(ep_actions)
            batch_returns.extend(returns)

        states = tensorflow.convert_to_tensor(batch_states, dtype=tensorflow.float32)
        actions = tensorflow.convert_to_tensor(batch_actions, dtype=tensorflow.float32)
        returns = tensorflow.convert_to_tensor(batch_returns, dtype=tensorflow.float32)
        returns = (returns - tensorflow.reduce_mean(returns)) / (tensorflow.math.reduce_std(returns) + 1e-8)

        critic_loss, actor_loss, mu, std = train_step(agent, states, actions, returns)
        rospy.loginfo(f"Epoch {epoch + 1}, Critic Loss: {critic_loss.numpy()}, Actor Loss: {actor_loss.numpy()}")

        actor_loss_ls.append(actor_loss.numpy())
        critic_loss_ls.append(critic_loss.numpy())
        mu_ls.append(mu.numpy())
        std_ls.append(std.numpy())

    # Visualize and save
    visualise_reward_over_episodes(Reward_table, episodes, window_size=100)
    visualise_model_loss(actor_loss_ls, critic_loss_ls)
    plot_mu_std(mu_ls, std_ls)
    save_models(agent.actor, agent.critic)


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

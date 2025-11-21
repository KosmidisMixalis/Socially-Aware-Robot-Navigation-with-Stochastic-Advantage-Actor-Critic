from geometry_msgs.msg import Twist

class Control:


    def reward(self, state, action, new_state, step, episode_steps):
        reward = 0.0
        done = False
        distance_diff = state[5] - new_state[5]
        
        if step >= episode_steps:
            return -2.0, True

        # Terminal condition: goal reached proxemics
        if 0.7 < new_state[5] < 1.2:
            return 10.0, True


        # Terminal condition: unsafe proximity or collision with human
        # Only reward standing still when human is in front AND the agent is not camping
        if state[14] == 1.0:
            if action[0] < 0.01:
                reward += 0.5  # reward safe stop
            else:
                return -2.0, True  # penalize risky forward move
        else:
            # human not near — discourage standing still
            if action[0] < 0.01:
                reward -= 0.2


        # Terminal condition: obstacle collision
        if action[0] >= 0.01 and state[23]== 1.0:
            return -2.0, True
        

        reward += distance_diff * 5.0  # Smoother reward scaling

        # Encourage smooth motion
        reward += 0.1 * action[0]        # move forward bonus
        reward -= 0.01 * abs(action[1])   # penalize sharp turns

        return reward, done
    

    def action_continuous(self, action):
    
        twist = Twist()
            
        twist.linear.x = action[0]
        twist.angular.z = action[1]
        return twist
    

 
import numpy as np
import tensorflow as tf
from tensorflow import keras
from tensorflow.keras.layers import Dense, Concatenate, Input, LeakyReLU # type: ignore
import tensorflow_probability as tfp
tfd = tfp.distributions

class ActorCritic:
    def __init__(self, state_shape, action_space, lr_actor, lr_critic, initial_log_std=0.0):
        self.action_dim = action_space
        self.actor = self.build_actor(state_shape)
        self.critic = self.build_critic(state_shape)
        
        self.actor_optimizer = tf.keras.optimizers.Adam(learning_rate=lr_actor)
        self.critic_optimizer = tf.keras.optimizers.Adam(learning_rate=lr_critic)

        # Fixed log_std with custom decay — NOT trainable
        self.initial_log_std = initial_log_std
        self.log_std = tf.Variable(
            initial_value=tf.ones(self.action_dim) * initial_log_std,
            trainable=False, dtype=tf.float32)

    def build_actor(self, state_shape):
        state_input = Input(shape=(state_shape,))
        x = Dense(512)(state_input)
        x = LeakyReLU(alpha=0.1)(x)
        x = Dense(256)(x)
        x = LeakyReLU(alpha=0.1)(x)
        x = Dense(128)(x)
        x = LeakyReLU(alpha=0.1)(x)
        x = Dense(64)(x)


        # Output layers:
        linear_mu = Dense(1, activation='sigmoid')(x)     # [0, 1] only forward
        angular_mu = Dense(1, activation='tanh')(x)       # [-1, 1] both right and left turn
        
        mu = Concatenate()([linear_mu, angular_mu])  # shape: (None, 2)


        model = keras.Model(inputs=state_input, outputs=mu)
        return model

    def build_critic(self, state_shape):
        model = keras.Sequential([
            Input(shape=(state_shape,)),
            Dense(512, activation='relu'),
            Dense(256, activation='relu'),
            Dense(128, activation='relu'),
            Dense(64, activation='relu'),
            Dense(1)
        ])
        return model

   
    def select_action(self, state):
        state = np.expand_dims(state, axis=0).astype(np.float32)

        mu = self.actor(state)
        std = tf.exp(self.log_std)

        dist = tfd.TruncatedNormal(loc=mu, scale=std, low=[0.0, -1.0], high=[1.0, 1.0])
        action = dist.sample()
       
    
        return np.array(action[0])
        
    def decay_log_std(self, current_epoch, decay_rate):
        new_log_std_scalar = self.initial_log_std - decay_rate * current_epoch
        new_log_std_vector = tf.ones(self.action_dim, dtype=tf.float32) * new_log_std_scalar
        self.log_std.assign(new_log_std_vector)

import matplotlib.pyplot as plt
import os
import numpy as np

# Define parameters
path = os.path.expanduser("~/Desktop/ActorCriticTest/Test")  # Adjust to your actual save location
# Ensure save directory exists
os.makedirs(path, exist_ok=True)

# Apply moving average for smoothing
def moving_average(data, window_size):
    return np.convolve(data, np.ones(window_size) / window_size, mode='valid')

def visualise_reward_over_episodes(Reward_table, num_episodes, window_size):
    episodes = np.arange(num_episodes)
    rewards = np.array(Reward_table)

    smoothed_rewards = moving_average(rewards, window_size)
    smoothed_episodes = np.arange(len(smoothed_rewards))

    plt.figure(figsize=(10, 6))
    plt.plot(episodes, rewards, label="Raw Reward", color="gray", alpha=0.3)
    plt.plot(smoothed_episodes, smoothed_rewards, label=f"Smoothed (window={window_size})", color="blue", linewidth=2)

    plt.xlabel("Episode")
    plt.ylabel("Reward")
    plt.title("Training Reward Over Time")
    plt.legend()
    plt.grid(True)

    os.makedirs(path, exist_ok=True)
    plt.savefig(os.path.join(path, "Reward.png"))
    plt.close()

    print(f"Plot saved in {path}/Reward.png")

def visualise_model_loss(actor_loss_ls, critic_loss_ls):

    plt.figure(figsize=(10, 5))
    plt.plot(actor_loss_ls, label='Actor Loss', color='blue')
    plt.plot(critic_loss_ls, label='Critic Loss', color='red')
    plt.xlabel('Training Steps')
    plt.ylabel('Loss')
    plt.title('Actor and Critic Loss over Time')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(os.path.join(path, "Loss.png"))
    plt.close()  # Close the figure to free memory



def plot_mu_std(mu_log, std_log):
    mu_means = [np.mean(mu, axis=0) for mu in mu_log]    # shape: [epochs, action_dim]
    std_means = [np.mean(std, axis=0) for std in std_log]

    mu_means = np.array(mu_means)
    std_means = np.array(std_means)

    epochs = np.arange(len(mu_means))

    action_dims = mu_means.shape[1]

    plt.figure(figsize=(14, 6))

    for dim in range(action_dims):
        plt.subplot(2, action_dims, dim + 1)
        plt.plot(epochs, mu_means[:, dim])
        plt.title(f'Mean (mu) for action dim {dim}')
        plt.xlabel('Epoch')
        plt.ylabel('Mean action value')
        plt.grid(True)

        plt.subplot(2, action_dims, action_dims + dim + 1)
        plt.plot(epochs, std_means[:, dim])
        plt.title(f'Std (sigma) for action dim {dim}')
        plt.xlabel('Epoch')
        plt.ylabel('Std dev')
        plt.grid(True)

    plt.tight_layout()
    plt.savefig(os.path.join(path, "Distribution.png"))



def save_models(actor, critic):
    
    actor.save(os.path.join(path, "actor_model"))
    critic.save(os.path.join(path, "critic_model"))
    print(f"Models saved in {path}")


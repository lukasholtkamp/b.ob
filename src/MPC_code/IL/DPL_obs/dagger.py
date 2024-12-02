import numpy as np
import os
import json
import torch
import torch.nn as nn
import torch.optim as optim
from environment import SimpleEnvironment
from expert_policy import expert_policy
from policy_model import PolicyModel
import logging

# Configure logging
logging.basicConfig(
    level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s"
)

# Paths for data and models
DATA_DIR = "data/"
MODELS_DIR = "models/"
TRAJECTORIES_DIR = "trajectories/"
os.makedirs(MODELS_DIR, exist_ok=True)
os.makedirs(TRAJECTORIES_DIR, exist_ok=True)

# Step 1: Load obstacle-aware pre-collected data
logging.info("Loading pre-collected obstacle-aware data...")
train_states = np.load(
    os.path.join(DATA_DIR, "train_states_obs.npy")
)  # State data including obstacles
train_controls = np.load(
    os.path.join(DATA_DIR, "train_controls_obs.npy")
)  # Control data

# Convert data to PyTorch tensors
train_states_tensor = torch.tensor(train_states, dtype=torch.float32)
train_controls_tensor = torch.tensor(train_controls, dtype=torch.float32)

# Step 2: Initialize environment, policy model, and optimizer
logging.info("Initializing environment and model...")
obstacle_count = (
    train_states.shape[1] - 5
) // 3  # Compute number of obstacles from input dimension
policy = PolicyModel(obstacle_count=1)
optimizer = optim.Adam(policy.parameters(), lr=1e-3, weight_decay=1e-4)
criterion = nn.MSELoss()

env = SimpleEnvironment(
    dt=0.3,
    max_steps=60
)

# Step 3: Pretrain the policy on pre-collected data
logging.info("Pretraining the policy...")
num_pretrain_epochs = 15
batch_size = 2048
train_dataset = torch.utils.data.TensorDataset(
    train_states_tensor, train_controls_tensor
)
train_loader = torch.utils.data.DataLoader(
    train_dataset, batch_size=batch_size, shuffle=True
)

for epoch in range(num_pretrain_epochs):
    epoch_loss = 0
    for batch_states, batch_actions in train_loader:
        optimizer.zero_grad()
        predictions = policy(batch_states)
        loss = criterion(predictions, batch_actions)
        loss.backward()
        optimizer.step()
        epoch_loss += loss.item()

    logging.info(
        f"Pretraining Epoch {epoch + 1}/{num_pretrain_epochs}, Loss: {epoch_loss / len(train_loader):.4f}"
    )

# Save the pretrained model
pretrained_path = os.path.join(MODELS_DIR, "pretrained_policy.pth")
torch.save(policy.state_dict(), pretrained_path)
logging.info(f"Pretrained policy saved to '{pretrained_path}'.")

# Step 4: Prepare for DAgger with Beta Blending
logging.info("Starting DAgger iterations with beta blending...")
dataset_states = train_states.tolist()
dataset_actions = train_controls.tolist()

num_iterations = 15
new_samples_per_iteration = 30000

for iteration in range(num_iterations):
    logging.info(f"Starting DAgger Iteration {iteration + 1}/{num_iterations}...")

    # Compute beta: Linearly decrease over iterations
    beta = max(1.0 - iteration / num_iterations, 0.1)  # Minimum blending factor is 0.1
    logging.info(f"Blending factor (beta): {beta:.2f}")

    state = env.reset()
    done = False
    iteration_states = []
    iteration_actions = []
    iteration_trajectory = []  # Store detailed trajectory data
    samples_collected = 0

    while samples_collected < new_samples_per_iteration:
        # Generate actions from both expert and learned policy
        policy_action = (
            policy(torch.tensor(state, dtype=torch.float32)).detach().numpy()
        )
        expert_action = expert_policy(state)

        # Blend actions using beta
        blended_action = beta * expert_action + (1 - beta) * policy_action
        action_difference = np.linalg.norm(blended_action - expert_action)

        iteration_states.append(state)
        iteration_actions.append(expert_action)  # Use expert actions for training data

        # Save trajectory data
        iteration_trajectory.append(
            {
                "state": state.tolist(),
                "policy_action": policy_action.tolist(),
                "expert_action": expert_action.tolist(),
                "blended_action": blended_action.tolist(),
                "action_difference": action_difference,
            }
        )

        samples_collected += 1

        state, _, done = env.step(blended_action)
        if done:
            state = env.reset()

        if samples_collected % 1000 == 0:
            logging.info(
                f"Collected {samples_collected}/{new_samples_per_iteration} samples."
            )

    logging.info(f"Collected {samples_collected} new state-action pairs.")

    # Save trajectory data to JSON file
    trajectory_path = os.path.join(
        TRAJECTORIES_DIR, f"iteration_{iteration + 1}_trajectory.json"
    )
    with open(trajectory_path, "w") as f:
        json.dump(iteration_trajectory, f, indent=2)
    logging.info(f"Trajectory data saved to '{trajectory_path}'.")

    # Update the dataset
    dataset_states.extend(iteration_states)
    dataset_actions.extend(iteration_actions)

    # Limit dataset size to avoid memory issues
    if len(dataset_states) > 200000:
        dataset_states = dataset_states[-200000:]
        dataset_actions = dataset_actions[-200000:]
        logging.info("Dataset size limited to the most recent 200,000 samples.")

    # Convert to tensors and prepare DataLoader
    aggregated_states_tensor = torch.tensor(dataset_states, dtype=torch.float32)
    aggregated_actions_tensor = torch.tensor(dataset_actions, dtype=torch.float32)
    train_dataset = torch.utils.data.TensorDataset(
        aggregated_states_tensor, aggregated_actions_tensor
    )
    train_loader = torch.utils.data.DataLoader(
        train_dataset, batch_size=batch_size, shuffle=True
    )

    # Train the policy on the aggregated dataset
    for epoch in range(5):
        epoch_loss = 0
        for batch_states, batch_actions in train_loader:
            optimizer.zero_grad()
            predictions = policy(batch_states)
            loss = criterion(predictions, batch_actions)
            loss.backward()
            optimizer.step()
            epoch_loss += loss.item()

        logging.info(
            f"DAgger Iteration {iteration + 1}/{num_iterations}, Epoch {epoch + 1}/5, Loss: {epoch_loss / len(train_loader):.4f}"
        )

    policy_path = os.path.join(MODELS_DIR, f"policy_iteration_{iteration + 1}.pth")
    torch.save(policy.state_dict(), policy_path)
    logging.info(
        f"Saved policy after DAgger iteration {iteration + 1} to '{policy_path}'."
    )

# Save the final policy model
final_path = os.path.join(MODELS_DIR, "final_policy.pth")
torch.save(policy.state_dict(), final_path)
logging.info(f"Final trained policy saved to '{final_path}'.")

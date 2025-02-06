from environment import SimpleEnvironment
from expert_policy import expert_policy
from policy_model import PolicyModel
import numpy as np
import os
import torch
import torch.nn as nn
import torch.optim as optim
import json

# Paths for data, models, and logs
DATA_DIR = "data/"
MODELS_DIR = "models/"
LOG_DIR = "logs/"
os.makedirs(MODELS_DIR, exist_ok=True)
os.makedirs(LOG_DIR, exist_ok=True)

# Step 1: Load pre-collected data
print("Loading pre-collected data...")
train_states = np.load(os.path.join(DATA_DIR, "train_states.npy"))  # Shape: [N, 5]
train_controls = np.load(os.path.join(DATA_DIR, "train_controls.npy"))  # Shape: [N, 3]

# Convert data to PyTorch tensors
train_states_tensor = torch.tensor(train_states, dtype=torch.float32)
train_controls_tensor = torch.tensor(train_controls, dtype=torch.float32)

# Step 2: Initialize environment, policy model, and optimizer
print("Initializing environment and model...")
env = SimpleEnvironment(dt=0.3, max_steps=60)
policy = PolicyModel(input_dim=5, output_dim=3)
optimizer = optim.Adam(policy.parameters(), lr=1e-3, weight_decay=1e-4)
criterion = nn.MSELoss()

# Step 3: Pretrain the policy on the pre-collected data
print("Pretraining the policy...")
num_pretrain_epochs = 30
batch_size = 1024
train_dataset = torch.utils.data.TensorDataset(
    train_states_tensor, train_controls_tensor
)
train_loader = torch.utils.data.DataLoader(
    train_dataset, batch_size=batch_size, shuffle=True
)

for epoch in range(num_pretrain_epochs):
    for batch_states, batch_actions in train_loader:
        optimizer.zero_grad()
        predictions = policy(batch_states)
        loss = criterion(predictions, batch_actions)
        loss.backward()
        optimizer.step()

    print(f"Pretraining Epoch {epoch + 1}/{num_pretrain_epochs}, Loss: {loss.item()}")

# Save the pretrained model
torch.save(policy.state_dict(), os.path.join(MODELS_DIR, "pretrained_policy.pth"))
print("Pretrained policy saved to 'models/pretrained_policy.pth'.")

# Step 4: Prepare for DAgger
print("Starting DAgger iterations...")
dataset_states = train_states.tolist()  # Initialize with pre-collected states
dataset_actions = train_controls.tolist()  # Initialize with pre-collected controls

# Logging data for analysis
trajectory_logs = []  # To store state-action trajectories and differences

<<<<<<< HEAD
num_iterations = 10  # Number of DAgger iterations
=======
num_iterations = 15  # Number of DAgger iterations
>>>>>>> cc283d6fdfbbbdfa046669c571a33c9f1e9ed097
new_samples_per_iteration = 20000  # Limit the number of new samples per iteration

for iteration in range(num_iterations):
    print(f"Starting DAgger Iteration {iteration + 1}/{num_iterations}...")

    # Reset the environment and initialize logging
    state = env.reset()
    done = False
    iteration_trajectory = (
        []
    )  # Store state-action and expert-policy differences for this iteration
    samples_collected = 0

    # Compute blending factor (β)
    beta = 1 - (iteration / num_iterations)  # Linear decay from 1 to 0

    while samples_collected < new_samples_per_iteration:
        # Use the current policy to predict the action
        policy_action = (
            policy(torch.tensor(state, dtype=torch.float32)).detach().numpy()
        )

        # Query the expert for the correct action
        expert_action = expert_policy(state)

        # Blend policy and expert actions
        blended_action = beta * expert_action + (1 - beta) * policy_action

        # Compute the difference between blended and expert actions
        action_difference = np.linalg.norm(blended_action - expert_action)

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

        # Store the new state-action pair
        dataset_states.append(state.tolist())
        dataset_actions.append(expert_action.tolist())
        samples_collected += 1

        # Step the environment
        state, _, done = env.step(blended_action)

        # If the episode ends, reset the environment
        if done:
            state = env.reset()

    print(f"Collected {samples_collected} new state-action pairs.")

    # Log the trajectory for this iteration
    trajectory_logs.append(iteration_trajectory)

    # Train on the aggregated dataset
    print(f"Training policy on aggregated dataset after iteration {iteration + 1}...")
    aggregated_states_tensor = torch.tensor(dataset_states, dtype=torch.float32)
    aggregated_actions_tensor = torch.tensor(dataset_actions, dtype=torch.float32)

    train_dataset = torch.utils.data.TensorDataset(
        aggregated_states_tensor, aggregated_actions_tensor
    )
    train_loader = torch.utils.data.DataLoader(
        train_dataset, batch_size=batch_size, shuffle=True
    )

<<<<<<< HEAD
    for epoch in range(7):  # Train for 7 epochs per iteration
=======
    for epoch in range(20):  # Train for 7 epochs per iteration
>>>>>>> cc283d6fdfbbbdfa046669c571a33c9f1e9ed097
        for batch_states, batch_actions in train_loader:
            optimizer.zero_grad()
            predictions = policy(batch_states)
            loss = criterion(predictions, batch_actions)
            loss.backward()
            optimizer.step()

    print(f"DAgger Iteration {iteration + 1}/{num_iterations}, Loss: {loss.item()}")

    # Save the policy after each iteration
    torch.save(
        policy.state_dict(),
        os.path.join(MODELS_DIR, f"policy_iteration_{iteration + 1}.pth"),
    )
    print(
        f"Policy after iteration {iteration + 1} saved to 'models/policy_iteration_{iteration + 1}.pth'."
    )

# Save final policy
torch.save(policy.state_dict(), os.path.join(MODELS_DIR, "final_policy.pth"))
print("Final trained policy saved to 'models/final_policy.pth'.")

# Save logs for analysis
with open(os.path.join(LOG_DIR, "trajectory_logs.json"), "w") as f:
    json.dump(trajectory_logs, f)
print("Logs saved to 'logs/trajectory_logs.json'.")

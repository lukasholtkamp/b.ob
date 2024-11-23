from environment import SimpleEnvironment
from expert_policy import expert_policy
from policy_model import PolicyModel
import numpy as np
import os
import torch
import torch.nn as nn
import torch.optim as optim

# Paths for data and models
DATA_DIR = "data/"
MODELS_DIR = "models/"
os.makedirs(MODELS_DIR, exist_ok=True)

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
optimizer = optim.Adam(policy.parameters(), lr=1e-3, weight_decay=1e-4)  # Add weight decay for regularization
criterion = nn.MSELoss()

# Step 3: Pretrain the policy on the pre-collected data
print("Pretraining the policy...")
num_pretrain_epochs = 30
batch_size = 1024
train_dataset = torch.utils.data.TensorDataset(train_states_tensor, train_controls_tensor)
train_loader = torch.utils.data.DataLoader(train_dataset, batch_size=batch_size, shuffle=True)

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

num_iterations = 10  # Number of DAgger iterations
new_samples_per_iteration = 20000  # Limit the number of new samples per iteration

for iteration in range(num_iterations):
    print(f"Starting DAgger Iteration {iteration + 1}/{num_iterations}...")
    
    # Reset the environment and initialize collection variables
    state = env.reset()
    done = False
    iteration_states = []
    iteration_actions = []

    # Collect a fixed number of new samples (e.g., 20,000)
    samples_collected = 0

    while samples_collected < new_samples_per_iteration:
        # Use the current policy to predict the action
        action = policy(torch.tensor(state, dtype=torch.float32)).detach().numpy()

        # Query the expert for the correct action
        expert_action = expert_policy(state)

        # Store the new state-action pair
        iteration_states.append(state)
        iteration_actions.append(expert_action)
        samples_collected += 1

        # Step the environment
        state, _, done = env.step(action)

        # If the episode ends, reset the environment
        if done:
            state = env.reset()
            done = False  # Reset the flag

    print(f"Collected {samples_collected} new state-action pairs.")

    # Add the new samples to the aggregated dataset
    dataset_states.extend(iteration_states)
    dataset_actions.extend(iteration_actions)

    # Subset management: Keep only the most recent 200,000 samples
    if len(dataset_states) > 200000:
        dataset_states = dataset_states[-200000:]
        dataset_actions = dataset_actions[-200000:]

    # Train on the aggregated dataset
    print(f"Training policy on aggregated dataset after iteration {iteration + 1}...")
    aggregated_states_tensor = torch.tensor(dataset_states, dtype=torch.float32)
    aggregated_actions_tensor = torch.tensor(dataset_actions, dtype=torch.float32)

    train_dataset = torch.utils.data.TensorDataset(aggregated_states_tensor, aggregated_actions_tensor)
    train_loader = torch.utils.data.DataLoader(train_dataset, batch_size=batch_size, shuffle=True)

    for epoch in range(7):  # Train for 5 epochs per iteration
        for batch_states, batch_actions in train_loader:
            optimizer.zero_grad()
            predictions = policy(batch_states)
            loss = criterion(predictions, batch_actions)
            loss.backward()
            optimizer.step()

    print(f"DAgger Iteration {iteration + 1}/{num_iterations}, Loss: {loss.item()}")

    # Save the policy after each iteration
    torch.save(policy.state_dict(), os.path.join(MODELS_DIR, f"policy_iteration_{iteration + 1}.pth"))
    print(f"Policy after iteration {iteration + 1} saved to 'models/policy_iteration_{iteration + 1}.pth'.")

# Save the final policy
torch.save(policy.state_dict(), os.path.join(MODELS_DIR, "final_policy.pth"))
print("Final trained policy saved to 'models/final_policy.pth'.")

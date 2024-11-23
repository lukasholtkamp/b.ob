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

# Step 1: Load obstacle-aware pre-collected data
print("Loading pre-collected obstacle-aware data...")
train_states = np.load(os.path.join(DATA_DIR, "train_states_obs.npy"))  # State data including obstacles
train_controls = np.load(os.path.join(DATA_DIR, "train_controls_obs.npy"))  # Control data

# Convert data to PyTorch tensors
train_states_tensor = torch.tensor(train_states, dtype=torch.float32)
train_controls_tensor = torch.tensor(train_controls, dtype=torch.float32)

# Step 2: Initialize environment, policy model, and optimizer
print("Initializing environment and model...")
obstacle_count = (train_states.shape[1] - 5) // 3  # Compute number of obstacles from input dimension
policy = PolicyModel(input_dim=5 + obstacle_count * 3, output_dim=3)
optimizer = optim.Adam(policy.parameters(), lr=1e-3, weight_decay=1e-4)
criterion = nn.MSELoss()

env = SimpleEnvironment(dt=0.3, max_steps=60, obstacles=[
    (np.random.uniform(-2, 2), np.random.uniform(-2, 2), np.random.uniform(0.2, 0.5))
    for _ in range(obstacle_count)
])

# Step 3: Pretrain the policy on pre-collected data
print("Pretraining the policy...")
num_pretrain_epochs = 20
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
dataset_states = train_states.tolist()
dataset_actions = train_controls.tolist()

num_iterations = 10
new_samples_per_iteration = 20000

for iteration in range(num_iterations):
    print(f"Starting DAgger Iteration {iteration + 1}/{num_iterations}...")
    state = env.reset()
    done = False
    iteration_states = []
    iteration_actions = []
    samples_collected = 0

    while samples_collected < new_samples_per_iteration:
        action = policy(torch.tensor(state, dtype=torch.float32)).detach().numpy()
        expert_action = expert_policy(state)

        iteration_states.append(state)
        iteration_actions.append(expert_action)
        samples_collected += 1

        state, _, done = env.step(action)
        if done:
            state = env.reset()

    print(f"Collected {samples_collected} new state-action pairs.")

    dataset_states.extend(iteration_states)
    dataset_actions.extend(iteration_actions)

    if len(dataset_states) > 200000:
        dataset_states = dataset_states[-200000:]
        dataset_actions = dataset_actions[-200000:]

    aggregated_states_tensor = torch.tensor(dataset_states, dtype=torch.float32)
    aggregated_actions_tensor = torch.tensor(dataset_actions, dtype=torch.float32)
    train_dataset = torch.utils.data.TensorDataset(aggregated_states_tensor, aggregated_actions_tensor)
    train_loader = torch.utils.data.DataLoader(train_dataset, batch_size=batch_size, shuffle=True)

    for epoch in range(5):
        for batch_states, batch_actions in train_loader:
            optimizer.zero_grad()
            predictions = policy(batch_states)
            loss = criterion(predictions, batch_actions)
            loss.backward()
            optimizer.step()

    print(f"DAgger Iteration {iteration + 1}/{num_iterations}, Loss: {loss.item()}")
    torch.save(policy.state_dict(), os.path.join(MODELS_DIR, f"policy_iteration_{iteration + 1}.pth"))

torch.save(policy.state_dict(), os.path.join(MODELS_DIR, "final_policy.pth"))
print("Final trained policy saved to 'models/final_policy.pth'.")

from environment import SimpleEnvironment
from policy_model import PolicyModel
import torch
import os

# Paths for models
MODELS_DIR = "models/"
FINAL_MODEL_PATH = os.path.join(MODELS_DIR, "final_policy.pth")

# Load the trained policy
print("Loading trained policy...")
policy = PolicyModel(input_dim=5, output_dim=3)
policy.load_state_dict(torch.load(FINAL_MODEL_PATH))
policy.eval()  # Set policy to evaluation mode

# Test the policy
print("Testing the trained policy...")
env = SimpleEnvironment(dt=0.3, max_steps=60)
state = env.reset()
done = False
step = 0

while not done:
    state_tensor = torch.tensor(state, dtype=torch.float32)
    action = policy(state_tensor).detach().numpy()  # Use learned policy
    state, _, done = env.step(action)
    step += 1
    print(f"Step: {step}, State: {state}, Action: {action}")

print("Testing complete.")

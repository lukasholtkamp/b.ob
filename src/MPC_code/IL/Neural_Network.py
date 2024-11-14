import numpy as np
import tensorflow as tf
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Dense

# Load the saved training data
train_data = np.load("train_states.npy")   # Shape: (num_samples, 5)
train_labels = np.load("train_controls.npy")  # Shape: (num_samples, 3)

# Verify the shapes of the data
print("Train data shape:", train_data.shape)
print("Train labels shape:", train_labels.shape)

# Define the neural network architecture
model = Sequential([
    Dense(64, activation='relu', input_shape=(5,)),  # Input layer with 5 features, first hidden layer with 64 units
    Dense(64, activation='relu'),                    # Second hidden layer with 64 units
    Dense(64, activation='relu'),                    # Third hidden layer with 64 units
    Dense(3)                                         # Output layer with 3 units (s, omega, v), no activation for regression
])

# Compile the model with mean squared error loss and an optimizer
model.compile(optimizer='adam', loss='mean_squared_error')

# Print the model summary to verify parameter count
model.summary()

# Train the model using the loaded data
model.fit(train_data, train_labels, epochs=50, batch_size=32, validation_split=0.2)

# Save the trained model to a file
model.save("path_following_model.h5")

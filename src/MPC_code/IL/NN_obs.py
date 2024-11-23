import numpy as np
import tensorflow as tf
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Dense

# Load the saved training data
train_data = np.load("train_states_obs.npy")   # Shape: (num_samples, 5)
train_labels = np.load("train_controls_obs.npy")  # Shape: (num_samples, 3)

# Verify the shapes of the data
print("Train data shape:", train_data.shape)
print("Train labels shape:", train_labels.shape)

# Define the neural network architecture
model = Sequential([
    Dense(64, activation='relu', input_shape=(8,)),  # Input layer with 5 features, first hidden layer with 64 units
    Dense(64, activation='relu'),                    # Second hidden layer with 64 units
    Dense(64, activation='relu'),                    # Third hidden layer with 64 units
    Dense(3)                                         # Output layer with 3 units (s, omega, v), no activation for regression
])

# Compile the model with mean squared error loss and an optimizer
model.compile(optimizer='adam', loss='mean_squared_error')

# Print the model summary to verify parameter count
model.summary()

# Train the model
from tensorflow.keras.callbacks import EarlyStopping, ReduceLROnPlateau

early_stopping = EarlyStopping(monitor='val_loss', patience=20, restore_best_weights=True)
reduce_lr = ReduceLROnPlateau(monitor='val_loss', factor=0.5, patience=10, min_lr=1e-6)

history = model.fit(
    train_data, train_labels,
    epochs=350,
    batch_size=64,
    validation_split=0.2,
    callbacks=[reduce_lr]
)
# Save the trained model to a file
model.save("path_following_obs_avoidance_model.h5")
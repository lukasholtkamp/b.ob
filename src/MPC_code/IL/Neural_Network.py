import tensorflow as tf
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Dense



# Define the neural network architecture
model = Sequential([
    Dense(64, activation='relu', input_shape=(5,)),  # Input layer with 5 features, first hidden layer with 64 units
    Dense(64, activation='relu'),                    # Second hidden layer with 64 units
    Dense(64, activation='relu'),                    # Third hidden layer with 64 units
    Dense(3)                                         # Output layer with 3 units (s, omega, v), no activation for regression
])

# Compile the model with mean squared error loss and an optimizer
model.compile(optimizer='adam', loss='mse')

# Print the model summary to verify parameter count
model.summary()


# Assuming train_data and train_labels are prepared datasets
# train_data shape: (num_samples, 5)
# train_labels shape: (num_samples, 3)

# Train the model
# model.fit(train_data, train_labels, epochs=50, batch_size=32, validation_split=0.2)

import tensorflow as tf
from tensorflow.keras.layers import Layer
import numpy as np

class CollisionPenaltyLayer(Layer):
    def __init__(self, safety_margin=0.41, base_weight=0.1, max_weight=0.5, **kwargs):
        super(CollisionPenaltyLayer, self).__init__(**kwargs)
        self.safety_margin = safety_margin
        self.base_weight = base_weight
        self.max_weight = max_weight

    def call(self, inputs):
        # Extract input components
        x, y = inputs[:, 0], inputs[:, 1]
        obs_x, obs_y, obs_r = inputs[:, 5], inputs[:, 6], inputs[:, 7]

        # Compute squared distance to the obstacle
        dist_to_obs_sq = (x - obs_x)**2 + (y - obs_y)**2
        safety_margin_sq = (obs_r + self.safety_margin) ** 2

        # Compute collision penalty
        collision_penalty = tf.nn.relu(safety_margin_sq - dist_to_obs_sq)

        # Compute dynamic weight
        dynamic_weight = self.base_weight + (self.max_weight - self.base_weight) * tf.exp(-dist_to_obs_sq / (2 * safety_margin_sq))
        dynamic_weight = tf.minimum(dynamic_weight, self.max_weight)  # Cap the weight

        # Scale the penalty
        scaled_penalty = dynamic_weight * collision_penalty

        # Return the mean penalty
        return tf.reduce_mean(scaled_penalty)


from tensorflow.keras.models import Model
from tensorflow.keras.layers import Input, Dense, Multiply

# Define input layer
input_data = Input(shape=(8,))

# Attention mechanism
attention_scores = Dense(8, activation='softmax', name="Attention_Scores")(input_data)
attended_data = Multiply(name="Attended_Features")([input_data, attention_scores])

# Feedforward layers
x = Dense(64, activation='relu')(attended_data)
x = Dense(64, activation='relu')(x)
x = Dense(64, activation='relu')(x)
outputs = Dense(3)(x)  # Outputs: linear velocity, angular velocity, path parameter rate

# Collision penalty computation
collision_penalty = CollisionPenaltyLayer()(input_data)

# Create the model
model = Model(inputs=input_data, outputs=[outputs, collision_penalty])


def custom_loss(y_true, y_pred):
    # Cast tensors to float32 for consistency
    y_true = tf.cast(y_true, tf.float32)
    y_pred = tf.cast(y_pred, tf.float32)
    train_data_float32 = tf.cast(train_data, tf.float32)

    # Compute imitation loss (Huber loss)
    imitation_loss = tf.keras.losses.Huber()(y_true, y_pred)

    # Extract obstacle-related information from the input data
    x, y = train_data_float32[:, 0], train_data_float32[:, 1]
    obs_x, obs_y, obs_r = train_data_float32[:, 5], train_data_float32[:, 6], train_data_float32[:, 7]

    # Compute collision penalty
    safety_margin_sq = (obs_r + 0.23 + 0.18) ** 2
    dist_to_obs_sq = (x - obs_x) ** 2 + (y - obs_y) ** 2
    collision_penalty = tf.reduce_mean(tf.nn.relu(safety_margin_sq - dist_to_obs_sq))

    # Compute dynamic penalty weight with a cap
    base_weight = 0.1  # Minimum weight when far from obstacles
    max_weight = 0.5   # Maximum weight near obstacles
    dynamic_weight = base_weight + (max_weight - base_weight) * tf.exp(-dist_to_obs_sq / (2 * safety_margin_sq))
    dynamic_weight = tf.minimum(dynamic_weight, max_weight)  # Cap the weight

    # Combine losses
    total_loss = imitation_loss + dynamic_weight * collision_penalty
    return total_loss



from tensorflow.keras.optimizers import Adam
from tensorflow.keras.callbacks import EarlyStopping, ReduceLROnPlateau

# Load your data
train_data = np.load("train_states_obs.npy")  # Shape: (num_samples, 8)
train_labels = np.load("train_controls_obs.npy")  # Shape: (num_samples, 3)

# Prepare labels for the model
train_labels_with_penalty = [train_labels, np.zeros((train_data.shape[0],))]

# Callbacks
early_stopping = EarlyStopping(monitor='val_loss', patience=20, restore_best_weights=True)
reduce_lr = ReduceLROnPlateau(monitor='val_loss', factor=0.5, patience=10, min_lr=1e-6)

model.compile(
    optimizer=Adam(learning_rate=0.001),
    loss=lambda y_true, y_pred: custom_loss(y_true, y_pred)
)

# Train the model
history = model.fit(
    train_data,
    train_labels,  # Only the control commands (v, omega, s_dot)
    epochs=300,
    batch_size=64,
    validation_split=0.2,
    callbacks=[early_stopping, reduce_lr]
)

# Save the model using the new `.keras` format
model.save("/home/bertrandt/b.ob/src/MPC_code/IL/path_following_obs_avoidance_with_penalty.keras")

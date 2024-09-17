import matplotlib.pyplot as plt
import numpy as np

# Create some test data
x = np.linspace(0, 10, 100)
y = np.sin(x)

# Create a figure and axis
fig, ax = plt.subplots()

# Plot the data
ax.plot(x, y, label='Sine wave')

# Add a title and labels
ax.set_title('Test Plot')
ax.set_xlabel('X axis')
ax.set_ylabel('Y axis')

# Add a legend
ax.legend()

# Display the plot in a window
plt.show()


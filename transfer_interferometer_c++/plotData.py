import matplotlib.pyplot as plt

data = []
with open('test_data.txt', 'r') as file:
    for line in file:
        data.append(float(line.strip()))

plt.plot(data)
plt.xlabel('X')
plt.ylabel('Y')
plt.grid(True)
plt.show()

import matplotlib.pyplot as plt

X1 = [2.132, 3.608, 5.083, 6.588, 8.109, 9.616, 11.109]
Y1 = [10.3, 6.9, 5, 3.6, 2.6, 2.2, 1.4]

X2 = [1.969, 3.432, 4.882, 6.368, 7.788, 9.197, 10.67, 12.07, 13.526]
Y2 = [13.5, 11.5, 9.8, 8.4, 7.3, 6.5, 5.8, 5.3, 4.8]

X3 = [3.982, 5.361, 6.938, 8.581]
Y3 = [6.8, 3.5, 1, 0.5]

plt.plot(X1, Y1, color = "orange", label = "impulse input for Hg")
plt.plot(X2, Y2, color = "g", label = "step input for Hg")
plt.plot(X3, Y3, color = "b", label = "impulse input for water")

plt.ylabel("Pressure (mm)")
plt.xlabel("time (s)")
plt.title("Pressure vs Time")

plt.legend()
plt.show()
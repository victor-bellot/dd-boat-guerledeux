import numpy as np
import matplotlib.pyplot as plt

fig = plt.figure()
names = ["test_acc_vitnulle.npy", "test_acc_vroum.npy", "test_acc_vroum_filtr.npy"]
for i in range(3):
    fig.add_subplot(3, 1, i+1)
    try:
        measurements = np.load(names[i])
    except:
        print("Les fichiers test_acc n'existent pas")
        break
    x = measurements[0, :]
    y = measurements[1, :]
    z = measurements[2, :]

    plt.plot(x, 'red')
    plt.plot(y, 'orange')
    plt.plot(z, 'pink')
    plt.title(names[i])
plt.show()

import matplotlib
matplotlib.use('QtAgg')
import matplotlib.pyplot as plt

import matplotlib.pyplot as plt

def on_key(event):
    print(event.key)

fig = plt.figure()
fig.canvas.mpl_connect('key_press_event', on_key)
plt.show()
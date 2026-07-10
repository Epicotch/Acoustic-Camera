import numpy as np
from matplotlib import pyplot as plt

with open("dump.txt", "r") as f:
    l = f.readlines()

datastr = ""

for i in range(len(l)):
    if l[i].strip() == "0x":
        datastr += l[i + 2].strip()

text = [datastr[i:i+4] for i in range(0, len(datastr), 4)]
vals = []
for i in text:
    vals.append(int.from_bytes(bytes.fromhex(i), byteorder="little", signed=True))

fft = np.fft.rfft(vals)
freq = np.arange(len(fft)) * 48000 / 1024

domain = np.arange(len(vals)) / 48000

plt.plot(freq, abs(fft))
# plt.plot(domain, vals)
plt.show()
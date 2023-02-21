import matplotlib.pyplot as plt
import numpy as np

df = open('data.txt', 'r')
lines = df.readlines()

pitch = []
roll  = []

for line in lines:
    temp = line.split('|')
    pitch.append( float(temp[0].split(':')[1]) )
    roll.append ( float(temp[1].split(':')[1]) )

pitch_fft = np.fft.fft(pitch)
roll_fft  = np.fft.fft(roll)

# lowpass
print(len(pitch_fft))
pitch_fft_lp = pitch_fft
roll_fft_lp = roll_fft
pitch_fft_lp[-150:] = np.zeros(150)
roll_fft_lp[-150:] = np.zeros(150)

# ifft
pitch_ifft = np.fft.ifft(pitch_fft_lp)
roll_ifft = np.fft.ifft(roll_fft_lp)


plt.subplot(211)
plt.title('original signal')
plt.plot(pitch, label='pitch')
plt.plot(roll, label='roll')
plt.legend(loc='upper right')
plt.xlabel('time(ms)')
plt.ylabel('signal')
plt.subplot(212)
plt.title('lowpass signal')
plt.plot(pitch_ifft, label='pitch')
plt.plot(roll_ifft, label='roll')
plt.legend(loc='upper right')
plt.xlabel('time(ms)')
plt.ylabel('signal')
plt.show()


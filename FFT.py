import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.fft import fft, fftfreq
from scipy.signal import welch,spectrogram

# === 1. Load CSV ===
file_path = "simulated_relay_noise.csv"  
df = pd.read_csv(file_path)
df.columns = df.columns.str.strip()  
time_col = df.columns[0]
volt_col = df.columns[1]

# === 2. Extract data ===
time_s = df[time_col].values
voltage_v = df[volt_col].values

# === 3. Sampling Parameters ===
dt = np.mean(np.diff(time_s))      # คำนวณช่วงเวลาเฉลี่ยระหว่างจุด (s)
fs = 1 / dt                        # Sampling frequency (Hz)
N = len(voltage_v)                 # จำนวน sample
nyquist = fs / 2                  # Nyquist frequency (Hz)
freq_resolution = fs / N          # ความละเอียดของความถี่

# === Print summary ===
print("=== Signal Parameters Summary ===")
print(f"Sample (N): {N}")
print(f"delta(t) (avg): {dt:.6e} seconds")
print(f"Sampling Frequency (fs): {fs:.2f} Hz")
print(f"Nyquist Frequency: {nyquist:.2f} Hz")
print(f"Frequency Resolution: {freq_resolution:.2f} Hz")

# === 4. Plot Time Domain (Original Signal) ===
plt.figure(figsize=(10, 4))
plt.plot(time_s, voltage_v, color='orange')
plt.title("Original Signal (Time Domain)")
plt.xlabel("Time (s)")
plt.ylabel("Voltage (V)")
plt.grid(True)
plt.tight_layout()

# === 4. Perform FFT ===
fft_vals = fft(voltage_v)
fft_freqs = fftfreq(N, dt)
positive_freqs = fft_freqs[:N // 2]
positive_magnitude = 2.0 / N * np.abs(fft_vals[:N // 2])  # ขนาดสเปกตรัม

# === 5. Plot FFT ===
plt.figure("Frequency Domain (FFT)", figsize=(10, 5))
plt.plot(positive_freqs, positive_magnitude)
plt.title("FFT of Signal (Magnitude Spectrum)")
plt.xlabel("Frequency (Hz)")
plt.ylabel("Amplitude")
plt.xlim(0, nyquist)
plt.grid(True)
plt.tight_layout()

f_welch, psd = welch(voltage_v, fs=fs, nperseg=1024)

plt.figure("Power Spectral Density (Welch)", figsize=(10, 5))
plt.semilogy(f_welch, psd)
plt.title("Power Spectral Density (Welch Method)")
plt.xlabel("Frequency (Hz)")
plt.ylabel("Power/Frequency (V²/Hz)")
plt.grid(True)
plt.tight_layout()


# === 7. Plot STFT Spectrogram ===
f_stft, t_stft, Sxx = spectrogram(voltage_v, fs=fs, nperseg=1024, noverlap=512)

plt.figure("STFT Spectrogram", figsize=(10, 5))
plt.pcolormesh(t_stft, f_stft, 10 * np.log10(Sxx), shading='gouraud', cmap='inferno')
plt.title("STFT Spectrogram")
plt.ylabel("Frequency (Hz)")
plt.xlabel("Time (s)")
plt.colorbar(label='Power (dB)')
plt.ylim(0, nyquist)
plt.tight_layout()
plt.show()
# === 6. Optional: Save/Print Top Frequencies ===
fft_df = pd.DataFrame({
    "Frequency (Hz)": positive_freqs,
    "Amplitude": positive_magnitude
})
top_peaks = fft_df.sort_values("Amplitude", ascending=False).head(10)
#print("\n=== Top 10 Peak Frequencies ===")
#print(top_peaks)

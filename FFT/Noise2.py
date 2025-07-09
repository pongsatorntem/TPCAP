import numpy as np
import cv2
import matplotlib.pyplot as plt
from scipy.fft import fft, fftfreq
from scipy.signal import spectrogram, butter, filtfilt
from scipy.interpolate import interp1d
from emd.sift import sift

# === Bandpass Filter Function ===
def bandpass_filter(data, lowcut, highcut, fs, order=4):
    nyq = 0.5 * fs
    low = lowcut / nyq
    high = highcut / nyq
    b, a = butter(order, [low, high], btype='band')
    return filtfilt(b, a, data)

trace_color = "yellow"   # → หรือ "blue"
image_path = r"D:\WORK_PYTHON\NOISE\FFT\r100c100pf.PNG"
fs = 20000  # Sampling frequency (Hz)
normalize_v = 3.3

# === Load and Extract Trace from Image ===
img = cv2.imread(image_path)
if img is None:
    raise FileNotFoundError(f"❌ ไม่พบไฟล์ภาพ: {image_path}")

hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
if trace_color == "yellow":
    lower, upper = np.array([20, 100, 100]), np.array([40, 255, 255])
elif trace_color == "blue":
    lower, upper = np.array([90, 100, 100]), np.array([130, 255, 255])
else:
    raise ValueError("trace_color ต้องเป็น 'yellow' หรือ 'blue'")

mask = cv2.inRange(hsv, lower, upper)
points = np.column_stack(np.where(mask > 0))
x_vals = points[:, 1]
y_vals = img.shape[0] - points[:, 0]

x_unique = np.unique(x_vals)
y_interp_raw = [np.mean(y_vals[x_vals == x]) for x in x_unique]
interp_func = interp1d(x_unique, y_interp_raw, kind='cubic')
new_x = np.linspace(x_unique.min(), x_unique.max(), fs)
y_interp = interp_func(new_x)

# === Normalize Voltage ===
voltage_v = (y_interp - np.min(y_interp)) / (np.max(y_interp) - np.min(y_interp)) * normalize_v
N = len(voltage_v)
dt = 1 / fs
time_s = np.arange(N) * dt

# === Summary Print ===
print("=== Signal Parameters Summary ===")
print(f"Sample (N): {N}")
print(f"delta(t): {dt:.6e} seconds")
print(f"Sampling Frequency: {fs:.2f} Hz")
print(f"Nyquist Frequency: {fs/2:.2f} Hz")
print(f"Frequency Resolution: {fs/N:.2f} Hz")

# === Plot Time-Domain Signal ===
plt.figure("Time Domain", figsize=(10, 4))
plt.plot(time_s, voltage_v, color='orange')
plt.title("Original Signal (Time Domain)")
plt.xlabel("Time (s)")
plt.ylabel("Voltage (V)")
plt.grid(True)
plt.tight_layout()

# === Optional Bandpass Filter (Enable if needed) ===
# voltage_v = bandpass_filter(voltage_v, 1000, 4500, fs)




# === Step 2: FFT ===
f_fft = fftfreq(N, dt)
fft_values = np.abs(fft(voltage_v))
plt.figure(figsize=(10, 4))
plt.plot(f_fft[:N//2], fft_values[:N//2])
plt.title("📊 Full FFT Spectrum")
plt.xlabel("Frequency [Hz]")
plt.ylabel("Magnitude")
plt.grid(True)
plt.tight_layout()
plt.show()

# === Step 3: EMD ===
IMFs = sift(voltage_v)

plt.figure(figsize=(10, 2 * len(IMFs)))
for i, imf in enumerate(IMFs):
    plt.subplot(len(IMFs), 1, i+1)
    plt.plot(time_s, imf)
    plt.title(f"IMF {i+1}")
plt.xlabel("Time [s]")
plt.tight_layout()
plt.show()

# === Step 4: FFT of each IMF ===
plt.figure(figsize=(12, 2 * len(IMFs)))
for i, imf in enumerate(IMFs):
    imf_fft = np.abs(fft(imf))
    plt.subplot(len(IMFs), 1, i+1)
    plt.plot(f_fft[:N//2], imf_fft[:N//2])
    plt.title(f"FFT of IMF {i+1}")
    plt.xlabel("Frequency [Hz]")
    plt.grid(True)
plt.tight_layout()
plt.show()

# === Step 5: STFT Spectrogram ===
f, t, Sxx = spectrogram(voltage_v, fs=fs, nperseg=512, noverlap=384)
plt.figure(figsize=(10, 4))
plt.pcolormesh(t, f, 10*np.log10(Sxx + 1e-12), shading='gouraud', cmap='inferno')
plt.colorbar(label='Power [dB]')
plt.title("📈 STFT Spectrogram")
plt.xlabel("Time [s]")
plt.ylabel("Frequency [Hz]")
plt.tight_layout()
plt.show()

# === Step 6: RC Filter Design ===
target_noise_freq = 12000  # Hz
C = 0.1e-6  # Farads (0.1 µF)
R = 1 / (2 * np.pi * target_noise_freq * C)

print(f"\n🎯 Design RC Low-pass Filter for {target_noise_freq/1000:.1f} kHz noise:")
print(f" - Recommended C: {C*1e6:.1f} µF")
print(f" - Calculated R: {R:.1f} ohms")
print(f" - RC Cutoff freq = {1/(2*np.pi*R*C):.1f} Hz")
print(f" - 🛠 Suggestion: Add ferrite bead on signal/GND/Vcc lines for high-frequency spike suppression.")
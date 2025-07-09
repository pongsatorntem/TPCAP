import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.fft import fft, fftfreq, ifft
from scipy.signal import welch,spectrogram,butter, filtfilt,savgol_filter
import cv2
from scipy.interpolate import interp1d


def bandpass_filter(data, lowcut, highcut, fs, order=4):
    nyq = 0.5 * fs
    low = lowcut / nyq
    high = highcut / nyq
    b, a = butter(order, [low, high], btype='band')
    return filtfilt(b, a, data)

trace_color = "yellow"   # → เปลี่ยนเป็น "blue" ถ้าจะเลือกเส้นสีฟ้า
# === Load waveform from image ===
image_path = r"D:\WORK_PYTHON\NOISE\FFT\r100c100pf.PNG"
fs = 20000  # Sampling frequency (Hz)
normalize_v = 3.3

# ========== [1] Load image and extract trace ==========
img = cv2.imread(image_path)
if img is None:
    raise FileNotFoundError(f"❌ ไม่พบไฟล์ภาพ: {image_path}")

hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

if trace_color == "yellow":
    lower = np.array([20, 100, 100])
    upper = np.array([40, 255, 255])
elif trace_color == "blue":
    lower = np.array([90, 100, 100])
    upper = np.array([130, 255, 255])
else:
    raise ValueError("trace_color ต้องเป็น 'yellow' หรือ 'blue'")

mask = cv2.inRange(hsv, lower, upper)
points = np.column_stack(np.where(mask > 0))
x_vals = points[:, 1]
y_vals = img.shape[0] - points[:, 0]  # flip vertically

# Interpolate y ตาม x
x_unique = np.unique(x_vals)
y_interp_raw = [np.mean(y_vals[x_vals == x]) for x in x_unique]
# สร้าง interpolation function (linear/cubic ก็ได้)
interp_func = interp1d(x_unique, y_interp_raw, kind='cubic')
# กำหนดจำนวนจุดใหม่ให้ละเอียดขึ้น (เช่น 2000 จุด)
new_x = np.linspace(x_unique.min(), x_unique.max(), fs)
y_interp = interp_func(new_x)




# Normalize y ให้เป็นโวลต์ (0–3.3V)
voltage_v = np.array(y_interp)
voltage_v = (voltage_v - voltage_v.min()) / (voltage_v.max() - voltage_v.min()) * normalize_v


# สร้างแกนเวลา
N = len(voltage_v)
dt = 1 / fs
time_s = np.arange(N) * dt




nyquist = fs / 2                  # Nyquist frequency (Hz)
freq_resolution = fs / N          # ความละเอียดของความถี่

# === Bandpass Filter เพื่อดูเฉพาะช่วง noise (เช่น 1000–4500 Hz)
# filtered_v = bandpass_filter(voltage_v, 1000, 4500, fs)
filtered_v = voltage_v


# แสดงให้ดูว่าหลัง filter เหลือแค่ noise
plt.figure("Filtered Noise", figsize=(10, 4))
plt.plot(time_s, filtered_v, color='red')
plt.title("Filtered High-Frequency Noise (1000–4500 Hz)")
plt.xlabel("Time (s)")
plt.ylabel("Voltage (V)")
plt.grid(True)
plt.tight_layout()


#------------------------------------------------------------------------------------------------------------
# === 1. Load CSV ===


# # file_path = "0.00001.csv"  
# # df = pd.read_csv(file_path)
# # df.columns = df.columns.str.strip()  
# # time_col = df.columns[0]
# # volt_col = df.columns[1]

# # # === 2. Extract data ===
# # time_s = df[time_col].values
# # voltage_v = df[volt_col].values

# # # === 3. Sampling Parameters ===
# # dt = np.mean(np.diff(time_s))      # คำนวณช่วงเวลาเฉลี่ยระหว่างจุด (s)
# # fs = 1 / dt                        # Sampling frequency (Hz)
# # N = len(voltage_v)                 # จำนวน sample


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
fft_vals = fft(filtered_v)
# fft_vals = fft(voltage_v)

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

f_welch, psd = welch(filtered_v, fs=fs, nperseg=512)
# f_welch, psd = welch(voltage_v, fs=fs, nperseg=1024)

plt.figure("Power Spectral Density (Welch)", figsize=(10, 5))
plt.semilogy(f_welch, psd)
plt.title("Power Spectral Density (Welch Method)")
plt.xlabel("Frequency (Hz)")
plt.ylabel("Power/Frequency (V²/Hz)")
plt.grid(True)
plt.tight_layout()


# === 7. Plot STFT Spectrogram ===

# f_stft, t_stft, Sxx = spectrogram(voltage_v, fs=fs, nperseg=1024, noverlap=512)
f_stft, t_stft, Sxx = spectrogram(filtered_v, fs=fs, nperseg=256, noverlap=128)
# f_stft, t_stft, Sxx = spectrogram(voltage_v, fs=fs, nperseg=256, noverlap=128)

# ป้องกัน log(0)
Sxx[Sxx == 0] = 1e-20

plt.figure("STFT Spectrogram", figsize=(10, 5))
plt.pcolormesh(t_stft, f_stft, 10 * np.log10(Sxx), shading='gouraud', cmap='inferno')
plt.title("STFT Spectrogram")
plt.ylabel("Frequency (Hz)")
plt.xlabel("Time (s)")
plt.colorbar(label='Power (dB)')
plt.ylim(0, nyquist)
plt.tight_layout()
# === 6. Optional: Save/Print Top Frequencies ===
fft_df = pd.DataFrame({
    "Frequency (Hz)": positive_freqs,
    "Amplitude": positive_magnitude
})
top_peaks = fft_df.sort_values("Amplitude", ascending=False).head(10)
#print("\n=== Top 10 Peak Frequencies ===")
#print(top_peaks)



#/////////////////////////////////////////////////////////////////// noise filter and invert to T-domain

# === Butterworth Low-Pass Filter ===
def lowpass_filter(data, cutoff, fs, order=4):
    nyq = 0.5 * fs
    norm_cutoff = cutoff / nyq
    b, a = butter(order, norm_cutoff, btype='low')
    return filtfilt(b, a, data)


sg_filtered = savgol_filter(voltage_v, window_length=51, polyorder=2)
# plt.figure("Savitzky-Golay Filter", figsize=(10, 4))
# plt.plot(time_s, voltage_v, label='Original', alpha=0.5)
# plt.plot(time_s, sg_filtered, label='Savitzky-Golay', color='green')
# plt.title("Savitzky-Golay Filter Result")
# plt.xlabel("Time (s)")
# plt.ylabel("Voltage (V)")
# plt.legend()
# plt.grid(True)
# plt.tight_layout()

# กรองที่ 500 Hz
lpf_filtered = lowpass_filter(voltage_v, cutoff=50, fs=fs)
plt.figure("Low-Pass Filter (Time Domain)", figsize=(10, 4))
plt.plot(time_s, voltage_v, label="Original", color='gray')
plt.plot(time_s, lpf_filtered, label="Low-Pass < ....Hz", color='blue')
plt.title("Low-Pass Filter Result")
plt.xlabel("Time (s)")
plt.ylabel("Voltage (V)")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()

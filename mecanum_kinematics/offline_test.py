import numpy as np

print("=== ENRO MECANUM ROBOT - OFFLINE KINEMATICS TEST (SECTION 7.1) ===")

# Tekerlek Yarıçapı
R = 0.1625

Lx = 0.4    

# Gövde 0.7m genişliğinde. Merkezden uzaklık (Half Width)
# wheel_offset_y = 0.35 + (0.09/2) = 0.395
Ly = 0.395  

# Mecanum Geometrik Toplamı
K = Lx + Ly

print(f"Parametreler: R={R}, Lx={Lx}, Ly={Ly}, K={K}")
print("-" * 50)

# 2. MATRİSLER (Standart Mecanum Formülleri)
# IK: Twist (vx, vy, w) -> Tekerlek Hızları
# Sütunlar: vx, vy, wz
# Satırlar: FL, FR, RL, RR
IK_Matrix = (1/R) * np.array([
    [1, -1, -K], # FL
    [1,  1,  K], # FR
    [1,  1, -K], # RL
    [1, -1,  K]  # RR
])

# FK: Tekerlek Hızları -> Twist
# (Inverse Kinematics'in tersi)
FK_Matrix = (R/4) * np.array([
    [1, 1, 1, 1],          # vx
    [-1, 1, 1, -1],        # vy
    [-1/K, 1/K, -1/K, 1/K] # wz
])

# 3. TEST SENARYOLARI
test_cases = [
    {"name": "ILERI GITME", "input": np.array([1.0, 0.0, 0.0])},
    {"name": "YENGEC (SOLA)", "input": np.array([0.0, 1.0, 0.0])}, # Vy testi
    {"name": "DONME (SAAT YONU)", "input": np.array([0.0, 0.0, -1.0])},
    {"name": "KARISIK HAREKET", "input": np.array([0.5, -0.2, 0.1])}
]

for case in test_cases:
    print(f"\nTEST: {case['name']}")
    target_twist = case['input']
    print(f"  1. Hedef Hız (Twist): {target_twist}")

    # Adım A: Inverse Kinematics (Hız -> Tekerlek)
    wheel_speeds = IK_Matrix @ target_twist
    print(f"  2. Hesaplanan Tekerlek Hızları: {np.round(wheel_speeds, 3)}")

    # Adım B: Forward Kinematics (Tekerlek -> Hız)
    recovered_twist = FK_Matrix @ wheel_speeds
    print(f"  3. Geri Dönüştürülen Hız (FK): {np.round(recovered_twist, 3)}")

    # Adım C: Hata Kontrolü (Fark 0'a çok yakınsa başarılıdır)
    error = np.abs(target_twist - recovered_twist)
    if np.all(error < 1e-6):
        print("  ✅ SONUÇ: BAŞARILI (Hata ~ 0)")
    else:
        print(f"  ❌ SONUÇ: BAŞARISIZ (Hata: {error})")

print("\n" + "="*50)
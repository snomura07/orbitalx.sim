## 技術スタック
- C++17
- ビルド: CMake
- 開発/実行環境：docker + docker-compose
- Dockerイメージはubuntu24以上とすること

## フォルダ構成（例）
docs/
  codex.md #このファイル
　README.md #実装内容詳細
docker/
　Dockerfile
  docker-compose.yml
src/
  core/
    plant/
      battery/
        Battery.h
        Battery.cpp
      motor/
        Motor.h
        Motor.cpp
      cpu/
        Cpu.h
        Cpu.cpp
  app/
    main.cpp
CMakeLists.txt

## 目的
調簡易的なロボトレースシミュレータをC++で実装する。
- 機体はモータ2個、タイヤ二個の構成とする（ロボトレース競技でよく使用される車体構成）
- バッテリは1sが3.7Vのバッテリーを２S直列接続で使用する
- 機体は無限に続く平面な直線を直進する
- ロボットにはロータリエンコーダとバッテリ監視センサがついているので、現在の速度とバッテリ残量が分かる
- 10msごとの車体速度/角速度、バッテリー残量、モータへの印加電圧/電流をコンソール画面に表示する
- 以下はパラメータとして外出し、開始時に読み込んで使用する
  - 最高速度[mm/s] : 1000 mm/s
  - 加速度[mm/ss] : 3000mm/ss
  - バッテリ電圧初期値[V] : 8.3V
  - タイヤ径[mm] : 24mm
  - ギヤ比(モータ車軸：タイヤ軸) : 20:64
  - 機体重量[g] : 125g
  - ホイールベース[mm]： 198mm

---

## 📏 型定義
### Pose
- x [mm]
- y [mm]
- theta [rad]

### VehicleState
- pose
- v [mm/s]
- omega [rad/s]
- vL [mm/s], vR [mm/s]
- dutyL, dutyR
- Vbat, VmotL, VmotR
- lateralError e

---

## 🤖 車体モデル（差動二輪・運動学）

### 運動学
v = (vR + vL)/2
omega = (vR - vL)/trackWidth

x += v * cos(theta) * dt
y += v * sin(theta) * dt
theta += omega * dt

---

## PWM制御
### 条件
- 実装上のPWM最大値は1500とし、算出したPWM値をこれで割ることでDutyを求める。
- Dutyにその時のバッテリ残量を掛けることで電圧に変換し、モータへの印加電圧とする。
duty = pwm / PWM_MAX
V_motor = duty * V_batt

---

## 🔋 電源モデル
### 条件
- バッテリーは Lipoバッテリー3.7V セル ×2直列（2S）
- 容量は350mAh
- 初期電圧 Vbat_init = 8.4 V（満充電相当）
- 最低電圧 Vbat_min = 6.0 V（簡易の下限）
- モータ電流からバッテリー消費(SoC)と電圧低下(Vbatt)を再現

### 因果の整理
- A) 制御（Controller）
  - 入力：`v_ref`（目標速度）, `v`（実速度）
  - 出力：`duty`（0..1）

- B) 物理（Plant：モータ＋ギヤ＋車体）
  - 入力：`duty`, `Vbatt`
  - 出力：`v`, `I_motor`（モータ電流）

- C) 電源（Battery）
  - 入力：`I_total`
  - 出力：`SoC`, `Vbatt`

- 制御の順：目標→電圧指令
- 物理・電源の順：電圧→電流→トルク→速度 / 電流→SoC→電圧

### 物理式（最小モデル）
- DCモータ（平均PWM）
  - `V_motor = duty * Vbatt`
  - `V_bemf = n_motor / kn`  （n_motor[rpm], kn[rpm/V]）
  - `I_motor = (V_motor - V_bemf) / R`
  - `τ_motor = Kt * (I_motor - I0)`

- ギヤ
  - `G = 64/20 = 3.2`
  - `n_motor = n_wheel * G`
  - `τ_wheel = τ_motor * G * η_gear`

- 車体
  - `F_drive = τ_wheel / r`
  - `F_resist = F0 + kv*v + k2*v^2`
  - `a = (F_drive - F_resist)/m`

- バッテリー
  - `SoC` はクーロン積算：`SoC -= (I_total*dt)/C_as`
  - 端子電圧：`Vbatt = OCV(SoC) - I_total*R_internal`

### モータ定数（データシートから入れる想定）
- `R` [Ω]（巻線抵抗）: 2.83
- `Kt` [N·m/A]（トルク定数）: 5.33
- `kn` [rpm/V]（回転数定数）: 1790
- `I0` [A]（無負荷電流）: 0.0226
- `C_as`（容量：Ah→A·s 変換：`Ah * 3600`）: 0.35Ah

### 推定パラメータ（まず動かすための初期値）
- Battery
  - `V_full = 8.4 [V]`
  - `V_empty = 6.4 [V]`
  - `OCV(SoC) = V_empty + (V_full - V_empty) * SoC`
  - `R_internal = 0.20 [Ω]`（電圧ドロップ用）
- Drivetrain / Resist
  - `η_gear = 0.80`
  - `F_resist = F0 + kv*v`（まずはk2=0でOK）
  - `F0 = 0.10 [N]`
  - `kv = 0.20 [N/(m/s)]`
  - `k2 = 0`
- Electronics
  - `I_mcu = 0.05 [A]`（定数でOK）


## ⚙️ モータモデル（簡易・一次遅れ）

モータは「平均印加電圧」を入力として、車輪速度（または車輪線速度）を出す簡易モデル。

wheelSpeed += (K * Vmot - wheelSpeed) * (dt / tau)

### パラメータ（初期値）
- K = 2.0  （V→[m/s] の係数。最終的に1m/sが出るよう調整する）
- tau = 0.05 [s]

※ Kの単位系は簡易。実機パラメータは後で合わせる。

---

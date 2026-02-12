# METU ALAZ Autonomous

## Setup

1. **Docker'ı kur**: Sisteminize Docker'ı indirip kurun
   - https://www.docker.com/get-started/

2. **Repoyu klonla**:
   ```bash
   git clone https://github.com/efekaanguler/alaz.git
   cd alaz
   ```

3. **Başlatma scriptini çalıştır**:
   ```bash
   ./repo_init.sh
   ```
## Running Autoware Container

```bash
./dev_run.sh
```
## Using Simulation

/sim altındaki .md dosyalarını incele

## Build

Paketleri derlemek için:

```bash
# Sadece sim_pkg paketini derle
./build_simpkg.sh

# modules/ altındaki tüm paketleri derle
./build_modules.sh

# Temiz derleme için --clean ekle
# Tüm paketler silinir ve yeniden buildlenir
./build_modules.sh --clean
```

Derleme sonrası paketleri kullanmak için:

```bash
source /opt/ros/humble/setup.bash
source ./install/setup.bash
```

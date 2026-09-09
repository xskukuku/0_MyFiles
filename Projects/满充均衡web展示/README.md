# 满充均衡Web演示程序

## 项目说明

这是一个用于演示BUS30 BCU平台满充均衡算法的Web应用程序，**直接调用真实的C代码**进行计算。

## 文件结构

```
满充均衡web展示/
├── balance_calc.c          # C源代码（提取自原始ApiSetBalanceFlag）
├── balance_calc.exe        # 编译后的可执行程序
├── server.js               # Node.js服务器
├── index_with_c.html       # 调用C程序的Web界面
├── index.html              # 纯JavaScript版本（备用）
├── config.json             # 配置参数JSON文件
├── 启动服务器.bat          # 一键启动脚本
└── README.md               # 说明文档
```

## 使用方法

### 方式1：调用真实C代码（推荐）

这种方式直接调用编译后的C程序，确保计算逻辑100%准确。

#### 步骤：

1. **确保已安装Node.js**
   - 如果没有安装，从 https://nodejs.org/ 下载安装

2. **启动服务器**
   - 双击 `启动服务器.bat`
   - 或在命令行中运行：`node server.js`

3. **打开浏览器**
   - 自动打开：http://localhost:3000
   - 或手动访问该地址

4. **使用界面**
   - 调整输入参数（最大单体电压、单体电压差）
   - 点击"计算结果"按钮
   - 查看从真实C代码返回的结果

### 方式2：纯JavaScript版本（无需服务器）

如果不想启动服务器，可以直接打开 `index.html`，使用JavaScript模拟的逻辑（可能与C代码有细微差异）。

## C程序说明

### balance_calc.c

这是从原始 `ApiSetBalanceFlag` 函数提取的核心逻辑，包含：

- **完整的分支逻辑**（3个主要分支）
- **calculateBalanceTime 函数**（根据uBalancingSoc计算持续时间）
- **所有配置参数**（可通过修改C代码重新编译）

### 编译命令

如果修改了C代码，重新编译：

```bash
gcc balance_calc.c -o balance_calc.exe
```

### 测试C程序

可以直接在命令行测试：

```bash
balance_calc.exe <voltMax> <voltDiff>

示例：
balance_calc.exe 3450 30
balance_calc.exe 3600 100
balance_calc.exe 3700 50
```

输出JSON格式结果：
```json
{
  "uBalanceDeltVolt": 20,
  "guContinueTime": 49,
  "uBalancingSoc": 2,
  "ucVoltPerMillSoc": 0,
  "isCleared": 0
}
```

## 算法逻辑

程序实现了ApiSetBalanceFlag函数的三个主要分支：

### 分支1: voltMax < BALANCE_CHGVOLTLVL0 (3450 mV)
- uBalanceDeltVolt = BALANCE_CHGDELTVOLT0 (20 mV)
- 如果 voltDiff > uBalanceDeltVolt:
  - guContinueTime = calculateBalanceTime(BALANCE_SOC_MIN)
  - uBalancingSoc = BALANCE_SOC_MIN (2)
  - ucVoltPerMillSoc = 0 (空)

### 分支2: BALANCE_CHGVOLTLVL0 <= voltMax < BALANCE_CHGVOLTLVL1 (3450-3500 mV)
- uBalanceDeltVolt = 线性插值 (20-40 mV)
- 如果 voltDiff > uBalanceDeltVolt:
  - guContinueTime = calculateBalanceTime(BALANCE_SOC_MIN)
  - uBalancingSoc = BALANCE_SOC_MIN (2)
  - ucVoltPerMillSoc = 0 (空)

### 分支3: voltMax >= BALANCE_CHGVOLTLVL1 (3500 mV+)
- uBalanceDeltVolt = 线性插值 (40-140 mV)，在3650 mV达到最大值140 mV
- 如果 voltDiff > uBalanceDeltVolt:
  - ucVoltPerMillSoc = 线性插值 (6-10 mV)
  - uBalancingSoc = 根据压差和ucVoltPerMillSoc计算
  - guContinueTime = calculateBalanceTime(uBalancingSoc)

### calculateBalanceTime 函数

```c
BALANCE_CONTINUE_AS = PRO_CELL_CAPACITY * SOH * 6 * 6 / 10 / 1000
BALANCE_CONTINUE_PRECESION = 5

guContinueTime = (BALANCE_CONTINUE_AS * uBalanceSoc + BALANCE_CONTINUE_PRECESION/2) / BALANCE_CONTINUE_PRECESION
```

默认参数：
- PRO_CELL_CAPACITY = 340 Ah
- SOH = 100%
- BALANCE_CONTINUE_AS = 122

示例计算：
- uBalancingSoc = 2 → guContinueTime = 49 分钟
- uBalancingSoc = 7 → guContinueTime = 171 分钟

## 配置参数

所有参数都在 `balance_calc.c` 中定义，可以修改后重新编译：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| PRO_CHG_CELL_VOLT_MAX | 3650 mV | LFP电池充电电压上限 |
| PRO_CELL_CAPACITY | 340 Ah | 电池容量 |
| SOH_VALUE | 100% | 电池健康状态 |
| BALANCE_CHGVOLTLVL0 | 3450 mV | 电压阈值0 |
| BALANCE_CHGVOLTLVL1 | 3500 mV | 电压阈值1 |
| BALANCE_CHGVOLTPERSOCLVL1 | 6 mV | 3500mV时每0.1%SOC电压变化 |
| BALANCE_CHGVOLTPERSOCLVL2 | 10 mV | 3650mV时每0.1%SOC电压变化 |
| BALANCE_CHGDELTVOLT0 | 20 mV | 等级0均衡压差 |
| BALANCE_CHGDELTVOLT1 | 40 mV | 等级1均衡压差 |
| BALANCE_CHGDELTVOLT2 | 140 mV | 充电电压上限时均衡压差 |
| BALANCE_SOC_MIN | 2 | 最小SOC阈值(0.1%SOC) |
| BALANCE_SOC_NORMAL | 4 | 正常SOC阈值(0.1%SOC) |
| BALANCE_SOC_MAX | 7 | 最大SOC阈值(0.1%SOC) |
| BALANCE_TIME_MAX | 1920 min | 最大均衡时间(32小时) |

## 技术实现

- **后端**：Node.js + child_process（调用C程序）
- **前端**：HTML + CSS + JavaScript（纯原生，无框架）
- **核心计算**：编译的C代码（balance_calc.exe）
- **通信方式**：HTTP RESTful API（JSON格式）

## 浏览器兼容性

- Chrome (推荐)
- Edge
- Firefox
- Safari

## 故障排除

### 问题1：服务器启动失败
- **原因**：未安装Node.js
- **解决**：从 https://nodejs.org/ 下载安装

### 问题2：计算错误
- **原因**：balance_calc.exe找不到
- **解决**：确保balance_calc.exe在同一目录

### 问题3：页面无法访问
- **原因**：端口3000被占用
- **解决**：修改server.js中的PORT值

## 开发信息

- **项目路径**: `C:\REPT\git\main\0_MyFiles\Projects\满充均衡web展示`
- **开发日期**: 2026-09-08
- **版本**: 2.0（调用真实C代码）

## 优势

✅ **100%准确** - 直接调用原始C代码逻辑  
✅ **易于验证** - 可以单独测试C程序  
✅ **易于维护** - C代码更新后只需重新编译  
✅ **实时交互** - Web界面友好直观  
✅ **无需修改原代码** - 独立的演示环境  

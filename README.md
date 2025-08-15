# 2025年全国大学生起重机创意大赛项目

本项目是为 "2025年全国大学生起重机创意大赛" 准备的备赛代码。

仓库地址: [Eternity-burial/crane_25_dev](https://github.com/Eternity-burial/crane_25_dev)

## ⚠️ 重要：开发环境配置

为了保证代码的兼容性和团队协作的统一性，本项目使用统一的开发环境。

完整的环境搭建教程、工具和脚本，请**参考**以下仓库的说明进行配置：

👉 **[环境配置参考：TongjiSuperPower/stm32_dev_env](https://github.com/TongjiSuperPower/stm32_dev_env)**

该环境基于 `VS Code + STM32CubeMX`，能够实现跨平台的开发。在开始之前，请务必先按照上述链接的指引搭好环境。

---

## 快速使用

1.  **克隆仓库**
    ```bash
    git clone -b dev2 https://github.com/Eternity-burial/crane_25_dev.git
    git submodule add https://github.com/TongjiSuperPower/sp_middleware.git
    cd crane_25_dev
    ```

2.  **编译**
    在 VS Code 中按下 F7 ：
    
    编译成功后，会在 `build` 目录下生成  `.elf` 文件。

3.  **烧录**
    连接好 cmsis-Link 后，在 VS Code 中按下 F8
    
## 硬件平台

*   **主控**: [STM32F407IGH6]
*   **电机**: [M3508 P19直流无刷减速电机]

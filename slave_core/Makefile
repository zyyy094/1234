# Makefile - 从核裸机程序构建
#
# 基于 Phytium Standalone SDK 的 openamp_for_linux 示例
#
# 用法:
#   make config_phytiumpi_aarch64   # 加载飞腾派 aarch64 配置
#   make                            # 编译
#   make image                      # 编译并复制 ELF
#   make clean                      # 清理
#
# 前提: 需要设置 SDK_DIR 环境变量指向 phytium-standalone-sdk 目录

# SDK 路径（相对于本文件的位置）
SDK_DIR ?= ../phytium-standalone-sdk

# 源文件
SRC = main.c \
      hardware_control.c \
      rpmsg_handler.c

# 头文件搜索路径
INC = -I. \
      -I$(SDK_DIR)/example/system/amp/openamp_for_linux/common \
      -I$(SDK_DIR)/example/system/amp/openamp_for_linux/inc

# 导出给 SDK 构建系统的变量
export EXTRA_SRC := $(SRC)
export EXTRA_INC := $(INC)

# 引入 SDK 的构建系统
include $(SDK_DIR)/example/system/amp/openamp_for_linux/makefile

# 飞腾派 aarch64 配置快捷命令
config_phytiumpi_aarch64:
	$(MAKE) -C $(SDK_DIR)/example/system/amp/openamp_for_linux \
		load_kconfig LOAD_CONFIG_NAME=pe2204_aarch64_phytiumpi_openamp_core0

# 部署到飞腾派
deploy: image
	scp output/openamp_core0.elf user@192.168.43.231:/lib/firmware/

.PHONY: config_phytiumpi_aarch64 deploy

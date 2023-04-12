cmake_minimum_required(VERSION 3.15)

project(UART_TX_RX_Test LANGUAGES C CXX ASM)

find_bsp(
	ID com.sysprogs.project_importers.stm32.cubemx
	SOURCE_PROJECT UART_TX_RX_Test.ioc
	HWREGISTER_LIST_FILE STM32F767.mcudef)

add_bsp_based_executable(
	NAME UART_TX_RX_Test
	GENERATE_BIN
	GENERATE_MAP)


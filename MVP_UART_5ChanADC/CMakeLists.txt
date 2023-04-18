cmake_minimum_required(VERSION 3.15)

project(MVP_UART_5ChanADC LANGUAGES C CXX ASM)

find_bsp(ID com.sysprogs.project_importers.stm32.cubemx SOURCE_PROJECT MVP_UART_5ChanADC.ioc)

add_bsp_based_executable(NAME MVP_UART_5ChanADC
	GENERATE_BIN
	GENERATE_MAP)


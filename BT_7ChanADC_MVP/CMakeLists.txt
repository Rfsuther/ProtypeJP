cmake_minimum_required(VERSION 3.15)

project(BT_7ChanADC_MVP LANGUAGES C CXX ASM)

find_bsp(ID com.sysprogs.project_importers.stm32.cubemx SOURCE_PROJECT BT_7ChanADC_MVP.ioc)

add_bsp_based_executable(NAME BT_7ChanADC_MVP
	GENERATE_BIN
	GENERATE_MAP)


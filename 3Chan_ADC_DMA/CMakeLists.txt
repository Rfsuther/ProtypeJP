cmake_minimum_required(VERSION 3.15)

project(3Chan_ADC_DMA LANGUAGES C CXX ASM)

find_bsp(ID com.sysprogs.project_importers.stm32.cubemx SOURCE_PROJECT 3Chan_ADC_DMA.ioc)

add_bsp_based_executable(NAME 3Chan_ADC_DMA
	GENERATE_BIN
	GENERATE_MAP)


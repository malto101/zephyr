# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.28.0)

find_package(Zephyr COMPONENTS unittest REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(lemans_tlb)

target_sources(testbinary PRIVATE main.c)

target_include_directories(testbinary PRIVATE
  ${ZEPHYR_BASE}/soc/qcom/lemans/include
)


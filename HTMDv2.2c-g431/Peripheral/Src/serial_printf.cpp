/**
 * @file serial_printf.cpp
 * @author  (8gn24gn25@gmail.com)
 * @brief UARTを使用したフォーマットされた文字列の送信
 * @version 0.1
 * @date 2025-05-06
 *
 * @copyright Copyright (c) 2025
 *
 */
#include "serial_printf.hpp"

template void serial_printf<>(const std::string &fmt);
template void serial_printf<unsigned char, unsigned char, unsigned char, unsigned char, unsigned char, unsigned char>(const std::string &fmt, unsigned char, unsigned char, unsigned char, unsigned char, unsigned char, unsigned char);
template void serial_printf<unsigned char, unsigned char, unsigned char, unsigned char>(const std::string &fmt, unsigned char, unsigned char, unsigned char, unsigned char);
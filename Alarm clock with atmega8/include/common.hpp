#pragma once

#define set_bit(byte, bit) (byte) |= (1 << (bit))
#define clear_bit(byte, bit) (byte) &= ~(1 << (bit))
#define write_bit(byte, bit, value) (value) == 0 ? clear_bit((byte), (bit)) : set_bit((byte), (bit))
#define read_bit(byte, bit) (((byte) & (1 << bit)) > 0)
#define toggle_bit(byte, bit) read_bit((byte), (bit)) ? write_bit(byte, bit, 0) : write_bit(byte, bit, 1)
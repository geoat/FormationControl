#pragma once
template <typename T>
union Converter {
    T m_value;
    uint8_t m_bytes[sizeof(T)];
};

union Float {
    float m_float;
    uint8_t m_bytes[sizeof(float)];
};
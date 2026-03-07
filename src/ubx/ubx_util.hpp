#pragma once

// UBX binary protocol decoders use memcpy into __attribute__((packed)) structs.
// This relies on the host being little-endian (UBX wire format is little-endian).
// Fail loudly at compile time on big-endian or mixed-endian targets.
static_assert(__BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__,
    "UBX parsers require a little-endian host: "
    "memcpy into packed structs produces wrong results on big-endian systems.");
